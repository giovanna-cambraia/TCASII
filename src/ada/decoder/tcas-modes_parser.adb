--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/decoder/tcas-modes_parser.adb
--  Module      : Mode S reply parser, "parse then act" gate (body)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P2 PROTOCOL_DECODING
--  Requirements: REQ-001, REQ-002 (placeholder IDs)
--
--  PURPOSE
--    Implements CRC24, Decode_Altitude, Range_From_Round_Trip and Parse,
--    declared in tcas-modes_parser.ads. The other building blocks are
--    expression functions in the specification. The specification holds
--    the contract (rules, order of checks, bit layout); this body holds
--    the code that makes it true. Read the specification first.
--
--  DESIGN RULES (as in the specification, with the way each is met here)
--    R1  No state: no variable outlives a call.
--    R2  Is_Well_Formed is the first check of Parse.
--    R3  Parse starts from (Malformed_Frame, Zero_Measurement). A failing
--        branch only changes the status, so the value stays zero by
--        construction. Only the single success branch writes a value.
--        Decode_Altitude and Range_From_Round_Trip follow the same
--        pattern.
--    R4  The address comes from the parity: Recovered_Address (CRC XOR
--        AP field) must equal the address we interrogated.
--    R5  The range is checked against Range_Ft'Last BEFORE the conversion,
--        so the conversion can never fail, and an impossible range is
--        rejected, not clamped.
--    R6  Bounded execution. CRC24 has two nested loops: at most 11 bytes,
--        8 bits each. No other loop, no recursion.
--    R7  Integer arithmetic only. The range runs in Long_Long_Integer: the
--        largest product is 4 294 967 295 * 49 178 553, about 2.1E17,
--        far below the 64-bit limit.
--
--  CRC-24 (Mode S parity)
--    Generator polynomial x**24 + (CRC_Polynomial), that is 16#1FFF409#.
--    The register starts at 0 and takes the message bits most significant
--    bit first. For each bit: feedback = (top bit of the register) XOR
--    (message bit); the register shifts left by one, stays in 24 bits,
--    and the polynomial is XORed in when the feedback is 1. The message
--    covers every byte except the last 3 (the AP field), so 4 bytes for a
--    short frame and 11 for a long one. For a DF 17 message the result
--    equals the last 3 bytes; for an addressed reply (DF 0, DF 16) the
--    result XOR the AP field is the aircraft address.
--    [VERIFY] against a captured message of a real transponder: the
--    unit test uses a published DF 17 example, but it is not a
--    substitute for a test against the actual transceiver.
--
--  ALTITUDE CODE (13 bits, bit 12 is the most significant)
--    bit: 12 11 10  9  8  7  6  5  4  3  2  1  0
--         C1 A1 C2 A2 C4 A4  M B1  Q B2 D2 B4 D4
--    M = 0 and Q = 1: the other 11 bits form N, with the bits taken in
--    order from C1 down to A4 (6 bits), then B1 (1 bit), then B2 to D4
--    (4 bits). Altitude = N * 25 - 1 000 ft. N lies in 0 .. 2 047, so the
--    result is -975 .. 50 175 ft, always inside Altitude_Ft.
--    Decisions, in order: code 0 is No_Altitude; M = 1 or Q = 0 is
--    Unsupported_Altitude (metric or Gillham coding); N = 0 is
--    No_Altitude (the specification says "N = 0: no altitude"; in the
--    standard it would be -1 000 ft, open item for v2); anything else
--    is Parsed.
--
--  STRUCTURE OF THE CODE
--    CRC24, Decode_Altitude, Range_From_Round_Trip and Parse, in that
--    order. Every function has ONE exit (a single return at the end) and
--    a flat if / elsif chain, one branch per check, in the order of the
--    specification, so each condition is a separate, testable branch for
--    MC/DC coverage (DO-178C Table A-7).
--
--  PROOF OBLIGATIONS (discharged by GNATprove, tcas2.gpr level 2)
--    * Absence of runtime errors: the index in CRC24 is at most 10 (the
--      precondition gives a length of 7 or 14), every conversion is
--      inside its range (the altitude from N in 1 .. 2 047, the range
--      from the explicit test, the bearing from Is_Well_Formed on a valid
--      frame), every shift amount is in 0 .. 31.
--    * The loop invariants of CRC24 (the register stays in 24 bits).
--    * The preconditions of the calls inside Parse (the Length check of
--      Has_Supported_Format gives the 7 or 14 that CRC24, Parity_Field
--      and Recovered_Address need).
--    * The postconditions of CRC24, Decode_Altitude,
--      Range_From_Round_Trip and Parse.
--
--  SPARK NOTES
--    * SPARK_Mode is On.
--    * The with and use clauses of the specification apply to this body,
--      so Tcas.Modes_Port, Tcas.Tracker and Tcas.Types are visible
--      without repeating them here.
--    * No deviations are requested by this body.

pragma SPARK_Mode (On);

with Interfaces;
with Tcas.Config;

package body Tcas.Modes_Parser is

   --  CRC-24

   function CRC24 (F : Port.Frame) return Port.U32 is
      --  index of the last byte of the message, before the AP field
      Last : constant Natural := Natural (F.Length) - 4;
      Reg  : Port.U32 := 0;
   begin
      for I in 0 .. Last loop
         pragma Loop_Invariant (Reg <= 16#FF_FFFF#);

         declare
            Byte : constant Port.U32 := Port.U32 (F.Data (I));
         begin
            for B in 0 .. 7 loop
               pragma Loop_Invariant (Reg <= 16#FF_FFFF#);

               declare
                  --  the bit of the byte, most significant first
                  Bit : constant Port.U32 :=
                    Interfaces.Shift_Right (Byte, 7 - B) and 1;
                  --  the top bit of the 24-bit register
                  Top : constant Port.U32 :=
                    Interfaces.Shift_Right (Reg, 23) and 1;
               begin
                  Reg := (Reg * 2) and 16#FF_FFFF#;

                  if (Top xor Bit) = 1 then
                     Reg := Reg xor CRC_Polynomial;
                  end if;
               end;
            end loop;
         end;
      end loop;

      return Reg;
   end CRC24;

   --  Altitude

   function Decode_Altitude (AC : Port.U16) return Altitude_Result is
      A : constant Integer := Integer (AC);

      M_Bit : constant Integer := (A / 64) mod 2;    --  bit 6
      Q_Bit : constant Integer := (A / 16) mod 2;    --  bit 4

      --  the 11 bits that are not M and Q: C1 .. A4 (bits 12 .. 7),
      --  B1 (bit 5), B2 .. D4 (bits 3 .. 0)
      N : constant Integer :=
        (A / 128) * 32 + ((A / 32) mod 2) * 16 + (A mod 16);

      Result : Altitude_Result :=
        (Status => Unsupported_Altitude, Value => 0);
   begin
      if AC = 0 then
         Result.Status := No_Altitude;                       --  code 0
      elsif M_Bit = 1 or else Q_Bit = 0 then
         null;   --  metric or Gillham coding: Unsupported_Altitude
      elsif N = 0 then
         Result.Status := No_Altitude;                       --  N = 0
      else
         Result :=
           (Status => Parsed,
            Value  => Altitude_Ft (N * 25 - 1_000));
      end if;

      return Result;
   end Decode_Altitude;

   --  Range

   function Range_From_Round_Trip (Round_Trip_Ns : Port.U32)
     return Range_Result
   is
      Delay_Ns : constant Long_Long_Integer :=
        Tcas.Config.Mode_S_Reply_Delay_Ns;

      Result : Range_Result := (Status => Bad_Timing, Value => 0);
   begin
      if Long_Long_Integer (Round_Trip_Ns) < Delay_Ns then
         null;   --  shorter than the reply delay: Bad_Timing
      else
         declare
            --  time of flight there and back, in nanoseconds
            Net : constant Long_Long_Integer :=
              Long_Long_Integer (Round_Trip_Ns) - Delay_Ns;

            --  one way, in feet, truncated toward zero (rule R7)
            Feet : constant Long_Long_Integer :=
              (Net * Tcas.Config.Ft_Per_Ns_Num) / Tcas.Config.Ft_Per_Ns_Den;
         begin
            if Feet > Long_Long_Integer (Range_Ft'Last) then
               null;   --  impossible range: Bad_Timing, not clamped (R5)
            else
               Result :=
                 (Status => Parsed,
                  Value  => Range_Ft (Feet));
            end if;
         end;
      end if;

      return Result;
   end Range_From_Round_Trip;

   --  Parse

   function Parse (F : Port.Frame; Expected : Mode_S_Address)
     return Parse_Result
   is
      Result : Parse_Result :=
        (Status => Malformed_Frame, Value => Zero_Measurement);
   begin
      if not Port.Is_Well_Formed (F) then
         null;   --  check 1: the C side broke a promise (Malformed_Frame)
      elsif F.Valid /= 1 then
         Result.Status := Driver_Rejected;                   --  check 2
      elsif not Has_Supported_Format (F) then
         Result.Status := Unsupported_Format;                --  check 3
      elsif Recovered_Address (F) /= Expected then
         Result.Status := Wrong_Address;                     --  check 4
      else
         declare
            Alt : constant Altitude_Result :=
              Decode_Altitude (Altitude_Field (F));
            Rng : constant Range_Result :=
              Range_From_Round_Trip (F.Round_Trip_Ns);
         begin
            if Alt.Status /= Parsed then
               Result.Status := Alt.Status;                  --  check 5
            elsif Rng.Status /= Parsed then
               Result.Status := Rng.Status;                  --  check 6
            else
               Result :=
                 (Status => Parsed,
                  Value  =>
                    (Address  => Expected,
                     Slant    => Rng.Value,
                     Bearing  => Bearing_Cdeg (F.Bearing),
                     Altitude => Alt.Value));
            end if;
         end;
      end if;

      return Result;
   end Parse;

end Tcas.Modes_Parser;