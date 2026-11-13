--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/decoder/tcas-a429_decoder.adb
--  Module      : ARINC 429 decoder, "parse then act" gate (body)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P2 PROTOCOL_DECODING
--  Requirements: REQ-001, REQ-002 (placeholder IDs)
--
--  PURPOSE
--    Implements the three decoders declared in tcas-a429_decoder.ads. The
--    specification holds the contract (rules, order of checks, BNR
--    decoding); this body holds the code that makes it true. Read the
--    specification first.
--
--  DESIGN RULES (as in the specification, with the way each is met here)
--    R1  No state: no variable outlives a call.
--    R2  Is_Well_Formed is the first check of every decoder.
--    R3  The scaled value is compared with the range of the target type
--        BEFORE the conversion, so the conversion can never fail, and an
--        out-of-range value is rejected, not clamped.
--    R4  The divisor is a parameter of type Positive, so every division is
--        proved free of division by zero by its type.
--    R5  Each result starts as (Malformed_Word, 0). A failing branch only
--        changes the status, so the value stays 0 by construction. Only
--        the single success branch writes a value.
--    R6  Defense in depth: SSM = Normal is checked again here, even though
--        the C driver only marks such words valid.
--
--  STRUCTURE OF THE CODE
--    Every function has ONE exit (a single return at the end) and a flat
--    if / elsif chain, one branch per check, in the order of the
--    specification. This keeps the decisions simple for MC/DC coverage
--    (DO-178C Table A-7): each condition is a separate, testable branch.
--
--    Decode_Altitude is shared by the pressure and the radio altitude
--    decoders, which differ only in label and divisor. It is local to this
--    body and carries its own postcondition, from which the postconditions
--    of both public decoders follow.
--
--  PROOF OBLIGATIONS (discharged by GNATprove, tcas2.gpr level 2)
--    * Scale precondition (Data <= Data_Max): from Is_Well_Formed on a
--      valid word.
--    * Range checks of the three conversions: from the explicit range test
--      before each of them.
--    * The three postconditions of the specification.
--
--  SPARK NOTES
--    * SPARK_Mode is On.
--    * No deviations are requested by this body.
--  ===========================================================================

pragma SPARK_Mode (On);

package body Tcas.A429_Decoder is

   --  Shared altitude decoder (local to this body)

   function Decode_Altitude
     (W       : Port.Word;
      Label   : Port.U8;
      Divisor : Positive) return Altitude_Result
   with
     Global => null,
     Post   =>
       (if Decode_Altitude'Result.Status /= Decoded
        then Decode_Altitude'Result.Value = 0)
       and then
       (if Decode_Altitude'Result.Status = Decoded
        then Port.Is_Well_Formed (W)
             and then W.Valid = 1
             and then W.Label = Label);

   function Decode_Altitude
     (W       : Port.Word;
      Label   : Port.U8;
      Divisor : Positive) return Altitude_Result
   is
      Result : Altitude_Result := (Status => Malformed_Word, Value => 0);
   begin
      if not Port.Is_Well_Formed (W) then
         null;   --  check 1: the C side broke a promise (Malformed_Word)
      elsif W.Valid /= 1 then
         Result.Status := Driver_Rejected;                   --  check 2
      elsif W.Ssm /= Port.Ssm_Normal then
         null;   --  check 3: valid word, abnormal SSM (Malformed_Word)
      elsif W.Label /= Label then
         Result.Status := Wrong_Label;                       --  check 4
      else
         declare
            Scaled : constant Integer := Scale (W.Data, Divisor);
         begin
            if Scaled < Integer (Tcas.Types.Altitude_Ft'First)
              or else Scaled > Integer (Tcas.Types.Altitude_Ft'Last)
            then
               Result.Status := Out_Of_Range;                --  check 5
            else
               Result :=
                 (Status => Decoded,
                  Value  => Tcas.Types.Altitude_Ft (Scaled));
            end if;
         end;
      end if;

      return Result;
   end Decode_Altitude;


   --  Public decoders

   function Decode_Pressure_Altitude (W : Port.Word)
     return Altitude_Result is
     (Decode_Altitude
        (W       => W,
         Label   => Port.Label_Pressure_Alt,
         Divisor => Pressure_Alt_Divisor));

   function Decode_Radio_Altitude (W : Port.Word)
     return Altitude_Result is
     (Decode_Altitude
        (W       => W,
         Label   => Port.Label_Radio_Alt,
         Divisor => Radio_Alt_Divisor));

   function Decode_Altitude_Rate (W : Port.Word)
     return Vertical_Rate_Result
   is
      Result : Vertical_Rate_Result :=
        (Status => Malformed_Word, Value => 0);
   begin
      if not Port.Is_Well_Formed (W) then
         null;   --  check 1: the C side broke a promise (Malformed_Word)
      elsif W.Valid /= 1 then
         Result.Status := Driver_Rejected;                   --  check 2
      elsif W.Ssm /= Port.Ssm_Normal then
         null;   --  check 3: valid word, abnormal SSM (Malformed_Word)
      elsif W.Label /= Port.Label_Alt_Rate then
         Result.Status := Wrong_Label;                       --  check 4
      else
         declare
            Scaled : constant Integer := Scale (W.Data, Alt_Rate_Divisor);
         begin
            if Scaled < Integer (Tcas.Types.Vertical_Rate_Fpm'First)
              or else Scaled > Integer (Tcas.Types.Vertical_Rate_Fpm'Last)
            then
               Result.Status := Out_Of_Range;                --  check 5
            else
               Result :=
                 (Status => Decoded,
                  Value  => Tcas.Types.Vertical_Rate_Fpm (Scaled));
            end if;
         end;
      end if;

      return Result;
   end Decode_Altitude_Rate;

end Tcas.A429_Decoder;