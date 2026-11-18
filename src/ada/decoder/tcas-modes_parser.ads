--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/decoder/tcas-modes_parser.ads
--  Module      : Mode S reply parser, "parse then act" gate (specification)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P2 PROTOCOL_DECODING
--  Requirements: REQ-001, REQ-002 (placeholder IDs)
--
--  PURPOSE
--    This is the second untrusted boundary (ASM-005), next to the ARINC 429
--    decoder. The C driver hands over one reply frame, seen here as
--    Tcas.Modes_Port.Frame, together with the address of the aircraft we
--    interrogated. This package turns the frame into a Tr.Measurement that
--    the tracker can take, or into a reason for rejecting it. After this
--    point no unit works with raw reply bits.
--
--    Only replies to our own addressed interrogations are accepted: DF 0
--    (short air-to-air reply, 56 bits) and DF 16 (long air-to-air reply,
--    112 bits). Acquisition squitters (DF 11) and ADS-B (DF 17) are not
--    parsed in this version.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  No package state. Every function has Global => null.
--    R2  Nothing is trusted before Is_Well_Formed (Tcas.Modes_Port) says
--        so. A frame that fails it is Malformed_Frame, whatever it claims.
--    R3  Fail closed. On any failure the measurement is Zero_Measurement:
--        a garbage value is never published (the same rule as the C
--        drivers and the ARINC 429 decoder).
--    R4  Address by parity. In an addressed reply the last 24 bits (the
--        AP field) hold the CRC of the message XOR the aircraft address.
--        The parser recovers the address and requires it to equal the
--        address we interrogated. A reply meant for another aircraft, or
--        corrupted on the way, fails here (security analysis: spoofing).
--    R5  Rejected, never clamped. A round trip that gives a range above
--        150 000 ft is Bad_Timing, not a saturated range. A clamp would
--        turn a faulty timestamp into a plausible position.
--    R6  Bounded execution. The only loops are in CRC24 (at most 11 bytes
--        of 8 bits). No recursion. The cost of one parse is constant.
--    R7  Integer arithmetic only, with truncation toward zero, so results
--        are bit-exact on host and target. The range conversion runs in
--        64 bits (the largest product is about 2.1E17).
--
--  ORDER OF CHECKS IN Parse (the first failure wins, so the reported cause
--  is stable)
--    1 Is_Well_Formed (F)                          Malformed_Frame
--    2 F.Valid = 1                                 Driver_Rejected
--    3 Has_Supported_Format (F)                    Unsupported_Format
--    4 Recovered_Address (F) = Expected            Wrong_Address
--    5 altitude code present, 25 ft coding         No_Altitude /
--                                                  Unsupported_Altitude
--    6 round trip gives a range in 0 .. 150 000    Bad_Timing
--
--  BIT LAYOUT USED (Mode S bits are numbered from 1; bit 1 is the most
--  significant bit of Data (0))
--    bits  1 ..  5   DF, downlink format
--    bits 20 .. 32   AC, altitude code (DF 0 and DF 16):
--                    C1 A1 C2 A2 C4 A4 M B1 Q B2 D2 B4 D4
--                    M (bit 26) = 0 : feet.  Q (bit 28) = 1 : 25 ft steps.
--                    With M = 0 and Q = 1 the other 11 bits form N, and the
--                    altitude is N * 25 - 1 000 ft (N = 0: no altitude).
--    last 24 bits    AP, address / parity
--
--  KNOWN SIMPLIFICATIONS (open items for v2)
--    No Gillham (100 ft) altitude coding. No DF 11 or DF 17. The vertical
--    status bit (bit 6, airborne or on the ground) is ignored. Range comes
--    from the round-trip time only; the reply delay is a single fixed
--    value ([VERIFY] in Tcas.Config).
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * The bodies are in tcas-modes_parser.adb; GNATprove proves the
--      postconditions and the absence of runtime errors there.
--    * No deviations are requested by this specification.

pragma SPARK_Mode (On);

with Tcas.Modes_Port;
with Tcas.Tracker;
with Tcas.Types; use Tcas.Types;

package Tcas.Modes_Parser is

   package Port renames Tcas.Modes_Port;
   package Tr   renames Tcas.Tracker;

   use type Port.U8;
   use type Port.U16;
   use type Port.U32;
   use type Tr.Measurement;

   --  Protocol constants

   --  Downlink formats accepted (replies to UF 0 and UF 16).
   DF_Short_Air_Air : constant Port.U8 := 0;
   DF_Long_Air_Air  : constant Port.U8 := 16;

   --  Mode S CRC-24 generator polynomial, without its x ** 24 term.
   CRC_Polynomial : constant Port.U32 := 16#FFF409#;

   --  Results

   type Parse_Status is
     (Parsed,                --  the value is valid
      Malformed_Frame,       --  the C side broke one of its own promises
      Driver_Rejected,       --  the driver marked the frame invalid
      Unsupported_Format,    --  not DF 0 / 7 bytes or DF 16 / 14 bytes
      Wrong_Address,         --  parity does not give the address asked
      No_Altitude,           --  the altitude code is 0
      Unsupported_Altitude,  --  metric, or Gillham (100 ft) coding
      Bad_Timing);           --  round trip impossible or out of range

   type Altitude_Result is record
      Status : Parse_Status;
      Value  : Altitude_Ft;
   end record;

   type Range_Result is record
      Status : Parse_Status;
      Value  : Range_Ft;
   end record;

   type Parse_Result is record
      Status : Parse_Status;
      Value  : Tr.Measurement;
   end record;

   --  The measurement published on every failure (rule R3).
   Zero_Measurement : constant Tr.Measurement :=
     (Address  => 0,
      Slant    => 0,
      Bearing  => 0,
      Altitude => 0);

   --  Building blocks (public so that the tests can check them alone)

   --  Downlink format: bits 1 .. 5.
   function Format_Of (F : Port.Frame) return Port.U8 is
     (F.Data (0) / 8)
   with
     Global => null,
     Post   => Format_Of'Result <= 31;

   --  DF 0 with a short frame, or DF 16 with a long frame.
   function Has_Supported_Format (F : Port.Frame) return Boolean is
     ((Format_Of (F) = DF_Short_Air_Air
       and then F.Length = Port.Short_Length)
      or else
        (Format_Of (F) = DF_Long_Air_Air
         and then F.Length = Port.Long_Length))
   with Global => null;

   --  CRC-24 of the message without its last 3 bytes (the AP field).
   --  Bitwise, most significant bit first, polynomial CRC_Polynomial.
   function CRC24 (F : Port.Frame) return Port.U32
   with
     Global => null,
     Pre    => F.Length = Port.Short_Length
               or else F.Length = Port.Long_Length,
     Post   => CRC24'Result <= 16#FF_FFFF#;

   --  The last 3 bytes of the message (the AP field), as a 24-bit number.
   function Parity_Field (F : Port.Frame) return Port.U32 is
     (Port.U32 (F.Data (Natural (F.Length) - 3)) * 65_536
      + Port.U32 (F.Data (Natural (F.Length) - 2)) * 256
      + Port.U32 (F.Data (Natural (F.Length) - 1)))
   with
     Global => null,
     Pre    => F.Length = Port.Short_Length
               or else F.Length = Port.Long_Length,
     Post   => Parity_Field'Result <= 16#FF_FFFF#;

   --  The aircraft address hidden in the AP field (rule R4).
   function Recovered_Address (F : Port.Frame) return Mode_S_Address is
     (Mode_S_Address (CRC24 (F) xor Parity_Field (F)))
   with
     Global => null,
     Pre    => F.Length = Port.Short_Length
               or else F.Length = Port.Long_Length;

   --  The 13-bit altitude code AC, bits 20 .. 32 (DF 0 and DF 16).
   function Altitude_Field (F : Port.Frame) return Port.U16 is
     (Port.U16 (F.Data (2) and 16#1F#) * 256 + Port.U16 (F.Data (3)))
   with
     Global => null,
     Post   => Altitude_Field'Result <= 16#1FFF#;

   --  Altitude of a 13-bit code. Parsed only for M = 0, Q = 1 and N > 0.
   --  Value = 0 whenever Status is not Parsed (rule R3).
   function Decode_Altitude (AC : Port.U16) return Altitude_Result
   with
     Global => null,
     Pre    => AC <= 16#1FFF#,
     Post   =>
       Decode_Altitude'Result.Status
         in Parsed | No_Altitude | Unsupported_Altitude
       and then
         (if Decode_Altitude'Result.Status /= Parsed
          then Decode_Altitude'Result.Value = 0);

   --  Slant range from the round-trip time: subtract the reply delay,
   --  convert at c / 2. Bad_Timing when the time is shorter than the delay
   --  or the range is above Range_Ft'Last (rule R5).
   --  Value = 0 whenever Status is not Parsed (rule R3).
   function Range_From_Round_Trip (Round_Trip_Ns : Port.U32)
     return Range_Result
   with
     Global => null,
     Post   =>
       Range_From_Round_Trip'Result.Status in Parsed | Bad_Timing
       and then
         (if Range_From_Round_Trip'Result.Status /= Parsed
          then Range_From_Round_Trip'Result.Value = 0);

   --  Parse

   --  Turns one frame into one measurement (see ORDER OF CHECKS).
   --  Expected is the address of the aircraft that was interrogated.
   function Parse (F : Port.Frame; Expected : Mode_S_Address)
     return Parse_Result
   with
     Global => null,
     Post   =>
       --  every failure publishes the zero measurement (rule R3)
       (if Parse'Result.Status /= Parsed
        then Parse'Result.Value = Zero_Measurement)
       --  a success has passed every check of the specification
       and then
         (if Parse'Result.Status = Parsed
          then Port.Is_Well_Formed (F)
               and then F.Valid = 1
               and then Has_Supported_Format (F)
               and then Parse'Result.Value.Address = Expected
               and then Natural (Parse'Result.Value.Bearing)
                          = Natural (F.Bearing));

end Tcas.Modes_Parser;