--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/decoder/tcas-a429_decoder.ads
--  Module      : ARINC 429 decoder, "parse then act" gate (specification)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P2 PROTOCOL_DECODING
--  Requirements: REQ-001, REQ-002 (placeholder IDs)
--
--  PURPOSE
--    This is the second half of the trust boundary. The C driver (P1)
--    hands over a tcas_a429_word_t, seen here as Tcas.A429_Port.Word. This
--    package turns it into a typed physical value or into a reason for
--    rejecting it. After this point no unit works with raw bits.
--
--    Three decoders are provided, one per label the system accepts:
--    pressure altitude, radio altitude and altitude rate. Each returns a
--    result record with a status and a value; the value is meaningful only
--    when the status is Decoded.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure package, no state. Every function has Global => null.
--    R2  Nothing is trusted before Is_Well_Formed (Tcas.A429_Port) says
--        so. A word that fails it is Malformed_Word, whatever it claims.
--    R3  Out-of-range values are REJECTED, never clamped. A clamp would
--        turn a sensor fault into a plausible number.
--    R4  Division truncates toward zero (Ada "/" on Integer), the same
--        rule on host and target, so results are bit-exact between lanes.
--        Every divisor is a positive literal, so no division can fail.
--    R5  On any failure Value = 0 (rule R7 of the C header, repeated
--        here): a garbage value is never published.
--    R6  Defense in depth: the C driver only passes SSM = Normal when it
--        marks a word valid. This package checks that again; a valid
--        word with any other SSM is Malformed_Word.
--
--  ORDER OF CHECKS (the first failure wins, so the reported cause is stable)
--    1 Is_Well_Formed (W)                         Malformed_Word
--    2 W.Valid = 1                                Driver_Rejected
--    3 W.Ssm = Ssm_Normal                         Malformed_Word
--    4 W.Label is the label of this decoder       Wrong_Label
--    5 scaled value inside the range of the type  Out_Of_Range
--
--  BNR DECODING (all values [VERIFY] against the installation ICD)
--    The 19-bit data field is two's complement, bit 18 is the sign.
--    Value = (sign-extended field) / Divisor, truncated toward zero.
--      label 203 pressure altitude : Divisor  2, 0.5 ft per count
--      label 164 radio altitude    : Divisor 32, 1/32 ft per count
--      label 212 altitude rate     : Divisor  8, 1/8 ft/min per count
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * The bodies are in tcas-a429_decoder.adb; GNATprove proves the
--      postconditions and the absence of runtime errors there.
--    * No deviations are requested by this specification.
--  ===========================================================================

pragma SPARK_Mode (On);

with Tcas.Types;
with Tcas.A429_Port;

package Tcas.A429_Decoder
  with Pure
is

   package Port renames Tcas.A429_Port;

   use type Port.U8;
   use type Port.U32;
   use type Tcas.Types.Altitude_Ft;
   use type Tcas.Types.Vertical_Rate_Fpm;

   --  ------------------------------------------------------------------
   --  Result of a decode
   --  ------------------------------------------------------------------

   type Decode_Status is
     (Decoded,           --  value is valid and inside its range
      Malformed_Word,    --  the C side broke one of its own promises
      Driver_Rejected,   --  the driver marked the word invalid
      Wrong_Label,       --  not the label this decoder handles
      Out_Of_Range);     --  scaled value outside the range of the type

   type Altitude_Result is record
      Status : Decode_Status;
      Value  : Tcas.Types.Altitude_Ft;
   end record;

   type Vertical_Rate_Result is record
      Status : Decode_Status;
      Value  : Tcas.Types.Vertical_Rate_Fpm;
   end record;

   --  ------------------------------------------------------------------
   --  BNR helpers (public so that the tests can check them alone)
   --  ------------------------------------------------------------------

   --  The 19-bit field is two's complement: the span is 2 ** 19 and the
   --  sign bit has the value 2 ** 18.
   Field_Span : constant := 524_288;
   Sign_Value : constant := 262_144;

   --  Signed value of a 19-bit field.
   function Sign_Extend (Raw : Port.U32) return Integer is
     (if Raw >= Port.U32 (Sign_Value)
      then Integer (Raw) - Field_Span
      else Integer (Raw))
   with
     Global => null,
     Pre    => Raw <= Port.Data_Max,
     Post   => Sign_Extend'Result in -Sign_Value .. Sign_Value - 1;

   --  Signed field divided by a positive divisor, truncated toward zero.
   function Scale (Raw : Port.U32; Divisor : Positive) return Integer is
     (Sign_Extend (Raw) / Divisor)
   with
     Global => null,
     Pre    => Raw <= Port.Data_Max;

   --  Divisors per label (see the table in the header).
   Pressure_Alt_Divisor : constant Positive := 2;
   Radio_Alt_Divisor    : constant Positive := 32;
   Alt_Rate_Divisor     : constant Positive := 8;

   --  ------------------------------------------------------------------
   --  Decoders
   --  ------------------------------------------------------------------

   --  Label 203. Returns Decoded only for a well-formed, valid word with
   --  this label and a value inside Altitude_Ft.
   function Decode_Pressure_Altitude (W : Port.Word)
     return Altitude_Result
   with
     Global => null,
     Post   =>
       (if Decode_Pressure_Altitude'Result.Status /= Decoded
        then Decode_Pressure_Altitude'Result.Value = 0)
       and then
       (if Decode_Pressure_Altitude'Result.Status = Decoded
        then Port.Is_Well_Formed (W)
             and then W.Valid = 1
             and then W.Label = Port.Label_Pressure_Alt);

   --  Label 164. Radio altitude is height above ground; it uses the same
   --  result type, and the same range, as pressure altitude.
   function Decode_Radio_Altitude (W : Port.Word)
     return Altitude_Result
   with
     Global => null,
     Post   =>
       (if Decode_Radio_Altitude'Result.Status /= Decoded
        then Decode_Radio_Altitude'Result.Value = 0)
       and then
       (if Decode_Radio_Altitude'Result.Status = Decoded
        then Port.Is_Well_Formed (W)
             and then W.Valid = 1
             and then W.Label = Port.Label_Radio_Alt);

   --  Label 212.
   function Decode_Altitude_Rate (W : Port.Word)
     return Vertical_Rate_Result
   with
     Global => null,
     Post   =>
       (if Decode_Altitude_Rate'Result.Status /= Decoded
        then Decode_Altitude_Rate'Result.Value = 0)
       and then
       (if Decode_Altitude_Rate'Result.Status = Decoded
        then Port.Is_Well_Formed (W)
             and then W.Valid = 1
             and then W.Label = Port.Label_Alt_Rate);

end Tcas.A429_Decoder;