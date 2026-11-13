--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/bindings/tcas-a429_port.ads
--  Module      : Ada view of the C ARINC 429 output word (specification only)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : boundary between P1 (C driver) and P2 (SPARK decoder)
--  Requirements: REQ-001, REQ-002, REQ-004 (placeholder IDs)
--
--  PURPOSE
--    The C driver (tcas_a429_drv.h) leaves one tcas_a429_word_t per
--    received word. This package gives the SPARK decoder the same record
--    with the same layout, so it can read the word in place, without a
--    copy. It also holds the checks that the decoder runs FIRST, before
--    it trusts anything: Is_Well_Formed tells whether the C side kept its
--    own promises (rule R7 of the C header).
--
--    This is the border of the trust boundary (ASM-005). Everything that
--    arrives here is untrusted until Is_Well_Formed is True; after that,
--    the decoder works with typed values only.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure package: no state, no body. Constants, one record type and
--        one expression function.
--    R2  The layout is the C layout, byte for byte: Convention C, an
--        explicit representation clause with byte offsets, and a size
--        check that stops the compilation if the record is not 16 bytes.
--    R3  Bytes stay bytes. Status, Ssm, Valid and the rest are
--        Unsigned_8, not enumeration types, because the C code can store
--        any byte there and an out-of-range enumeration value is an
--        invalid value in Ada. The decoder reads the byte, and
--        Is_Well_Formed checks that it is in range.
--    R4  Every code and constant has the value of its C counterpart, and
--        the comment names it. A change in one place needs a change in the
--        other; the C unit tests and the cross-check named below catch a
--        mismatch.
--    R5  Is_Well_Formed has Global => null and no side effects, so it can
--        be used in preconditions of the decoder.
--
--  LAYOUT (must equal tcas_a429_word_t in tcas_a429_drv.h, 16 bytes)
--    offset  size  field
--       0      4   Data        19-bit BNR value, right-justified, or 0
--       4      4   Frame       cycle number of the read
--       8      1   Label       label value after bit reversal
--       9      1   Sdi         0 .. 3
--      10      1   Ssm         0 .. 3
--      11      1   Status      OK or an ERR code, 0 .. 9
--      12      1   Valid       1 valid, 0 invalid
--      13      1   Reserved8   always 0
--      14      2   Reserved16  always 0
--    [VERIFY] The offsets assume 4-byte alignment of uint32_t and no
--    padding, true for the Cortex-R52 and x86_64 hosts. The C header has a
--    static assertion on the size; the offsets are checked by
--    tests/unit/c (cross-check test, to be written).
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * No deviations are requested by this specification.
--  ===========================================================================

pragma SPARK_Mode (On);

with Interfaces;

package Tcas.A429_Port
  with Pure
is

   subtype U8  is Interfaces.Unsigned_8;
   subtype U16 is Interfaces.Unsigned_16;
   subtype U32 is Interfaces.Unsigned_32;

   use type U8;
   use type U16;
   use type U32;

   --  Status codes (TCAS_A429_OK and TCAS_A429_ERR_* in the C header)

   Status_OK          : constant U8 := 0;
   Status_Err_Null    : constant U8 := 1;
   Status_Err_Config  : constant U8 := 2;
   Status_Err_Parity  : constant U8 := 3;
   Status_Err_Label   : constant U8 := 4;
   Status_Err_Ssm_Fw  : constant U8 := 5;   --  failure warning
   Status_Err_Ssm_Ncd : constant U8 := 6;   --  no computed data
   Status_Err_Ssm_Ft  : constant U8 := 7;   --  functional test
   Status_Err_Overrun : constant U8 := 8;
   Status_Err_Backlog : constant U8 := 9;

   --  Sign/status matrix, BNR coding (TCAS_A429_SSM_* in the C header)

   Ssm_Failure_Warning  : constant U8 := 0;
   Ssm_No_Computed_Data : constant U8 := 1;
   Ssm_Functional_Test  : constant U8 := 2;
   Ssm_Normal           : constant U8 := 3;

   --  Labels (TCAS_A429_LABEL_* in the C header)
   --  placeholders until the Interface Control Document is written
   --  [VERIFY] each value against the ARINC 429 ICD of the installation

   Label_Pressure_Alt : constant U8 := 16#83#;   --  octal 203, altitude
   Label_Radio_Alt    : constant U8 := 16#74#;   --  octal 164, radio altitude
   Label_Alt_Rate     : constant U8 := 16#8A#;   --  octal 212, altitude rate

   --  Data field

   --  Largest value of the 19-bit BNR data field.
   Data_Max : constant U32 := 16#7_FFFF#;

   --  The output word (tcas_a429_word_t in the C header)

   type Word is record
      Data       : U32;
      Frame      : U32;
      Label      : U8;
      Sdi        : U8;
      Ssm        : U8;
      Status     : U8;
      Valid      : U8;
      Reserved8  : U8;
      Reserved16 : U16;
   end record
     with Convention => C;

   for Word use record
      Data       at  0 range 0 .. 31;
      Frame      at  4 range 0 .. 31;
      Label      at  8 range 0 .. 7;
      Sdi        at  9 range 0 .. 7;
      Ssm        at 10 range 0 .. 7;
      Status     at 11 range 0 .. 7;
      Valid      at 12 range 0 .. 7;
      Reserved8  at 13 range 0 .. 7;
      Reserved16 at 14 range 0 .. 15;
   end record;

   for Word'Size use 128;

   --  The record must be exactly 16 bytes (rule R2). If it is not,
   --  compilation stops with the message below.
   pragma Compile_Time_Error
     (Word'Size /= 128, "Word must be 16 bytes, as in the C header");

   --  trust check (rule R5)

   --  True when the C driver kept every promise of its rule R7:
   --    * reserved fields are 0
   --    * Sdi and Ssm are 0 .. 3, Status is 0 .. 9, Valid is 0 or 1
   --    * Valid = 1 exactly when Status = OK
   --    * a valid word has a 19-bit Data value
   --    * an invalid word has Data = 0 (garbage is never published)
   --  The decoder runs this first. A word that fails it is treated as
   --  invalid, whatever it says about itself.
   function Is_Well_Formed (W : Word) return Boolean is
     (W.Reserved8 = 0
      and then W.Reserved16 = 0
      and then W.Sdi <= 3
      and then W.Ssm <= 3
      and then W.Status <= Status_Err_Backlog
      and then W.Valid <= 1
      and then
        (if W.Valid = 1
         then W.Status = Status_OK and then W.Data <= Data_Max
         else W.Status /= Status_OK and then W.Data = 0))
   with Global => null;

end Tcas.A429_Port;