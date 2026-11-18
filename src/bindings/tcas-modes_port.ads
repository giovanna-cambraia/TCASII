--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/bindings/tcas-modes_port.ads
--  Module      : Ada view of the C Mode S reply frame (specification only)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : boundary between P1 (C driver) and P2 (SPARK parser)
--  Requirements: REQ-001, REQ-002, REQ-004 (placeholder IDs)
--
--  PURPOSE
--    The C driver (the future tcas_modes_drv.h) receives a Mode S reply
--    from the transceiver, checks its length, and leaves one frame in
--    memory: the reply bytes, the round-trip time and the bearing that the
--    transceiver measured. This package gives the SPARK parser the same
--    record with the same layout, so it can read the frame in place,
--    without a copy. It also holds the check that the parser runs FIRST,
--    before it trusts anything: Is_Well_Formed tells whether the C side
--    kept its own promises.
--
--    This is the second untrusted boundary (ASM-005), next to the ARINC 429
--    one. Everything that arrives here is untrusted until Is_Well_Formed
--    is True; after that, the parser works with typed values only.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure package: no state, no body. Constants, one array type, one
--        record type and one expression function.
--    R2  The layout is the C layout, byte for byte: Convention C, an
--        explicit representation clause with byte offsets, and a size
--        check that stops the compilation if the record is not 24 bytes.
--    R3  Bytes stay bytes. Status, Length, Valid and Reserved are
--        Unsigned_8, not enumeration types or ranges, because the C code
--        can store any byte there and an out-of-range value is an invalid
--        value in Ada. Is_Well_Formed checks that each one is in range.
--    R4  Every code and constant has the value of its C counterpart, and
--        the comment names it. A cross-check test (to be written, like
--        check_a429_layout.c) compares both sides.
--    R5  Is_Well_Formed has Global => null and no side effects, so it can
--        be used in preconditions of the parser.
--    R6  FAIL CLOSED. A frame that the driver rejected carries NOTHING:
--        no bytes, no length, no time, no bearing. A garbage value is
--        never published, even by accident (the same rule as R7 of the
--        ARINC 429 driver). A short frame also carries zero in every
--        byte after its length, so the parser never sees stale data.
--
--  BIT NUMBERING
--    Mode S bits are numbered from 1, and bit 1 is the most significant
--    bit of Data (0). So Data (0) holds bits 1 .. 8, Data (1) holds bits
--    9 .. 16, and so on. The format code (DF) is bits 1 .. 5.
--
--  LAYOUT (must equal tcas_modes_frame_t in tcas_modes_drv.h, 24 bytes)
--    offset  size  field
--       0     14   Data           reply bytes, first byte first
--      14      1   Length         7 (short) or 14 (long) when valid, else 0
--      15      1   Status         OK or an ERR code, 0 .. 6
--      16      4   Round_Trip_Ns  nanoseconds, see ROUND TRIP below
--      20      2   Bearing        hundredths of a degree, 0 .. 35 999
--      22      1   Valid          1 valid, 0 invalid
--      23      1   Reserved       always 0
--    [VERIFY] The offsets assume 4-byte alignment of uint32_t and no
--    padding, true for the Cortex-R52 and x86_64 hosts. The C header will
--    have a static assertion on the size; the offsets are checked by the
--    cross-check test.
--
--  ROUND TRIP
--    Round_Trip_Ns is the time from the end of our interrogation to the
--    start of the reply, measured by the transceiver hardware. The parser
--    subtracts the fixed transponder reply delay and converts the rest to
--    a slant range. [VERIFY] the exact reference points of both
--    timestamps with the transceiver ICD (RISK-001).
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * No deviations are requested by this specification.

pragma SPARK_Mode (On);

with Interfaces;

package Tcas.Modes_Port
  with Pure
is

   subtype U8  is Interfaces.Unsigned_8;
   subtype U16 is Interfaces.Unsigned_16;
   subtype U32 is Interfaces.Unsigned_32;

   use type U8;
   use type U16;
   use type U32;

   --  Frame sizes

   --  Short Mode S message: 56 bits. Long Mode S message: 112 bits.
   Short_Length : constant U8 := 7;
   Long_Length  : constant U8 := 14;

   subtype Frame_Index is Natural range 0 .. 13;

   --  The reply bytes. A short frame uses the first 7; the rest are 0.
   type Frame_Bytes is array (Frame_Index) of U8
     with Convention => C;

   --  Status codes (placeholders until tcas_modes_drv.h is written)

   Status_OK             : constant U8 := 0;
   Status_Err_Null       : constant U8 := 1;
   Status_Err_Config     : constant U8 := 2;
   Status_Err_Length     : constant U8 := 3;   --  not 7 or 14 bytes
   Status_Err_No_Reply   : constant U8 := 4;   --  nothing received in time
   Status_Err_Timestamp  : constant U8 := 5;   --  no hardware timestamp
   Status_Err_Overrun    : constant U8 := 6;   --  frames were lost

   --  Bearing

   --  Largest bearing: 359.99 degrees (Tcas.Types.Bearing_Cdeg'Last).
   Bearing_Max : constant U16 := 35_999;

   --  The reply frame (tcas_modes_frame_t in the C header)

   type Frame is record
      Data          : Frame_Bytes;
      Length        : U8;
      Status        : U8;
      Round_Trip_Ns : U32;
      Bearing       : U16;
      Valid         : U8;
      Reserved      : U8;
   end record
     with Convention => C;

   for Frame use record
      Data          at  0 range 0 .. 111;
      Length        at 14 range 0 .. 7;
      Status        at 15 range 0 .. 7;
      Round_Trip_Ns at 16 range 0 .. 31;
      Bearing       at 20 range 0 .. 15;
      Valid         at 22 range 0 .. 7;
      Reserved      at 23 range 0 .. 7;
   end record;

   for Frame'Size use 192;

   --  The record must be exactly 24 bytes (rule R2). If it is not,
   --  compilation stops with the message below.
   pragma Compile_Time_Error
     (Frame'Size /= 192, "Frame must be 24 bytes, as in the C header");

   --  Trust check (rule R5)

   --  True when the C driver kept every promise:
   --    * Reserved is 0, Valid is 0 or 1, Status is 0 .. 6
   --    * Valid = 1 exactly when Status = OK
   --    * a valid frame is 7 or 14 bytes long, has a bearing in range, and
   --      has 0 in every byte after its length
   --    * an invalid frame carries nothing (rule R6): length 0, time 0,
   --      bearing 0 and every byte 0
   --  The parser runs this first. A frame that fails it is treated as
   --  invalid, whatever it says about itself.
   function Is_Well_Formed (F : Frame) return Boolean is
     (F.Reserved = 0
      and then F.Status <= Status_Err_Overrun
      and then F.Valid <= 1
      and then
        (if F.Valid = 1
         then F.Status = Status_OK
              and then (F.Length = Short_Length
                        or else F.Length = Long_Length)
              and then F.Bearing <= Bearing_Max
              and then
                (for all I in Frame_Index =>
                   (if I >= Integer (F.Length) then F.Data (I) = 0))
         else F.Status /= Status_OK
              and then F.Length = 0
              and then F.Round_Trip_Ns = 0
              and then F.Bearing = 0
              and then (for all I in Frame_Index => F.Data (I) = 0)))
   with Global => null;

end Tcas.Modes_Port;