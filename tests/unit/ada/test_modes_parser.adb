--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : tests/unit/ada/test_modes_parser.adb
--  Module      : Unit tests of the Mode S parser (plain Ada program)
--  Language    : Ada 2012 (the tests are not in the SPARK subset)
--  Assurance   : DO-178C Software Level B (target), verification artifact
--  Tests       : src/ada/decoder/tcas-modes_parser.ads and .adb
--                src/bindings/tcas-modes_port.ads
--  Requirements: REQ-001, REQ-002 (placeholder IDs)
--
--  PURPOSE
--    Checks every behavior promised in tcas-modes_parser.ads: the trust
--    check of the frame, the field helpers, the CRC-24, the address
--    recovery, the altitude code, the range from the round trip, and the
--    six checks of Parse in their order. GNATprove proves the absence of
--    runtime errors and the postconditions; these tests check that the
--    bits and the numbers are the right ones.
--
--  DESIGN RULES OF THE TESTS
--    T1  The CRC is checked against THREE things that do not come from
--        the code under test:
--          a) a published DF 17 message (the "1090 MHz Riddle" example),
--             whose AP field equals the CRC of the rest of the message;
--          b) two values worked out by hand: a single set bit in the last
--             message byte gives x**24 mod G, which is the polynomial
--             itself (16#FFF409#), and the next bit gives 16#001C1B#;
--          c) Ref_CRC, a second implementation written as long division
--             over a bit array, a different algorithm from the one in the
--             parser. It is checked against (a) and (b) first, then used
--             to build every test frame.
--        Properties that need no oracle are checked too: every single-bit
--        error is detected, and the CRC is linear.
--    T2  Altitude codes and round-trip times are worked out by hand and
--        written next to each check. The sweeps check structure (counts,
--        grid, monotonic steps), not a second copy of the formula.
--    T3  Values are written as literals taken from Tcas.Config (all
--        [VERIFY]): reply delay 128 000 ns, 49 178 553 / 100 000 000 ft
--        per nanosecond (c / 2). When the table is corrected in v2, these
--        tests must change with it, and that is intended.
--    T4  Every failing parse is checked for the zero measurement as well
--        as for its status (rule R3 of the specification).
--    T5  Every test builds its own frames, so tests do not depend on each
--        other or on their order.
--    T6  Output: one line per failed check, then a summary. The exit
--        status is 1 if anything failed, else 0, so a script can use it.
--
--  HAND ARITHMETIC USED BELOW
--    Range: feet = (round trip - 128 000) * 49 178 553 / 100 000 000,
--           truncated toward zero.
--      net 0 -> 0, net 2 -> 0, net 3 -> 1 (147 535 659 / 1E8)
--      net 10 000  -> 4 917      net 12 345 -> 6 071
--      net 20 000  -> 9 835      net 100 000 -> 49 178
--      net 305 013 -> 150 000 (150 000.98)  net 305 014 -> 150 001 (rejected)
--    Altitude code (13 bits, bit 12 first): C1 A1 C2 A2 C4 A4 M B1 Q B2 D2
--      B4 D4. M is bit 6, Q is bit 4. N = bits 12..7 (6 bits), then B1,
--      then bits 3..0; altitude = N * 25 - 1 000 ft.
--      16#06B8#: N = 440 -> 10 000 ft    16#1838#: N = 1 560 -> 38 000 ft
--      16#0011#: N = 1 -> -975 ft        16#0030#: N = 16 -> -600 ft
--      16#0090#: N = 32 -> -200 ft       16#1FBF#: N = 2 047 -> 50 175 ft
--    Address by parity: for the DF 17 example, AP = 16#576098# and the CRC
--      is 16#576098#, so the recovered address is 0. With AP = 16#FCAD77#
--      the recovered address is 16#576098# xor 16#FCAD77# = 16#ABCDEF#.
--
--  TESTS BY GROUP
--    trust check   : Port.Is_Well_Formed, one rule at a time
--    fields        : format, supported format, parity field, altitude field
--    crc           : vector, hand values, reference, single-bit errors,
--                    random frames, linearity
--    address       : recovered address (hand and built frames)
--    altitude      : hand codes, all 8 192 codes
--    range         : hand values, boundaries, 500 001 round-trip times
--    parse         : success, each failure, order of checks, bearing
--    sweep         : 1 680 frames checked against the order of checks
--
--  USAGE (from the project root; no project file is needed)
--    mkdir -p build/obj_test build/bin
--    gnatmake tests/unit/ada/test_modes_parser.adb -gnat2012 -gnatwa \
--        -D build/obj_test -o build/bin/test_modes_parser.exe \
--        -Isrc/ada/common -Isrc/ada/common/types \
--        -Isrc/ada/common/config -Isrc/ada/common/units \
--        -Isrc/ada/tracker -Isrc/ada/decoder -Isrc/bindings
--    build/bin/test_modes_parser.exe
--
--  NOTES
--    * tcas2.gpr does not list tests/, so the project build never sees
--      this program (it has a main procedure and is not part of the
--      product).
--    * If only the DF 17 vector check fails, check the vector before the
--      parser: the hand values and Ref_CRC are independent of it.

with Ada.Command_Line;
with Ada.Text_IO;

with Tcas.Modes_Parser;
with Tcas.Modes_Port;
with Tcas.Types;

procedure Test_Modes_Parser is

   package Mp   renames Tcas.Modes_Parser;
   package Port renames Tcas.Modes_Port;
   package Ty   renames Tcas.Types;

   use type Mp.Parse_Status;
   use type Port.U32;

   --  Test bookkeeping

   Total  : Natural := 0;
   Failed : Natural := 0;

   procedure Check (Name : String; Condition : Boolean) is
   begin
      Total := Total + 1;
      if not Condition then
         Failed := Failed + 1;
         Ada.Text_IO.Put_Line ("FAIL: " & Name);
      end if;
   end Check;

   --  Constants

   Exp_Addr   : constant Natural := 16#ABCDEF#;   --  the address we asked
   Wrong_Addr : constant Natural := 16#ABCDE0#;   --  a different aircraft

   Good_AC      : constant Natural := 16#06B8#;   --  10 000 ft
   Good_RTT     : constant Long_Long_Integer := 148_000;   --  9 835 ft
   Good_Slant   : constant Natural := 9_835;
   Good_Alt     : constant Integer := 10_000;
   Good_Bearing : constant Natural := 9_000;

   --  The published DF 17 example. The last 3 bytes (16#576098#) are the
   --  CRC of the first 11.
   V17 : constant Port.Frame :=
     (Data          => (16#8D#, 16#48#, 16#40#, 16#D6#, 16#20#, 16#2C#,
                        16#C3#, 16#71#, 16#C3#, 16#2C#, 16#E0#, 16#57#,
                        16#60#, 16#98#),
      Length        => 14,
      Status        => 0,
      Round_Trip_Ns => 0,
      Bearing       => 0,
      Valid         => 1,
      Reserved      => 0);

   --  Reference CRC (rule T1 c): long division over a bit array, a
   --  different algorithm from the register method of the parser

   --  Bit K of the generator polynomial 16#1FFF409#, K = 0 is x ** 24.
   function Gen_Bit (K : Natural) return Boolean is
     ((16#1FFF409# / 2 ** (24 - K)) mod 2 = 1);

   type Bit_Array is array (0 .. 111) of Boolean;

   --  CRC-24 of the first N bytes of Bytes, N in 0 .. 11.
   function Ref_CRC (Bytes : Port.Frame_Bytes; N : Natural) return Natural is
      Bits  : Bit_Array := (others => False);
      Total : constant Natural := N * 8;
      R     : Natural := 0;
   begin
      --  the message bits, most significant first, then 24 zero bits
      for I in 0 .. Total - 1 loop
         Bits (I) :=
           (Natural (Bytes (I / 8)) / 2 ** (7 - I mod 8)) mod 2 = 1;
      end loop;

      --  divide: wherever a bit is set, subtract (XOR) the generator
      for I in 0 .. Total - 1 loop
         if Bits (I) then
            for K in 0 .. 24 loop
               if Gen_Bit (K) then
                  Bits (I + K) := not Bits (I + K);
               end if;
            end loop;
         end if;
      end loop;

      --  the remainder is what is left in the last 24 bits
      for I in Total .. Total + 23 loop
         R := R * 2 + (if Bits (I) then 1 else 0);
      end loop;

      return R;
   end Ref_CRC;

   --  Frame builders

   --  A frame of the given length with every data byte 0.
   function Blank (Len : Natural) return Port.Frame is
     (Data          => (others => 0),
      Length        => Port.U8 (Len),
      Status        => Port.Status_OK,
      Round_Trip_Ns => 0,
      Bearing       => 0,
      Valid         => 1,
      Reserved      => 0);

   --  A frame the driver rejected: invalid, with nothing in it.
   function Rejected (Status : Natural) return Port.Frame is
     (Data          => (others => 0),
      Length        => 0,
      Status        => Port.U8 (Status),
      Round_Trip_Ns => 0,
      Bearing       => 0,
      Valid         => 0,
      Reserved      => 0);

   --  A valid reply of aircraft Addr: format DF, altitude code AC, the
   --  AP field built with the REFERENCE CRC (so the parser's own CRC is
   --  checked against it by every Parse call).
   function Build
     (DF      : Natural;
      Long    : Boolean;
      AC      : Natural;
      Addr    : Natural;
      RTT     : Long_Long_Integer;
      Bearing : Natural) return Port.Frame
   is
      Len : constant Natural := (if Long then 14 else 7);
      N   : constant Natural := Len - 3;
      F   : Port.Frame :=
        (Data          => (others => 0),
         Length        =>
           (if Long then Port.Long_Length else Port.Short_Length),
         Status        => Port.Status_OK,
         Round_Trip_Ns => Port.U32 (RTT),
         Bearing       => Port.U16 (Bearing),
         Valid         => 1,
         Reserved      => 0);
      AP : Port.U32;
   begin
      F.Data (0) := Port.U8 (DF * 8);
      F.Data (2) := Port.U8 (AC / 256);
      F.Data (3) := Port.U8 (AC mod 256);

      if Long then
         for I in 4 .. 10 loop
            F.Data (I) := Port.U8 (16#A0# + I);
         end loop;
      end if;

      AP := Port.U32 (Ref_CRC (F.Data, N)) xor Port.U32 (Addr);
      F.Data (N)     := Port.U8 (AP / 65_536);
      F.Data (N + 1) := Port.U8 ((AP / 256) mod 256);
      F.Data (N + 2) := Port.U8 (AP mod 256);

      return F;
   end Build;

   --  Expectation helpers

   procedure Expect_Fail
     (Name   : String;
      F      : Port.Frame;
      Status : Mp.Parse_Status)
   is
      R : constant Mp.Parse_Result :=
        Mp.Parse (F, Ty.Mode_S_Address (Exp_Addr));
   begin
      Check (Name & " (expected " & Mp.Parse_Status'Image (Status)
             & ", got " & Mp.Parse_Status'Image (R.Status) & ")",
             R.Status = Status);
      Check (Name & ": zero measurement",
             Natural (R.Value.Address) = 0
             and then Natural (R.Value.Slant) = 0
             and then Natural (R.Value.Bearing) = 0
             and then Integer (R.Value.Altitude) = 0);
   end Expect_Fail;

   procedure Expect_Parsed
     (Name    : String;
      F       : Port.Frame;
      Slant   : Natural;
      Bearing : Natural;
      Alt     : Integer)
   is
      R : constant Mp.Parse_Result :=
        Mp.Parse (F, Ty.Mode_S_Address (Exp_Addr));
   begin
      Check (Name & ": status (got " & Mp.Parse_Status'Image (R.Status)
             & ")",
             R.Status = Mp.Parsed);
      Check (Name & ": address",  Natural (R.Value.Address) = Exp_Addr);
      Check (Name & ": slant",    Natural (R.Value.Slant) = Slant);
      Check (Name & ": bearing",  Natural (R.Value.Bearing) = Bearing);
      Check (Name & ": altitude", Integer (R.Value.Altitude) = Alt);
   end Expect_Parsed;

   --  Group: trust check

   procedure Test_Well_Formed is
      V : constant Port.Frame :=
        Build (0, False, Good_AC, Exp_Addr, Good_RTT, Good_Bearing);
      L : constant Port.Frame :=
        Build (16, True, Good_AC, Exp_Addr, Good_RTT, Good_Bearing);
      W : Port.Frame;
   begin
      Check ("trust: valid short frame", Port.Is_Well_Formed (V));
      Check ("trust: valid long frame",  Port.Is_Well_Formed (L));
      Check ("trust: DF 17 vector",      Port.Is_Well_Formed (V17));

      W := V;  W.Reserved := 1;
      Check ("trust: reserved set", not Port.Is_Well_Formed (W));

      W := V;  W.Valid := 2;
      Check ("trust: valid is 2", not Port.Is_Well_Formed (W));

      W := V;  W.Status := 7;
      Check ("trust: status 7", not Port.Is_Well_Formed (W));

      W := V;  W.Status := Port.Status_Err_Length;
      Check ("trust: valid with an error status",
             not Port.Is_Well_Formed (W));

      W := V;  W.Length := 8;
      Check ("trust: valid with length 8", not Port.Is_Well_Formed (W));

      W := V;  W.Length := 0;
      Check ("trust: valid with length 0", not Port.Is_Well_Formed (W));

      W := V;  W.Data (7) := 1;
      Check ("trust: short frame, byte after the length (7)",
             not Port.Is_Well_Formed (W));

      W := V;  W.Data (13) := 1;
      Check ("trust: short frame, byte after the length (13)",
             not Port.Is_Well_Formed (W));

      W := V;  W.Bearing := 36_000;
      Check ("trust: bearing 36000", not Port.Is_Well_Formed (W));

      W := Build (0, False, Good_AC, Exp_Addr, Good_RTT, 35_999);
      Check ("trust: bearing 35999", Port.Is_Well_Formed (W));

      --  frames the driver rejected carry nothing (rule R6)
      for S in 1 .. 6 loop
         Check ("trust: properly rejected frame, status"
                & Integer'Image (S),
                Port.Is_Well_Formed (Rejected (S)));
      end loop;

      Check ("trust: invalid with status OK",
             not Port.Is_Well_Formed (Rejected (0)));
      Check ("trust: invalid with status 7",
             not Port.Is_Well_Formed (Rejected (7)));

      W := Rejected (3);  W.Length := 7;
      Check ("trust: invalid with a length", not Port.Is_Well_Formed (W));

      W := Rejected (3);  W.Round_Trip_Ns := 1;
      Check ("trust: invalid with a round trip",
             not Port.Is_Well_Formed (W));

      W := Rejected (3);  W.Bearing := 1;
      Check ("trust: invalid with a bearing", not Port.Is_Well_Formed (W));

      W := Rejected (3);  W.Data (0) := 1;
      Check ("trust: invalid with data", not Port.Is_Well_Formed (W));

      W := Rejected (3);  W.Reserved := 1;
      Check ("trust: invalid with reserved set",
             not Port.Is_Well_Formed (W));
   end Test_Well_Formed;

   --  Group: field helpers

   procedure Expect_Format (Byte : Natural; DF : Natural) is
      F : Port.Frame := Blank (7);
   begin
      F.Data (0) := Port.U8 (Byte);
      Check ("format: first byte" & Natural'Image (Byte) & " is DF"
             & Natural'Image (DF),
             Natural (Mp.Format_Of (F)) = DF);
   end Expect_Format;

   function Supported (Byte : Natural; Len : Natural) return Boolean is
      F : Port.Frame := Blank (Len);
   begin
      F.Data (0) := Port.U8 (Byte);
      return Mp.Has_Supported_Format (F);
   end Supported;

   function Alt_Field (B2 : Natural; B3 : Natural) return Natural is
      F : Port.Frame := Blank (7);
   begin
      F.Data (2) := Port.U8 (B2);
      F.Data (3) := Port.U8 (B3);
      return Natural (Mp.Altitude_Field (F));
   end Alt_Field;

   procedure Test_Fields is
      S : Port.Frame := Blank (7);
      L : Port.Frame := Blank (14);
   begin
      --  format: bits 1 .. 5, the low 3 bits of the first byte are ignored
      Expect_Format (16#00#, 0);
      Expect_Format (16#07#, 0);
      Expect_Format (16#08#, 1);
      Expect_Format (16#58#, 11);
      Expect_Format (16#80#, 16);
      Expect_Format (16#87#, 16);
      Expect_Format (16#88#, 17);
      Expect_Format (16#8D#, 17);
      Expect_Format (16#F8#, 31);
      Expect_Format (16#FF#, 31);

      --  supported: DF 0 with 7 bytes, DF 16 with 14 bytes, nothing else
      Check ("supported: DF 0, 7 bytes",     Supported (16#00#, 7));
      Check ("supported: DF 0 low bits, 7",  Supported (16#07#, 7));
      Check ("supported: DF 16, 14 bytes",   Supported (16#80#, 14));
      Check ("supported: DF 16 low bits, 14", Supported (16#87#, 14));
      Check ("not supported: DF 0, 14 bytes", not Supported (16#00#, 14));
      Check ("not supported: DF 16, 7 bytes", not Supported (16#80#, 7));
      Check ("not supported: DF 17, 14 bytes", not Supported (16#88#, 14));
      Check ("not supported: DF 17, 7 bytes",  not Supported (16#88#, 7));
      Check ("not supported: DF 11, 7 bytes",  not Supported (16#58#, 7));

      --  parity field: the last 3 bytes, whatever is before them
      S.Data (3) := 16#FF#;
      S.Data (4) := 16#12#;  S.Data (5) := 16#34#;  S.Data (6) := 16#56#;
      Check ("parity: short frame 123456",
             Natural (Mp.Parity_Field (S)) = 16#123456#);

      S.Data (4) := 16#FF#;  S.Data (5) := 16#FF#;  S.Data (6) := 16#FF#;
      Check ("parity: short frame FFFFFF",
             Natural (Mp.Parity_Field (S)) = 16#FFFFFF#);

      S.Data (4) := 0;  S.Data (5) := 0;  S.Data (6) := 0;
      Check ("parity: short frame zero", Natural (Mp.Parity_Field (S)) = 0);

      for I in 4 .. 10 loop
         L.Data (I) := 16#FF#;
      end loop;
      L.Data (11) := 16#AB#;  L.Data (12) := 16#CD#;  L.Data (13) := 16#EF#;
      Check ("parity: long frame ABCDEF",
             Natural (Mp.Parity_Field (L)) = 16#ABCDEF#);

      Check ("parity: DF 17 vector",
             Natural (Mp.Parity_Field (V17)) = 16#576098#);

      --  altitude field: bits 20 .. 32, the top 3 bits of byte 2 are not
      --  part of it
      Check ("altitude field: all zero",   Alt_Field (16#00#, 16#00#) = 0);
      Check ("altitude field: all ones",   Alt_Field (16#FF#, 16#FF#) = 16#1FFF#);
      Check ("altitude field: top bits ignored",
             Alt_Field (16#E0#, 16#00#) = 0);
      Check ("altitude field: 1F FF",      Alt_Field (16#1F#, 16#FF#) = 16#1FFF#);
      Check ("altitude field: 06 B8",      Alt_Field (16#06#, 16#B8#) = 16#06B8#);
      Check ("altitude field: 18 38",      Alt_Field (16#18#, 16#38#) = 16#1838#);
      Check ("altitude field: 20 01",      Alt_Field (16#20#, 16#01#) = 1);
   end Test_Fields;

   --  Group: CRC-24

   --  Pseudo-random bytes (a linear congruential generator): the same
   --  frames on every run, on every machine.
   procedure Fill
     (F     : in out Port.Frame;
      Len   : Natural;
      State : in out Long_Long_Integer) is
   begin
      for I in 0 .. Len - 1 loop
         State := (State * 1_103_515_245 + 12_345) mod 2_147_483_648;
         F.Data (I) := Port.U8 ((State / 65_536) mod 256);
      end loop;
   end Fill;

   procedure Test_CRC is
      State : Long_Long_Integer := 12_345;
      Z7    : Port.Frame := Blank (7);
      Z14   : Port.Frame := Blank (14);
      Bad   : Natural := 0;
   begin
      --  a) the published vector, and the reference against it
      Check ("crc: DF 17 vector",
             Natural (Mp.CRC24 (V17)) = 16#576098#);
      Check ("crc: reference agrees with the DF 17 vector",
             Ref_CRC (V17.Data, 11) = 16#576098#);

      --  b) values worked out by hand
      Check ("crc: zero message, short", Natural (Mp.CRC24 (Z7)) = 0);
      Check ("crc: zero message, long",  Natural (Mp.CRC24 (Z14)) = 0);

      Z7.Data (3) := 1;
      Check ("crc: last message bit set (short) is the polynomial",
             Natural (Mp.CRC24 (Z7)) = 16#FFF409#);
      Check ("crc: reference agrees (short)",
             Ref_CRC (Z7.Data, 4) = 16#FFF409#);

      Z7.Data (3) := 2;
      Check ("crc: next bit gives 001C1B",
             Natural (Mp.CRC24 (Z7)) = 16#001C1B#);
      Check ("crc: reference agrees (next bit)",
             Ref_CRC (Z7.Data, 4) = 16#001C1B#);

      Z14.Data (10) := 1;
      Check ("crc: last message bit set (long) is the polynomial",
             Natural (Mp.CRC24 (Z14)) = 16#FFF409#);

      --  the AP field is not part of the message: changing it does not
      --  change the CRC
      Z7 := Blank (7);
      Z7.Data (4) := 16#FF#;  Z7.Data (5) := 16#FF#;  Z7.Data (6) := 16#FF#;
      Check ("crc: the AP field is not part of the message",
             Natural (Mp.CRC24 (Z7)) = 0);

      --  every single-bit error in the message is detected
      for B in 0 .. 87 loop
         declare
            W    : Port.Frame := V17;
            Old  : constant Natural := Natural (W.Data (B / 8));
            Mask : constant Natural := 2 ** (7 - B mod 8);
            New_B : constant Natural :=
              (if (Old / Mask) mod 2 = 1 then Old - Mask else Old + Mask);
         begin
            W.Data (B / 8) := Port.U8 (New_B);
            if Natural (Mp.CRC24 (W)) = 16#576098# then
               Bad := Bad + 1;
            end if;
         end;
      end loop;
      Check ("crc: all 88 single-bit errors detected (missed"
             & Natural'Image (Bad) & ")", Bad = 0);

      --  random frames: the parser's CRC against the reference
      Bad := 0;
      for K in 1 .. 1_000 loop
         declare
            S : Port.Frame := Blank (7);
            L : Port.Frame := Blank (14);
         begin
            Fill (S, 7, State);
            Fill (L, 14, State);
            if Natural (Mp.CRC24 (S)) /= Ref_CRC (S.Data, 4) then
               Bad := Bad + 1;
            end if;
            if Natural (Mp.CRC24 (L)) /= Ref_CRC (L.Data, 11) then
               Bad := Bad + 1;
            end if;
         end;
      end loop;
      Check ("crc: 2000 random frames agree with the reference (mismatches"
             & Natural'Image (Bad) & ")", Bad = 0);

      --  linearity: CRC (a xor b) = CRC (a) xor CRC (b), same length,
      --  because the register starts at 0 and nothing is XORed at the end
      Bad := 0;
      for K in 1 .. 500 loop
         declare
            A  : Port.Frame := Blank (14);
            B  : Port.Frame := Blank (14);
            AB : Port.Frame := Blank (14);
         begin
            Fill (A, 14, State);
            Fill (B, 14, State);
            for I in 0 .. 13 loop
               AB.Data (I) :=
                 Port.U8 (Port.U32 (A.Data (I)) xor Port.U32 (B.Data (I)));
            end loop;
            if Natural (Mp.CRC24 (AB))
              /= Natural (Port.U32 (Mp.CRC24 (A)) xor Port.U32 (Mp.CRC24 (B)))
            then
               Bad := Bad + 1;
            end if;
         end;
      end loop;
      Check ("crc: linear on 500 random pairs (mismatches"
             & Natural'Image (Bad) & ")", Bad = 0);
   end Test_CRC;

   --  Group: address recovery

   procedure Test_Address is
      W : Port.Frame := V17;
   begin
      --  the DF 17 example: CRC and AP are equal, the address is 0
      Check ("address: DF 17 vector gives 0",
             Natural (Mp.Recovered_Address (V17)) = 0);

      --  AP = 576098 xor ABCDEF = FCAD77, worked out by hand
      W.Data (11) := 16#FC#;  W.Data (12) := 16#AD#;  W.Data (13) := 16#77#;
      Check ("address: AP FCAD77 recovers ABCDEF",
             Natural (Mp.Recovered_Address (W)) = 16#ABCDEF#);

      --  frames built with the reference CRC
      Check ("address: built short frame",
             Natural (Mp.Recovered_Address
                        (Build (0, False, Good_AC, 16#123456#, 0, 0)))
             = 16#123456#);
      Check ("address: built long frame",
             Natural (Mp.Recovered_Address
                        (Build (16, True, Good_AC, 16#ABCDEF#, 0, 0)))
             = 16#ABCDEF#);
      Check ("address: all ones",
             Natural (Mp.Recovered_Address
                        (Build (0, False, Good_AC, 16#FFFFFF#, 0, 0)))
             = 16#FFFFFF#);
      Check ("address: zero",
             Natural (Mp.Recovered_Address
                        (Build (0, False, Good_AC, 0, 0, 0)))
             = 0);
      Check ("address: one",
             Natural (Mp.Recovered_Address
                        (Build (16, True, 0, 1, 0, 0)))
             = 1);
   end Test_Address;

   --  Group: altitude code

   procedure Expect_Alt
     (Code   : Natural;
      Status : Mp.Parse_Status;
      Alt    : Integer)
   is
      R : constant Mp.Altitude_Result :=
        Mp.Decode_Altitude (Port.U16 (Code));
   begin
      Check ("altitude: code" & Natural'Image (Code) & " status (expected "
             & Mp.Parse_Status'Image (Status) & ", got "
             & Mp.Parse_Status'Image (R.Status) & ")",
             R.Status = Status);
      Check ("altitude: code" & Natural'Image (Code) & " value",
             Integer (R.Value) = Alt);
   end Expect_Alt;

   procedure Test_Altitude is
   begin
      --  no altitude
      Expect_Alt (16#0000#, Mp.No_Altitude, 0);
      Expect_Alt (16#0010#, Mp.No_Altitude, 0);          --  M=0, Q=1, N=0

      --  hand-worked codes (header)
      Expect_Alt (16#0011#, Mp.Parsed, -975);            --  N = 1
      Expect_Alt (16#0012#, Mp.Parsed, -950);            --  N = 2
      Expect_Alt (16#001F#, Mp.Parsed, -625);            --  N = 15
      Expect_Alt (16#0030#, Mp.Parsed, -600);            --  N = 16 (B1)
      Expect_Alt (16#0031#, Mp.Parsed, -575);            --  N = 17
      Expect_Alt (16#0090#, Mp.Parsed, -200);            --  N = 32 (A4)
      Expect_Alt (16#06B8#, Mp.Parsed, 10_000);          --  N = 440
      Expect_Alt (16#1838#, Mp.Parsed, 38_000);          --  N = 1 560
      Expect_Alt (16#1FBF#, Mp.Parsed, 50_175);          --  N = 2 047

      --  metric (M = 1) and Gillham (Q = 0) coding
      Expect_Alt (16#0040#, Mp.Unsupported_Altitude, 0);  --  M only
      Expect_Alt (16#0050#, Mp.Unsupported_Altitude, 0);  --  M and Q
      Expect_Alt (16#0001#, Mp.Unsupported_Altitude, 0);  --  Q = 0
      Expect_Alt (16#0020#, Mp.Unsupported_Altitude, 0);  --  B1, Q = 0
      Expect_Alt (16#0080#, Mp.Unsupported_Altitude, 0);  --  A4, Q = 0
      Expect_Alt (16#1FFF#, Mp.Unsupported_Altitude, 0);  --  everything set
   end Test_Altitude;

   --  Every one of the 8 192 codes. Counts worked out by hand:
   --    codes with M = 0 and Q = 1: 2 ** 11 = 2 048
   --      N = 0 (code 16): No_Altitude          1
   --      N = 1 .. 2 047:  Parsed           2 047
   --    code 0:                  No_Altitude    1
   --    every other code:        Unsupported 8 192 - 2 048 - 1 = 6 143
   --  The 2 047 parsed altitudes are 2 047 DIFFERENT values on the 25 ft
   --  grid, from -975 to 50 175, and their sum is
   --    25 * (2 047 * 2 048 / 2) - 1 000 * 2 047 = 50 356 200.
   procedure Sweep_Altitude is
      Parsed    : Natural := 0;
      No_Alt    : Natural := 0;
      Unsup     : Natural := 0;
      Bad_Value : Natural := 0;
      Dup       : Natural := 0;
      Off_Grid  : Natural := 0;
      Sum       : Integer := 0;
      Min_V     : Integer := Integer'Last;
      Max_V     : Integer := Integer'First;
      Seen      : array (0 .. 2_047) of Boolean := (others => False);
   begin
      for A in 0 .. 8_191 loop
         declare
            R : constant Mp.Altitude_Result :=
              Mp.Decode_Altitude (Port.U16 (A));
            V : constant Integer := Integer (R.Value);
         begin
            case R.Status is
               when Mp.Parsed =>
                  Parsed := Parsed + 1;
                  Sum    := Sum + V;
                  Min_V  := Integer'Min (Min_V, V);
                  Max_V  := Integer'Max (Max_V, V);

                  if (V + 1_000) mod 25 /= 0 then
                     Off_Grid := Off_Grid + 1;
                  elsif (V + 1_000) / 25 > 2_047 then
                     Off_Grid := Off_Grid + 1;
                  elsif Seen ((V + 1_000) / 25) then
                     Dup := Dup + 1;
                  else
                     Seen ((V + 1_000) / 25) := True;
                  end if;

               when Mp.No_Altitude =>
                  No_Alt := No_Alt + 1;
                  if V /= 0 then
                     Bad_Value := Bad_Value + 1;
                  end if;

               when Mp.Unsupported_Altitude =>
                  Unsup := Unsup + 1;
                  if V /= 0 then
                     Bad_Value := Bad_Value + 1;
                  end if;

               when others =>
                  Bad_Value := Bad_Value + 1;
            end case;
         end;
      end loop;

      Check ("altitude sweep: 2047 parsed (got" & Natural'Image (Parsed)
             & ")", Parsed = 2_047);
      Check ("altitude sweep: 2 no-altitude codes (got"
             & Natural'Image (No_Alt) & ")", No_Alt = 2);
      Check ("altitude sweep: 6143 unsupported codes (got"
             & Natural'Image (Unsup) & ")", Unsup = 6_143);
      Check ("altitude sweep: a failure always has value 0 (bad"
             & Natural'Image (Bad_Value) & ")", Bad_Value = 0);
      Check ("altitude sweep: every parsed value is on the 25 ft grid (bad"
             & Natural'Image (Off_Grid) & ")", Off_Grid = 0);
      Check ("altitude sweep: no two codes give the same altitude (dups"
             & Natural'Image (Dup) & ")", Dup = 0);
      Check ("altitude sweep: lowest is -975 ft", Min_V = -975);
      Check ("altitude sweep: highest is 50175 ft", Max_V = 50_175);
      Check ("altitude sweep: sum is 50356200 (got" & Integer'Image (Sum)
             & ")", Sum = 50_356_200);
   end Sweep_Altitude;

   --  Group: range from the round trip

   procedure Expect_Range
     (RTT    : Long_Long_Integer;
      Status : Mp.Parse_Status;
      Feet   : Natural)
   is
      R : constant Mp.Range_Result :=
        Mp.Range_From_Round_Trip (Port.U32 (RTT));
   begin
      Check ("range: round trip" & Long_Long_Integer'Image (RTT)
             & " status (expected " & Mp.Parse_Status'Image (Status)
             & ", got " & Mp.Parse_Status'Image (R.Status) & ")",
             R.Status = Status);
      Check ("range: round trip" & Long_Long_Integer'Image (RTT) & " value",
             Natural (R.Value) = Feet);
   end Expect_Range;

   procedure Test_Range is
   begin
      --  shorter than the transponder reply delay (128 000 ns)
      Expect_Range (0,                 Mp.Bad_Timing, 0);
      Expect_Range (1,                 Mp.Bad_Timing, 0);
      Expect_Range (127_999,           Mp.Bad_Timing, 0);

      --  just above it, the truncation of the first feet (header)
      Expect_Range (128_000,           Mp.Parsed, 0);       --  net 0
      Expect_Range (128_001,           Mp.Parsed, 0);       --  0.49 ft
      Expect_Range (128_002,           Mp.Parsed, 0);       --  0.98 ft
      Expect_Range (128_003,           Mp.Parsed, 1);       --  1.47 ft

      --  hand-worked values
      Expect_Range (138_000,           Mp.Parsed, 4_917);   --  net 10 000
      Expect_Range (140_345,           Mp.Parsed, 6_071);   --  net 12 345
      Expect_Range (148_000,           Mp.Parsed, 9_835);   --  net 20 000
      Expect_Range (228_000,           Mp.Parsed, 49_178);  --  net 100 000

      --  the largest range: 150 000 ft is accepted, 150 001 is not
      Expect_Range (433_013,           Mp.Parsed, 150_000);
      Expect_Range (433_014,           Mp.Bad_Timing, 0);
      Expect_Range (1_000_000,         Mp.Bad_Timing, 0);
      Expect_Range (4_294_967_295,     Mp.Bad_Timing, 0);
   end Test_Range;

   --  Every round trip from 0 to 500 000 ns. Structure, not a second copy
   --  of the formula:
   --    * below 128 000 and above 433 013: Bad_Timing with value 0
   --    * 128 000 .. 433 013 (305 014 values): Parsed
   --    * the value starts at 0, ends at 150 000, never goes down and
   --      never jumps by more than 1 ft per nanosecond (the slope is
   --      0.49 ft/ns), so it goes up exactly 150 000 times.
   procedure Sweep_Range is
      Parsed_Count : Natural := 0;
      Increments   : Natural := 0;
      Bad_Status   : Natural := 0;
      Bad_Step     : Natural := 0;
      Bad_Zero     : Natural := 0;
      Prev         : Integer := 0;
      First_V      : Integer := -1;
      Last_V       : Integer := -1;
   begin
      for T in 0 .. 500_000 loop
         declare
            R : constant Mp.Range_Result :=
              Mp.Range_From_Round_Trip (Port.U32 (T));
            V : constant Integer := Integer (R.Value);
            In_Window : constant Boolean :=
              T >= 128_000 and then T <= 433_013;
         begin
            if In_Window then
               if R.Status /= Mp.Parsed then
                  Bad_Status := Bad_Status + 1;
               else
                  Parsed_Count := Parsed_Count + 1;

                  if T = 128_000 then
                     First_V := V;
                  elsif V - Prev = 1 then
                     Increments := Increments + 1;
                  elsif V /= Prev then
                     Bad_Step := Bad_Step + 1;
                  end if;

                  Prev   := V;
                  Last_V := V;
               end if;
            else
               if R.Status /= Mp.Bad_Timing then
                  Bad_Status := Bad_Status + 1;
               elsif V /= 0 then
                  Bad_Zero := Bad_Zero + 1;
               end if;
            end if;
         end;
      end loop;

      Check ("range sweep: 305014 parsed (got"
             & Natural'Image (Parsed_Count) & ")",
             Parsed_Count = 305_014);
      Check ("range sweep: status outside and inside the window (bad"
             & Natural'Image (Bad_Status) & ")", Bad_Status = 0);
      Check ("range sweep: failures have value 0 (bad"
             & Natural'Image (Bad_Zero) & ")", Bad_Zero = 0);
      Check ("range sweep: never down, never more than 1 ft per ns (bad"
             & Natural'Image (Bad_Step) & ")", Bad_Step = 0);
      Check ("range sweep: starts at 0 ft", First_V = 0);
      Check ("range sweep: ends at 150000 ft", Last_V = 150_000);
      Check ("range sweep: goes up exactly 150000 times (got"
             & Natural'Image (Increments) & ")", Increments = 150_000);
   end Sweep_Range;

   --  Group: Parse

   procedure Test_Parse is
      V : constant Port.Frame :=
        Build (0, False, Good_AC, Exp_Addr, Good_RTT, Good_Bearing);
      L : constant Port.Frame :=
        Build (16, True, Good_AC, Exp_Addr, Good_RTT, Good_Bearing);
      W : Port.Frame;
   begin
      --  success, short and long
      Expect_Parsed ("parse short DF 0", V, Good_Slant, Good_Bearing,
                     Good_Alt);
      Expect_Parsed ("parse long DF 16", L, Good_Slant, Good_Bearing,
                     Good_Alt);

      --  the bearing passes through unchanged, at its two ends
      Expect_Parsed ("parse bearing 0",
                     Build (0, False, Good_AC, Exp_Addr, Good_RTT, 0),
                     Good_Slant, 0, Good_Alt);
      Expect_Parsed ("parse bearing 35999",
                     Build (0, False, Good_AC, Exp_Addr, Good_RTT, 35_999),
                     Good_Slant, 35_999, Good_Alt);

      --  the altitude and the range come through the right fields
      Expect_Parsed ("parse 38000 ft",
                     Build (0, False, 16#1838#, Exp_Addr, Good_RTT, 0),
                     Good_Slant, 0, 38_000);
      Expect_Parsed ("parse -975 ft",
                     Build (16, True, 16#0011#, Exp_Addr, Good_RTT, 0),
                     Good_Slant, 0, -975);
      Expect_Parsed ("parse 50175 ft",
                     Build (0, False, 16#1FBF#, Exp_Addr, Good_RTT, 0),
                     Good_Slant, 0, 50_175);
      Expect_Parsed ("parse range 0 ft",
                     Build (0, False, Good_AC, Exp_Addr, 128_000, 0),
                     0, 0, Good_Alt);
      Expect_Parsed ("parse range 150000 ft",
                     Build (0, False, Good_AC, Exp_Addr, 433_013, 0),
                     150_000, 0, Good_Alt);

      --  check 1: the C side broke a promise
      W := V;  W.Reserved := 1;
      Expect_Fail ("parse: reserved set", W, Mp.Malformed_Frame);

      W := V;  W.Status := Port.Status_Err_Length;
      Expect_Fail ("parse: valid with an error status", W,
                   Mp.Malformed_Frame);

      W := V;  W.Bearing := 36_000;
      Expect_Fail ("parse: bearing 36000", W, Mp.Malformed_Frame);

      W := V;  W.Length := 8;
      Expect_Fail ("parse: length 8", W, Mp.Malformed_Frame);

      W := V;  W.Data (9) := 1;
      Expect_Fail ("parse: byte after the length", W, Mp.Malformed_Frame);

      W := Rejected (3);  W.Data (0) := 1;
      Expect_Fail ("parse: rejected frame carrying data", W,
                   Mp.Malformed_Frame);

      --  check 2: the driver rejected the frame
      for S in 1 .. 6 loop
         Expect_Fail ("parse: driver status" & Integer'Image (S),
                      Rejected (S), Mp.Driver_Rejected);
      end loop;

      --  check 1 before check 2: rejected AND malformed
      W := Rejected (3);  W.Reserved := 1;
      Expect_Fail ("parse: malformed wins over driver-rejected", W,
                   Mp.Malformed_Frame);

      --  check 3: format and length
      Expect_Fail ("parse: DF 17",
                   Build (17, True, Good_AC, Exp_Addr, Good_RTT, 0),
                   Mp.Unsupported_Format);
      Expect_Fail ("parse: DF 0 with 14 bytes",
                   Build (0, True, Good_AC, Exp_Addr, Good_RTT, 0),
                   Mp.Unsupported_Format);
      Expect_Fail ("parse: DF 16 with 7 bytes",
                   Build (16, False, Good_AC, Exp_Addr, Good_RTT, 0),
                   Mp.Unsupported_Format);
      Expect_Fail ("parse: DF 11",
                   Build (11, False, Good_AC, Exp_Addr, Good_RTT, 0),
                   Mp.Unsupported_Format);
      Expect_Fail ("parse: DF 31",
                   Build (31, True, Good_AC, Exp_Addr, Good_RTT, 0),
                   Mp.Unsupported_Format);

      --  check 4: the address recovered from the parity
      Expect_Fail ("parse: frame meant for another aircraft",
                   Build (0, False, Good_AC, Wrong_Addr, Good_RTT, 0),
                   Mp.Wrong_Address);

      W := V;  W.Data (3) := 16#B9#;     --  one bit of the message changed
      Expect_Fail ("parse: corrupted message (CRC no longer matches)", W,
                   Mp.Wrong_Address);

      W := V;                            --  one bit of the AP field changed
      W.Data (6) := Port.U8 ((Natural (V.Data (6)) + 1) mod 256);
      Expect_Fail ("parse: corrupted AP field", W, Mp.Wrong_Address);

      W := L;  W.Data (6) := 16#00#;     --  a byte inside a long message
      Expect_Fail ("parse: corrupted long message", W, Mp.Wrong_Address);

      --  check 5: altitude
      Expect_Fail ("parse: altitude code 0",
                   Build (0, False, 16#0000#, Exp_Addr, Good_RTT, 0),
                   Mp.No_Altitude);
      Expect_Fail ("parse: altitude N = 0",
                   Build (0, False, 16#0010#, Exp_Addr, Good_RTT, 0),
                   Mp.No_Altitude);
      Expect_Fail ("parse: metric altitude",
                   Build (0, False, 16#1FFF#, Exp_Addr, Good_RTT, 0),
                   Mp.Unsupported_Altitude);
      Expect_Fail ("parse: Gillham altitude (Q = 0)",
                   Build (16, True, 16#0001#, Exp_Addr, Good_RTT, 0),
                   Mp.Unsupported_Altitude);

      --  check 6: timing
      Expect_Fail ("parse: round trip below the reply delay",
                   Build (0, False, Good_AC, Exp_Addr, 127_999, 0),
                   Mp.Bad_Timing);
      Expect_Fail ("parse: round trip zero",
                   Build (0, False, Good_AC, Exp_Addr, 0, 0),
                   Mp.Bad_Timing);
      Expect_Fail ("parse: range 150001 ft",
                   Build (0, False, Good_AC, Exp_Addr, 433_014, 0),
                   Mp.Bad_Timing);
      Expect_Fail ("parse: largest round trip",
                   Build (0, False, Good_AC, Exp_Addr, 4_294_967_295, 0),
                   Mp.Bad_Timing);

      --  order of the checks: the first failure wins
      Expect_Fail ("order: format before address",
                   Build (17, True, Good_AC, Wrong_Addr, Good_RTT, 0),
                   Mp.Unsupported_Format);
      Expect_Fail ("order: address before altitude",
                   Build (0, False, 16#0000#, Wrong_Addr, Good_RTT, 0),
                   Mp.Wrong_Address);
      Expect_Fail ("order: altitude before timing",
                   Build (0, False, 16#0000#, Exp_Addr, 0, 0),
                   Mp.No_Altitude);
      Expect_Fail ("order: unsupported altitude before timing",
                   Build (16, True, 16#1FFF#, Exp_Addr, 433_014, 0),
                   Mp.Unsupported_Altitude);

      W := Build (17, True, 16#0000#, Wrong_Addr, 0, 0);
      W.Reserved := 1;
      Expect_Fail ("order: malformed before everything", W,
                   Mp.Malformed_Frame);
   end Test_Parse;

   --  Group: sweep of frames against the order of checks

   type Fmt_Case is record
      DF   : Natural;
      Long : Boolean;
      Ok   : Boolean;
   end record;

   type AC_Case is record
      Code   : Natural;
      Status : Mp.Parse_Status;
      Alt    : Integer;
   end record;

   type RTT_Case is record
      RTT   : Long_Long_Integer;
      Bad   : Boolean;
      Slant : Natural;
   end record;

   --  1 680 frames: 4 formats x 2 addresses x 10 altitude codes x 7 round
   --  trips x 3 bearings. The expected status follows the order of checks
   --  of the specification: format, address, altitude, timing.
   procedure Sweep_Parse is
      Fmts : constant array (1 .. 4) of Fmt_Case :=
        ((0, False, True),     --  DF 0, 7 bytes
         (16, True, True),     --  DF 16, 14 bytes
         (17, True, False),    --  DF 17
         (11, False, False));  --  DF 11

      ACs : constant array (1 .. 10) of AC_Case :=
        ((16#0000#, Mp.No_Altitude, 0),
         (16#0010#, Mp.No_Altitude, 0),
         (16#0001#, Mp.Unsupported_Altitude, 0),
         (16#0040#, Mp.Unsupported_Altitude, 0),
         (16#1FFF#, Mp.Unsupported_Altitude, 0),
         (16#0011#, Mp.Parsed, -975),
         (16#0030#, Mp.Parsed, -600),
         (16#1838#, Mp.Parsed, 38_000),
         (16#06B8#, Mp.Parsed, 10_000),
         (16#1FBF#, Mp.Parsed, 50_175));

      RTTs : constant array (1 .. 7) of RTT_Case :=
        ((127_999, True, 0),
         (128_000, False, 0),
         (128_003, False, 1),
         (148_000, False, 9_835),
         (228_000, False, 49_178),
         (433_013, False, 150_000),
         (433_014, True, 0));

      Bearings : constant array (1 .. 3) of Natural :=
        (0, 18_000, 35_999);

      Combos : Natural := 0;
      Bad    : Natural := 0;
   begin
      for Fi in Fmts'Range loop
         for Wi in 1 .. 2 loop
            for Ai in ACs'Range loop
               for Ri in RTTs'Range loop
                  for Bi in Bearings'Range loop
                     declare
                        Fm : Fmt_Case renames Fmts (Fi);
                        AC : AC_Case  renames ACs (Ai);
                        Rt : RTT_Case renames RTTs (Ri);

                        F : constant Port.Frame :=
                          Build (Fm.DF, Fm.Long, AC.Code,
                                 (if Wi = 1 then Exp_Addr else Wrong_Addr),
                                 Rt.RTT, Bearings (Bi));

                        R : constant Mp.Parse_Result :=
                          Mp.Parse (F, Ty.Mode_S_Address (Exp_Addr));

                        Expected : constant Mp.Parse_Status :=
                          (if not Fm.Ok then Mp.Unsupported_Format
                           elsif Wi = 2 then Mp.Wrong_Address
                           elsif AC.Status /= Mp.Parsed then AC.Status
                           elsif Rt.Bad then Mp.Bad_Timing
                           else Mp.Parsed);

                        Value_Ok : constant Boolean :=
                          (if Expected = Mp.Parsed
                           then Natural (R.Value.Address) = Exp_Addr
                                and then Natural (R.Value.Slant) = Rt.Slant
                                and then Natural (R.Value.Bearing)
                                           = Bearings (Bi)
                                and then Integer (R.Value.Altitude) = AC.Alt
                           else Natural (R.Value.Address) = 0
                                and then Natural (R.Value.Slant) = 0
                                and then Natural (R.Value.Bearing) = 0
                                and then Integer (R.Value.Altitude) = 0);
                     begin
                        Combos := Combos + 1;

                        if R.Status /= Expected or else not Value_Ok then
                           if Bad = 0 then
                              Ada.Text_IO.Put_Line
                                ("sweep first mismatch: format"
                                 & Integer'Image (Fi)
                                 & " address" & Integer'Image (Wi)
                                 & " altitude" & Integer'Image (Ai)
                                 & " round trip" & Integer'Image (Ri)
                                 & " bearing" & Integer'Image (Bi)
                                 & ", expected "
                                 & Mp.Parse_Status'Image (Expected)
                                 & ", got "
                                 & Mp.Parse_Status'Image (R.Status));
                           end if;
                           Bad := Bad + 1;
                        end if;
                     end;
                  end loop;
               end loop;
            end loop;
         end loop;
      end loop;

      Check ("sweep: all" & Natural'Image (Combos)
             & " frames (mismatches" & Natural'Image (Bad) & ")",
             Combos = 1_680 and then Bad = 0);
   end Sweep_Parse;

begin
   Test_Well_Formed;
   Test_Fields;
   Test_CRC;
   Test_Address;
   Test_Altitude;
   Sweep_Altitude;
   Test_Range;
   Sweep_Range;
   Test_Parse;
   Sweep_Parse;

   Ada.Text_IO.Put_Line
     ("test_modes_parser:" & Natural'Image (Total) & " checks,"
      & Natural'Image (Failed) & " failed");

   if Failed > 0 then
      Ada.Text_IO.Put_Line ("FAIL");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   else
      Ada.Text_IO.Put_Line ("OK");
   end if;
end Test_Modes_Parser;