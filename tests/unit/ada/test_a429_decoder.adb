--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : tests/unit/ada/test_a429_decoder.adb
--  Module      : Unit tests of the ARINC 429 decoder (plain Ada program)
--  Language    : Ada 2012 (the tests are not in the SPARK subset)
--  Assurance   : DO-178C Software Level B (target), verification artifact
--  Tests       : src/ada/decoder/tcas-a429_decoder.ads and .adb
--                src/bindings/tcas-a429_port.ads
--  Requirements: REQ-001, REQ-002 (placeholder IDs)
--
--  PURPOSE
--    Checks every behavior promised in tcas-a429_decoder.ads: the BNR
--    bit math, the trust check, the five decoder checks in their order,
--    the exact range limits, and the rule that a failure never publishes
--    a value. GNATprove proves the absence of runtime errors and the
--    postconditions; these tests check that the numbers are the right ones.
--
--  DESIGN RULES OF THE TESTS
--    T1  The expected values are computed by the test with its OWN
--        arithmetic (Signed_Of, a divide and a range test). The decoder's
--        functions are never used to compute an expectation.
--    T2  The boundary tests use literals taken from the specification
--        (for example 126 700 ft), not constants read from the code under
--        test, so a wrong constant in the code is caught.
--    T3  Every failing decode is checked for Value = 0 as well as for its
--        status (rule R5 of the specification).
--    T4  Every pair of faults that can occur together is tested, to prove
--        which check wins (order of checks in the specification).
--    T5  Output: one line per failed check, then a summary. The exit
--        status is 1 if anything failed, else 0, so a script can use it.
--
--  TESTS BY GROUP
--    bit math       : Sign_Extend and Scale at the edges
--    trust check    : Is_Well_Formed, one rule at a time
--    pressure alt   : normal value, both range limits, wrong label
--    radio alt      : normal value, both range limits
--    altitude rate  : normal value, both range limits, truncation
--    order          : first-failure-wins for each pair of faults
--    sweep          : all 2 ** 19 data fields through each decoder
--
--  USAGE (from the project root; no project file is needed)
--    mkdir -p build/obj_test build/bin
--    gnatmake tests/unit/ada/test_a429_decoder.adb -gnat2012 -gnatwa \
--        -D build/obj_test -o build/bin/test_a429_decoder.exe \
--        -Isrc/ada/common -Isrc/ada/common/types \
--        -Isrc/ada/decoder -Isrc/bindings
--    build/bin/test_a429_decoder.exe
--
--  NOTES
--    * tcas2.gpr does not list tests/, so the project build never sees
--      this program (it has a main procedure and is not part of the
--      product).
--  ===========================================================================

with Ada.Command_Line;
with Ada.Text_IO;

with Tcas.A429_Decoder;
with Tcas.A429_Port;
with Tcas.Types;

procedure Test_A429_Decoder is

   package Port renames Tcas.A429_Port;
   package Dec  renames Tcas.A429_Decoder;

   use type Port.U8;
   use type Port.U32;
   use type Dec.Decode_Status;
   use type Tcas.Types.Altitude_Ft;
   use type Tcas.Types.Vertical_Rate_Fpm;


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

   --  Helpers (independent of the code under test, rule T1)

   --  Signed value of a 19-bit two's complement field.
   function Signed_Of (Raw : Integer) return Integer is
     (if Raw >= 262_144 then Raw - 524_288 else Raw);

   --  A word that the C driver could really produce: well-formed, valid,
   --  normal SSM, status OK.
   function Valid_Word
     (Label : Port.U8;
      Data  : Port.U32;
      Ssm   : Port.U8 := Port.Ssm_Normal) return Port.Word is
     (Data       => Data,
      Frame      => 1,
      Label      => Label,
      Sdi        => 0,
      Ssm        => Ssm,
      Status     => Port.Status_OK,
      Valid      => 1,
      Reserved8  => 0,
      Reserved16 => 0);

   --  A word the driver rejected: invalid, data 0, an error status.
   function Rejected_Word
     (Label : Port.U8; Status : Port.U8) return Port.Word is
     (Data       => 0,
      Frame      => 1,
      Label      => Label,
      Sdi        => 0,
      Ssm        => 0,
      Status     => Status,
      Valid      => 0,
      Reserved8  => 0,
      Reserved16 => 0);

   function Is_Alt (R : Dec.Altitude_Result;
                    S : Dec.Decode_Status) return Boolean is
     (R.Status = S and then R.Value = 0);

   function Is_Rate (R : Dec.Vertical_Rate_Result;
                     S : Dec.Decode_Status) return Boolean is
     (R.Status = S and then R.Value = 0);

   --  Group: bit math

   procedure Test_Bit_Math is
   begin
      Check ("sign_extend 0",          Dec.Sign_Extend (0) = 0);
      Check ("sign_extend 1",          Dec.Sign_Extend (1) = 1);
      Check ("sign_extend max positive",
             Dec.Sign_Extend (16#3_FFFF#) = 262_143);
      Check ("sign_extend most negative",
             Dec.Sign_Extend (16#4_0000#) = -262_144);
      Check ("sign_extend all ones is -1",
             Dec.Sign_Extend (16#7_FFFF#) = -1);

      Check ("scale positive",         Dec.Scale (3, 2) = 1);
      Check ("scale negative exact",
             Dec.Scale (16#4_0000#, 2) = -131_072);
      Check ("scale -1 / 2 truncates toward zero",
             Dec.Scale (16#7_FFFF#, 2) = 0);
      Check ("scale -3 / 2 truncates toward zero",
             Dec.Scale (16#7_FFFD#, 2) = -1);
      Check ("scale -1 / 32 is 0",
             Dec.Scale (16#7_FFFF#, 32) = 0);
   end Test_Bit_Math;

   --  Group: trust check

   procedure Test_Trust_Check is
      W : Port.Word := Valid_Word (Port.Label_Pressure_Alt, 100);
   begin
      Check ("well-formed: valid word", Port.Is_Well_Formed (W));

      W := Valid_Word (Port.Label_Pressure_Alt, Port.Data_Max);
      Check ("well-formed: data at the 19-bit limit",
             Port.Is_Well_Formed (W));

      W := Valid_Word (Port.Label_Pressure_Alt, Port.Data_Max + 1);
      Check ("not well-formed: valid word with data above 19 bits",
             not Port.Is_Well_Formed (W));

      W := Valid_Word (Port.Label_Pressure_Alt, 100);
      W.Reserved8 := 1;
      Check ("not well-formed: reserved8 set", not Port.Is_Well_Formed (W));

      W := Valid_Word (Port.Label_Pressure_Alt, 100);
      W.Reserved16 := 1;
      Check ("not well-formed: reserved16 set",
             not Port.Is_Well_Formed (W));

      W := Valid_Word (Port.Label_Pressure_Alt, 100);
      W.Sdi := 4;
      Check ("not well-formed: sdi above 3", not Port.Is_Well_Formed (W));

      W := Valid_Word (Port.Label_Pressure_Alt, 100);
      W.Ssm := 4;
      Check ("not well-formed: ssm above 3", not Port.Is_Well_Formed (W));

      W := Valid_Word (Port.Label_Pressure_Alt, 100);
      W.Status := 10;
      Check ("not well-formed: status above 9", not Port.Is_Well_Formed (W));

      W := Valid_Word (Port.Label_Pressure_Alt, 100);
      W.Valid := 2;
      Check ("not well-formed: valid is 2", not Port.Is_Well_Formed (W));

      W := Valid_Word (Port.Label_Pressure_Alt, 100);
      W.Status := Port.Status_Err_Parity;
      Check ("not well-formed: valid = 1 with an error status",
             not Port.Is_Well_Formed (W));

      W := Rejected_Word (Port.Label_Pressure_Alt, Port.Status_Err_Parity);
      Check ("well-formed: a properly rejected word",
             Port.Is_Well_Formed (W));

      W := Rejected_Word (Port.Label_Pressure_Alt, Port.Status_OK);
      Check ("not well-formed: valid = 0 with status OK",
             not Port.Is_Well_Formed (W));

      W := Rejected_Word (Port.Label_Pressure_Alt, Port.Status_Err_Parity);
      W.Data := 5;
      Check ("not well-formed: invalid word with data not 0",
             not Port.Is_Well_Formed (W));
   end Test_Trust_Check;

   --  Group: pressure altitude (label 203, divisor 2)

   procedure Test_Pressure_Altitude is
      R : Dec.Altitude_Result;
   begin
      R := Dec.Decode_Pressure_Altitude
             (Valid_Word (Port.Label_Pressure_Alt, 20_000));
      Check ("pressure: 20000 counts is 10000 ft",
             R.Status = Dec.Decoded and then R.Value = 10_000);

      R := Dec.Decode_Pressure_Altitude
             (Valid_Word (Port.Label_Pressure_Alt, 0));
      Check ("pressure: zero",
             R.Status = Dec.Decoded and then R.Value = 0);

      --  upper limit: 126 700 ft is 253 400 counts
      R := Dec.Decode_Pressure_Altitude
             (Valid_Word (Port.Label_Pressure_Alt, 253_400));
      Check ("pressure: 126700 ft decodes",
             R.Status = Dec.Decoded and then R.Value = 126_700);

      R := Dec.Decode_Pressure_Altitude
             (Valid_Word (Port.Label_Pressure_Alt, 253_402));
      Check ("pressure: 126701 ft is out of range",
             Is_Alt (R, Dec.Out_Of_Range));

      --  lower limit: -1 200 ft is -2 400 counts = 521 888 raw
      R := Dec.Decode_Pressure_Altitude
             (Valid_Word (Port.Label_Pressure_Alt, 521_888));
      Check ("pressure: -1200 ft decodes",
             R.Status = Dec.Decoded and then R.Value = -1_200);

      R := Dec.Decode_Pressure_Altitude
             (Valid_Word (Port.Label_Pressure_Alt, 521_886));
      Check ("pressure: -1201 ft is out of range",
             Is_Alt (R, Dec.Out_Of_Range));

      R := Dec.Decode_Pressure_Altitude
             (Valid_Word (Port.Label_Pressure_Alt, 16#3_FFFF#));
      Check ("pressure: largest field is out of range, not clamped",
             Is_Alt (R, Dec.Out_Of_Range));

      R := Dec.Decode_Pressure_Altitude
             (Valid_Word (Port.Label_Pressure_Alt, 16#4_0000#));
      Check ("pressure: most negative field is out of range",
             Is_Alt (R, Dec.Out_Of_Range));

      R := Dec.Decode_Pressure_Altitude
             (Valid_Word (Port.Label_Radio_Alt, 20_000));
      Check ("pressure: radio label is the wrong label",
             Is_Alt (R, Dec.Wrong_Label));

      R := Dec.Decode_Pressure_Altitude
             (Rejected_Word (Port.Label_Pressure_Alt,
                             Port.Status_Err_Ssm_Fw));
      Check ("pressure: driver-rejected word",
             Is_Alt (R, Dec.Driver_Rejected));
   end Test_Pressure_Altitude;

   --  Group: radio altitude (label 164, divisor 32)

   procedure Test_Radio_Altitude is
      R : Dec.Altitude_Result;
   begin
      R := Dec.Decode_Radio_Altitude
             (Valid_Word (Port.Label_Radio_Alt, 16_000));
      Check ("radio: 16000 counts is 500 ft",
             R.Status = Dec.Decoded and then R.Value = 500);

      R := Dec.Decode_Radio_Altitude
             (Valid_Word (Port.Label_Radio_Alt, 16#3_FFFF#));
      Check ("radio: largest field is 8191 ft",
             R.Status = Dec.Decoded and then R.Value = 8_191);

      --  -20 ft is -640 counts = 523 648 raw
      R := Dec.Decode_Radio_Altitude
             (Valid_Word (Port.Label_Radio_Alt, 523_648));
      Check ("radio: -20 ft decodes",
             R.Status = Dec.Decoded and then R.Value = -20);

      --  -1 200 ft is -38 400 counts = 485 888 raw; -1 201 ft is 485 856
      R := Dec.Decode_Radio_Altitude
             (Valid_Word (Port.Label_Radio_Alt, 485_888));
      Check ("radio: -1200 ft decodes",
             R.Status = Dec.Decoded and then R.Value = -1_200);

      R := Dec.Decode_Radio_Altitude
             (Valid_Word (Port.Label_Radio_Alt, 485_856));
      Check ("radio: -1201 ft is out of range",
             Is_Alt (R, Dec.Out_Of_Range));

      R := Dec.Decode_Radio_Altitude
             (Valid_Word (Port.Label_Radio_Alt, 16#4_0000#));
      Check ("radio: most negative field is out of range",
             Is_Alt (R, Dec.Out_Of_Range));

      R := Dec.Decode_Radio_Altitude
             (Valid_Word (Port.Label_Pressure_Alt, 16_000));
      Check ("radio: pressure label is the wrong label",
             Is_Alt (R, Dec.Wrong_Label));
   end Test_Radio_Altitude;

   --  Group: altitude rate (label 212, divisor 8)

   procedure Test_Altitude_Rate is
      R : Dec.Vertical_Rate_Result;
   begin
      R := Dec.Decode_Altitude_Rate
             (Valid_Word (Port.Label_Alt_Rate, 12_000));
      Check ("rate: 12000 counts is 1500 ft/min",
             R.Status = Dec.Decoded and then R.Value = 1_500);

      --  upper limit: 10 000 ft/min is 80 000 counts
      R := Dec.Decode_Altitude_Rate
             (Valid_Word (Port.Label_Alt_Rate, 80_000));
      Check ("rate: 10000 ft/min decodes",
             R.Status = Dec.Decoded and then R.Value = 10_000);

      R := Dec.Decode_Altitude_Rate
             (Valid_Word (Port.Label_Alt_Rate, 80_008));
      Check ("rate: 10001 ft/min is out of range",
             Is_Rate (R, Dec.Out_Of_Range));

      --  lower limit: -10 000 ft/min is 524 288 - 80 000 = 444 288 raw
      R := Dec.Decode_Altitude_Rate
             (Valid_Word (Port.Label_Alt_Rate, 444_288));
      Check ("rate: -10000 ft/min decodes",
             R.Status = Dec.Decoded and then R.Value = -10_000);

      R := Dec.Decode_Altitude_Rate
             (Valid_Word (Port.Label_Alt_Rate, 444_280));
      Check ("rate: -10001 ft/min is out of range",
             Is_Rate (R, Dec.Out_Of_Range));

      R := Dec.Decode_Altitude_Rate
             (Valid_Word (Port.Label_Alt_Rate, 7));
      Check ("rate: +7 counts truncates to 0",
             R.Status = Dec.Decoded and then R.Value = 0);

      R := Dec.Decode_Altitude_Rate
             (Valid_Word (Port.Label_Alt_Rate, 524_281));
      Check ("rate: -7 counts truncates to 0, not -1",
             R.Status = Dec.Decoded and then R.Value = 0);

      R := Dec.Decode_Altitude_Rate
             (Valid_Word (Port.Label_Alt_Rate, 16#3_FFFF#));
      Check ("rate: largest field is out of range, not clamped",
             Is_Rate (R, Dec.Out_Of_Range));

      R := Dec.Decode_Altitude_Rate
             (Valid_Word (Port.Label_Pressure_Alt, 12_000));
      Check ("rate: pressure label is the wrong label",
             Is_Rate (R, Dec.Wrong_Label));
   end Test_Altitude_Rate;


   --  Group: order of checks (rule T4)

   procedure Test_Order is
      W : Port.Word;
      R : Dec.Altitude_Result;
   begin
      --  check 1 before check 2: malformed AND invalid
      W := Rejected_Word (Port.Label_Pressure_Alt, Port.Status_Err_Parity);
      W.Reserved8 := 1;
      R := Dec.Decode_Pressure_Altitude (W);
      Check ("order: malformed wins over driver-rejected",
             Is_Alt (R, Dec.Malformed_Word));

      --  invalid word with data not 0 is malformed, not driver-rejected
      W := Rejected_Word (Port.Label_Pressure_Alt, Port.Status_Err_Parity);
      W.Data := 5;
      R := Dec.Decode_Pressure_Altitude (W);
      Check ("order: invalid word carrying data is malformed",
             Is_Alt (R, Dec.Malformed_Word));

      --  check 2 before check 4: invalid AND wrong label
      W := Rejected_Word (Port.Label_Radio_Alt, Port.Status_Err_Label);
      R := Dec.Decode_Pressure_Altitude (W);
      Check ("order: driver-rejected wins over wrong label",
             Is_Alt (R, Dec.Driver_Rejected));

      --  check 3 on its own: valid word, abnormal SSM
      W := Valid_Word (Port.Label_Pressure_Alt, 20_000, Port.Ssm_Failure_Warning);
      R := Dec.Decode_Pressure_Altitude (W);
      Check ("order: valid word with failure-warning SSM is malformed",
             Is_Alt (R, Dec.Malformed_Word));

      W := Valid_Word (Port.Label_Pressure_Alt, 20_000, Port.Ssm_Functional_Test);
      R := Dec.Decode_Pressure_Altitude (W);
      Check ("order: valid word with functional-test SSM is malformed",
             Is_Alt (R, Dec.Malformed_Word));

      --  check 3 before check 4: abnormal SSM AND wrong label
      W := Valid_Word (Port.Label_Radio_Alt, 20_000, Port.Ssm_Failure_Warning);
      R := Dec.Decode_Pressure_Altitude (W);
      Check ("order: abnormal SSM wins over wrong label",
             Is_Alt (R, Dec.Malformed_Word));

      --  check 4 before check 5: wrong label AND huge value
      W := Valid_Word (Port.Label_Radio_Alt, 16#3_FFFF#);
      R := Dec.Decode_Pressure_Altitude (W);
      Check ("order: wrong label wins over out-of-range",
             Is_Alt (R, Dec.Wrong_Label));
   end Test_Order;

   --  Group: sweep of every 19-bit data field (rule T1)

   --  Runs all 2 ** 19 data fields through one altitude decoder and
   --  compares with the expectation built from Signed_Of, a plain divide
   --  and a plain range test. Reports the number of mismatches and the
   --  first one.
   procedure Sweep_Altitude
     (Name    : String;
      Label   : Port.U8;
      Divisor : Integer;
      Pressure : Boolean)
   is
      Bad       : Natural := 0;
      First_Bad : Integer := -1;
   begin
      for Raw in 0 .. 524_287 loop
         declare
            Scaled : constant Integer := Signed_Of (Raw) / Divisor;
            In_Rng : constant Boolean :=
              Scaled in Integer (Tcas.Types.Altitude_Ft'First)
                     .. Integer (Tcas.Types.Altitude_Ft'Last);
            W : constant Port.Word := Valid_Word (Label, Port.U32 (Raw));
            R : constant Dec.Altitude_Result :=
              (if Pressure then Dec.Decode_Pressure_Altitude (W)
               else Dec.Decode_Radio_Altitude (W));
            Good : constant Boolean :=
              (if In_Rng
               then R.Status = Dec.Decoded
                    and then Integer (R.Value) = Scaled
               else Is_Alt (R, Dec.Out_Of_Range));
         begin
            if not Good then
               Bad := Bad + 1;
               if First_Bad < 0 then
                  First_Bad := Raw;
               end if;
            end if;
         end;
      end loop;

      Check ("sweep " & Name & ": every data field (mismatches"
             & Natural'Image (Bad) & ", first raw"
             & Integer'Image (First_Bad) & ")",
             Bad = 0);
   end Sweep_Altitude;

   procedure Sweep_Rate is
      Bad       : Natural := 0;
      First_Bad : Integer := -1;
   begin
      for Raw in 0 .. 524_287 loop
         declare
            Scaled : constant Integer := Signed_Of (Raw) / 8;
            In_Rng : constant Boolean :=
              Scaled in Integer (Tcas.Types.Vertical_Rate_Fpm'First)
                     .. Integer (Tcas.Types.Vertical_Rate_Fpm'Last);
            R : constant Dec.Vertical_Rate_Result :=
              Dec.Decode_Altitude_Rate
                (Valid_Word (Port.Label_Alt_Rate, Port.U32 (Raw)));
            Good : constant Boolean :=
              (if In_Rng
               then R.Status = Dec.Decoded
                    and then Integer (R.Value) = Scaled
               else Is_Rate (R, Dec.Out_Of_Range));
         begin
            if not Good then
               Bad := Bad + 1;
               if First_Bad < 0 then
                  First_Bad := Raw;
               end if;
            end if;
         end;
      end loop;

      Check ("sweep rate: every data field (mismatches"
             & Natural'Image (Bad) & ", first raw"
             & Integer'Image (First_Bad) & ")",
             Bad = 0);
   end Sweep_Rate;

begin
   Test_Bit_Math;
   Test_Trust_Check;
   Test_Pressure_Altitude;
   Test_Radio_Altitude;
   Test_Altitude_Rate;
   Test_Order;

   Sweep_Altitude ("pressure", Port.Label_Pressure_Alt, 2, True);
   Sweep_Altitude ("radio",    Port.Label_Radio_Alt,   32, False);
   Sweep_Rate;

   Ada.Text_IO.Put_Line
     ("test_a429_decoder:" & Natural'Image (Total) & " checks,"
      & Natural'Image (Failed) & " failed");

   if Failed > 0 then
      Ada.Text_IO.Put_Line ("FAIL");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   else
      Ada.Text_IO.Put_Line ("OK");
   end if;
end Test_A429_Decoder;