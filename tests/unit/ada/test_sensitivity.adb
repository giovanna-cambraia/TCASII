--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : tests/unit/ada/test_sensitivity.adb
--  Module      : Unit tests of the sensitivity level selection (plain Ada)
--  Language    : Ada 2012 (the tests are not in the SPARK subset)
--  Assurance   : DO-178C Software Level B (target), verification artifact
--  Tests       : src/ada/threat/tcas-sensitivity.ads and .adb
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Checks every behavior promised in tcas-sensitivity.ads: the eight
--    decisions in their order, the fail-safe level for an untrusted
--    pressure altitude, the low-band rule for a missing radio altitude,
--    and the rule that SL 1 is never returned. GNATprove proves the
--    absence of runtime errors and the postconditions; these tests check
--    that the boundaries are the right numbers.
--
--  DESIGN RULES OF THE TESTS
--    T1  Expected levels are worked out by hand and written next to each
--        check. There is no second copy of the logic in this file.
--    T2  Boundaries are written as literals taken from the current
--        Tcas.Config values (all [VERIFY]): 1 000, 2 350, 5 000, 10 000
--        and 20 000 ft. When the table is corrected in v2, these tests
--        must change with it, and that is intended.
--    T3  The Value of an invalid source must never matter. Tests give
--        invalid sources misleading values (a Value that would change the
--        answer if it were used).
--    T4  The sweeps cover EVERY altitude of the type (127 901 values)
--        against a fixed other source, and compare the histogram (how
--        many altitudes give each level) with a count worked out by hand.
--        A moved boundary changes a count, so every boundary is covered
--        at once.
--    T5  Output: one line per failed check, then a summary. The exit
--        status is 1 if anything failed, else 0, so a script can use it.
--
--  BOUNDARIES IN USE (Tcas.Config, [VERIFY])
--    radio <= 1 000 : SL 2    radio <= 2 350 : SL 3
--    pressure <= 5 000 : SL 4    <= 10 000 : SL 5    <= 20 000 : SL 6
--    above 20 000 : SL 7
--    Altitude range of the type: -1 200 .. 126 700 ft, 127 901 values.
--
--  TESTS BY GROUP
--    decision 1 : pressure altitude not valid
--    decision 2 : radio altitude at or below the SL 2 limit
--    decision 3 : radio altitude in the SL 3 band
--    decision 4 : radio altitude missing, low band
--    decisions 5 to 8 : pressure altitude bands, with and without radio
--    sweeps     : every altitude against a fixed other source
--
--  USAGE (from the project root; no project file is needed)
--    mkdir -p build/obj_test build/bin
--    gnatmake tests/unit/ada/test_sensitivity.adb -gnat2012 -gnatwa \
--        -D build/obj_test -o build/bin/test_sensitivity.exe \
--        -Isrc/ada/common -Isrc/ada/common/types \
--        -Isrc/ada/common/config -Isrc/ada/threat
--    build/bin/test_sensitivity.exe
--
--  NOTES
--    * tcas2.gpr does not list tests/, so the project build never sees
--      this program (it has a main procedure and is not part of the
--      product).
--    * If a check fails, recheck the hand arithmetic written next to it
--      before blaming the selection.

with Ada.Command_Line;
with Ada.Text_IO;

with Tcas.Sensitivity;
with Tcas.Types;

procedure Test_Sensitivity is

   package Se renames Tcas.Sensitivity;
   package Ty renames Tcas.Types;
--
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

   --  Helpers

   function Src (Valid : Boolean; Alt : Integer) return Se.Altitude_Source is
     (Valid => Valid, Value => Ty.Altitude_Ft (Alt));

   --  Level for plain numbers: PV and RV say whether the pressure and the
   --  radio altitude are valid.
   function Level
     (PV : Boolean; P : Integer;
      RV : Boolean; R : Integer) return Natural is
     (Natural (Se.Select_Level (Src (PV, P), Src (RV, R))));

   procedure Expect
     (Name     : String;
      Actual   : Natural;
      Expected : Natural) is
   begin
      Check (Name & " (expected SL" & Natural'Image (Expected)
             & ", got SL" & Natural'Image (Actual) & ")",
             Actual = Expected);
   end Expect;

   --  Group: decision 1, pressure altitude not valid

   procedure Test_Decision_1 is
   begin
      Expect ("d1: pressure invalid, radio invalid",
              Level (False, 15_000, False, 500), 2);
      Expect ("d1: pressure invalid beats a valid radio at 500 ft",
              Level (False, 15_000, True, 500), 2);
      Expect ("d1: pressure invalid beats a valid radio at 1500 ft (SL3)",
              Level (False, 15_000, True, 1_500), 2);
      Expect ("d1: pressure invalid beats a valid radio at 3000 ft",
              Level (False, 15_000, True, 3_000), 2);
      Expect ("d1: pressure invalid at 126700 ft",
              Level (False, 126_700, False, 0), 2);
      Expect ("d1: pressure invalid at -1200 ft",
              Level (False, -1_200, True, 3_000), 2);
   end Test_Decision_1;

   --  Group: decision 2, radio altitude <= 1 000 ft

   procedure Test_Decision_2 is
   begin
      Expect ("d2: radio 1000 ft",   Level (True, 20_000, True, 1_000), 2);
      Expect ("d2: radio 999 ft",    Level (True, 20_000, True, 999), 2);
      Expect ("d2: radio 0 ft",      Level (True, 20_000, True, 0), 2);
      Expect ("d2: radio -20 ft",    Level (True, 20_000, True, -20), 2);
      Expect ("d2: radio -1200 ft (type minimum)",
              Level (True, 20_000, True, -1_200), 2);

      --  the radio altitude wins over a very high pressure altitude
      Expect ("d2: radio 1000 ft, pressure 126700 ft",
              Level (True, 126_700, True, 1_000), 2);
      Expect ("d2: radio 500 ft, pressure 3000 ft",
              Level (True, 3_000, True, 500), 2);
   end Test_Decision_2;

   --  Group: decision 3, radio altitude in the SL 3 band

   procedure Test_Decision_3 is
   begin
      Expect ("d3: radio 1001 ft",   Level (True, 20_000, True, 1_001), 3);
      Expect ("d3: radio 1500 ft",   Level (True, 20_000, True, 1_500), 3);
      Expect ("d3: radio 2350 ft",   Level (True, 20_000, True, 2_350), 3);

      --  the radio altitude wins over the pressure altitude (terrain)
      Expect ("d3: radio 2350 ft, pressure 126700 ft",
              Level (True, 126_700, True, 2_350), 3);
      Expect ("d3: radio 1500 ft, pressure 1500 ft",
              Level (True, 1_500, True, 1_500), 3);
   end Test_Decision_3;

   --  Group: decision 4, radio altitude missing, low band

   procedure Test_Decision_4 is
   begin
      Expect ("d4: pressure 2350 ft, no radio",
              Level (True, 2_350, False, 0), 2);
      Expect ("d4: pressure 2000 ft, no radio",
              Level (True, 2_000, False, 0), 2);
      Expect ("d4: pressure 0 ft, no radio",
              Level (True, 0, False, 0), 2);
      Expect ("d4: pressure -1200 ft, no radio",
              Level (True, -1_200, False, 0), 2);

      --  the Value of an invalid radio altitude must never matter (T3):
      --  5000 would give SL 4 if it were used
      Expect ("d4: garbage radio value 5000 ignored",
              Level (True, 2_000, False, 5_000), 2);

      --  one foot above the low band the pressure altitude decides
      Expect ("d4: pressure 2351 ft, no radio, gives SL 4",
              Level (True, 2_351, False, 0), 4);

      --  a valid radio altitude above the low band with a low pressure
      --  altitude (a high valley): NOT SL 2, the pressure band decides
      Expect ("d4: valid radio 2351 ft, pressure 2000 ft, gives SL 4",
              Level (True, 2_000, True, 2_351), 4);
      Expect ("d4: valid radio 3000 ft, pressure 0 ft, gives SL 4",
              Level (True, 0, True, 3_000), 4);
   end Test_Decision_4;

   --  Group: decisions 5 to 8, pressure altitude bands

   procedure Test_Bands is
   begin
      --  without a radio altitude (cruise: normal, not a fault)
      Expect ("bands: 5000 ft",    Level (True, 5_000, False, 0), 4);
      Expect ("bands: 5001 ft",    Level (True, 5_001, False, 0), 5);
      Expect ("bands: 10000 ft",   Level (True, 10_000, False, 0), 5);
      Expect ("bands: 10001 ft",   Level (True, 10_001, False, 0), 6);
      Expect ("bands: 20000 ft",   Level (True, 20_000, False, 0), 6);
      Expect ("bands: 20001 ft",   Level (True, 20_001, False, 0), 7);
      Expect ("bands: 126700 ft",  Level (True, 126_700, False, 0), 7);

      --  a missing radio altitude at cruise must NOT force SL 2
      Expect ("bands: FL350 with no radio is SL 7, not SL 2",
              Level (True, 35_000, False, 0), 7);

      --  the Value of an invalid radio altitude never matters (T3):
      --  500 would give SL 2 if it were used
      Expect ("bands: garbage radio value 500 ignored at 15000 ft",
              Level (True, 15_000, False, 500), 6);

      --  with a valid radio altitude above 2 350 ft the pressure
      --  altitude decides, the same way
      Expect ("bands: radio 3000, pressure 5000", Level (True, 5_000, True, 3_000), 4);
      Expect ("bands: radio 3000, pressure 5001", Level (True, 5_001, True, 3_000), 5);
      Expect ("bands: radio 3000, pressure 10001", Level (True, 10_001, True, 3_000), 6);
      Expect ("bands: radio 3000, pressure 20001", Level (True, 20_001, True, 3_000), 7);

      --  the radio altitude one foot above its band (2351) also lets the
      --  pressure altitude decide
      Expect ("bands: radio 2351, pressure 15000",
              Level (True, 15_000, True, 2_351), 6);
   end Test_Bands;

   --  Group: sweeps over every altitude of the type (rule T4)

   --  how many altitudes gave each level, index 1 .. 7
   type Counts is array (1 .. 7) of Natural;

   --  Runs every altitude from -1 200 to 126 700 ft through Select_Level.
   --    Vary_Pressure = True : the pressure altitude varies, the radio
   --                           source is fixed (RV, R_Fixed)
   --    Vary_Pressure = False: the radio altitude varies, the pressure
   --                           source is fixed (PV, P_Fixed)
   --  Compares the histogram of levels with Expected. In the pressure
   --  sweeps, also checks that the level never decreases as the altitude
   --  rises (it can only stay or go up).
   procedure Sweep
     (Name          : String;
      Vary_Pressure : Boolean;
      PV            : Boolean;
      P_Fixed       : Integer;
      RV            : Boolean;
      R_Fixed       : Integer;
      Expected      : Counts)
   is
      Hist     : Counts  := (others => 0);
      Previous : Natural := 0;
      Mono_Bad : Natural := 0;
      Never_1  : Boolean := True;
   begin
      for Alt in -1_200 .. 126_700 loop
         declare
            Lvl : constant Natural :=
              (if Vary_Pressure
               then Level (PV, Alt, RV, R_Fixed)
               else Level (PV, P_Fixed, RV, Alt));
         begin
            Hist (Lvl) := Hist (Lvl) + 1;

            if Lvl = 1 then
               Never_1 := False;
            end if;

            if Vary_Pressure and then Lvl < Previous then
               Mono_Bad := Mono_Bad + 1;
            end if;

            Previous := Lvl;
         end;
      end loop;

      if Hist /= Expected then
         Ada.Text_IO.Put_Line ("sweep " & Name & ": histogram differs");
         for L in Counts'Range loop
            Ada.Text_IO.Put_Line
              ("  SL" & Integer'Image (L)
               & ": got" & Natural'Image (Hist (L))
               & ", expected" & Natural'Image (Expected (L)));
         end loop;
      end if;

      Check ("sweep " & Name & ": histogram", Hist = Expected);
      Check ("sweep " & Name & ": SL 1 is never returned", Never_1);

      if Vary_Pressure then
         Check ("sweep " & Name & ": level never decreases (violations"
                & Natural'Image (Mono_Bad) & ")",
                Mono_Bad = 0);
      end if;
   end Sweep;

   procedure Test_Sweeps is
   begin
      --  Pressure altitude varies, no radio altitude.
      --    SL 2 : -1200 .. 2350   =  3 551 values
      --    SL 4 :  2351 .. 5000   =  2 650
      --    SL 5 :  5001 .. 10000  =  5 000
      --    SL 6 : 10001 .. 20000  = 10 000
      --    SL 7 : 20001 .. 126700 = 106 700        total 127 901
      Sweep ("pressure, no radio", True, True, 0, False, 500,
             (2 => 3_551, 4 => 2_650, 5 => 5_000, 6 => 10_000,
              7 => 106_700, others => 0));

      --  Pressure altitude varies, radio altitude valid at 3 000 ft (above
      --  its own bands, so the pressure altitude decides everywhere).
      --    SL 4 : -1200 .. 5000   =  6 201
      --    SL 5 :  5001 .. 10000  =  5 000
      --    SL 6 : 10001 .. 20000  = 10 000
      --    SL 7 : 20001 .. 126700 = 106 700        total 127 901
      Sweep ("pressure, radio 3000", True, True, 0, True, 3_000,
             (4 => 6_201, 5 => 5_000, 6 => 10_000, 7 => 106_700,
              others => 0));

      --  Pressure altitude varies, radio altitude valid at 500 ft: the
      --  radio altitude gives SL 2 for every pressure altitude.
      Sweep ("pressure, radio 500", True, True, 0, True, 500,
             (2 => 127_901, others => 0));

      --  Pressure altitude varies, radio altitude valid at 1 500 ft: SL 3
      --  for every pressure altitude.
      Sweep ("pressure, radio 1500", True, True, 0, True, 1_500,
             (3 => 127_901, others => 0));

      --  Pressure altitude not valid: SL 2 whatever the Value says.
      Sweep ("pressure invalid, radio invalid", True, False, 0, False, 0,
             (2 => 127_901, others => 0));
      Sweep ("pressure invalid, radio 3000", True, False, 0, True, 3_000,
             (2 => 127_901, others => 0));

      --  Radio altitude varies, pressure altitude valid at 15 000 ft.
      --    SL 2 : -1200 .. 1000   =   2 201 values
      --    SL 3 :  1001 .. 2350   =   1 350
      --    SL 6 :  2351 .. 126700 = 124 350  (pressure 15000 is SL 6)
      --                                              total 127 901
      Sweep ("radio, pressure 15000", False, True, 15_000, True, 0,
             (2 => 2_201, 3 => 1_350, 6 => 124_350, others => 0));

      --  Radio altitude varies, pressure altitude valid at 1 500 ft (the
      --  low band, a high valley when the radio altitude is high).
      --    SL 2 : -1200 .. 1000   =   2 201
      --    SL 3 :  1001 .. 2350   =   1 350
      --    SL 4 :  2351 .. 126700 = 124 350  (pressure 1500 is SL 4 once
      --                                       the radio altitude is valid
      --                                       and above its bands)
      Sweep ("radio, pressure 1500", False, True, 1_500, True, 0,
             (2 => 2_201, 3 => 1_350, 4 => 124_350, others => 0));

      --  Radio altitude varies, pressure altitude not valid: SL 2 always.
      Sweep ("radio, pressure invalid", False, False, 15_000, True, 0,
             (2 => 127_901, others => 0));
   end Test_Sweeps;

begin
   Test_Decision_1;
   Test_Decision_2;
   Test_Decision_3;
   Test_Decision_4;
   Test_Bands;
   Test_Sweeps;

   Ada.Text_IO.Put_Line
     ("test_sensitivity:" & Natural'Image (Total) & " checks,"
      & Natural'Image (Failed) & " failed");

   if Failed > 0 then
      Ada.Text_IO.Put_Line ("FAIL");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   else
      Ada.Text_IO.Put_Line ("OK");
   end if;
end Test_Sensitivity;