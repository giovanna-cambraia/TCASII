--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : tests/unit/ada/test_ra.adb
--  Module      : Unit tests of the RA generation (plain Ada program)
--  Language    : Ada 2012 (the tests are not in the SPARK subset)
--  Assurance   : DO-178C Software Level B (target), verification artifact
--  Tests       : src/ada/ra/tcas-ra.ads and .adb
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Checks every behavior promised in tcas-ra.ads: the time used for the
--    projection, the pilot response model, the signed separation of each
--    sense, the six decisions of Choose_Sense in their order, and the
--    advisory built from the sense. GNATprove proves the absence of
--    runtime errors and the postconditions; these tests check that the
--    numbers are the right ones.
--
--  DESIGN RULES OF THE TESTS
--    T1  Sense_Separation is checked against numbers worked out by hand
--        and written next to each check. The Choose_Sense checks then use
--        geometries whose separations are known from those numbers.
--    T2  Truncation toward zero is visible in the own-rate term and in the
--        intruder projection. The cases that show it are marked
--        "truncation": floor division would give a different answer.
--    T3  Values are written as literals taken from Tcas.Config (all
--        [VERIFY]): response delay 5 s, target rate 1 500 ft/min, and the
--        separation goals ALIM 300 / 300 / 350 / 400 / 600 ft for SL 3 to
--        7. When the table is corrected in v2, these tests must change
--        with it, and that is intended.
--    T4  Every test builds its own track (Make). The ownship is at
--        10 000 ft with a vertical rate of 0 unless a test says otherwise.
--    T5  The sweep re-derives the decision from Sense_Separation (already
--        checked by hand) and compares it with Choose_Sense. It is a
--        consistency check of the decision chain over many geometries, not
--        an independent oracle: the hand-worked cases are the oracle.
--    T6  Output: one line per failed check, then a summary. The exit
--        status is 1 if anything failed, else 0, so a script can use it.
--
--  HOW TO READ THE GEOMETRIES (own at 10 000 ft, rate 0)
--    "tau 25" is Make (10 000, -400, ...): slant 10 000 at -400 ft/s.
--    "tau 10" is Make (3 000, -300, ...).
--    Both senses gain the same distance from the pilot response: 125 ft at
--    tau 10 (1 500 * 5 / 60) and 500 ft at tau 25 (1 500 * 20 / 60).
--    So with d = own projected altitude minus intruder projected altitude:
--      Upward   separation = d + gain
--      Downward separation = gain - d
--
--  TESTS BY GROUP
--    effective tau : closing, not closing, truncation, clamp
--    separation    : delay split, truncation, signs, rates, ceiling, floor
--    choose sense  : six decisions, boundaries, non-crossing, ALIM table
--    generate      : every field of both advisories
--    sweep         : 15 750 geometries checked against the decision chain
--
--  USAGE (from the project root; no project file is needed)
--    mkdir -p build/obj_test build/bin
--    gnatmake tests/unit/ada/test_ra.adb -gnat2012 -gnatwa \
--        -D build/obj_test -o build/bin/test_ra.exe \
--        -Isrc/ada/common -Isrc/ada/common/types \
--        -Isrc/ada/common/config -Isrc/ada/common/units \
--        -Isrc/ada/tracker -Isrc/ada/ra
--    build/bin/test_ra.exe
--
--  NOTES
--    * tcas2.gpr does not list tests/, so the project build never sees
--      this program (it has a main procedure and is not part of the
--      product).
--    * If a check fails, recheck the hand arithmetic written next to it
--      before blaming the generator.

with Ada.Command_Line;
with Ada.Text_IO;

with Tcas.RA;
with Tcas.Tracker;
with Tcas.Types;

procedure Test_RA is

   package Rg renames Tcas.RA;
   package Tr renames Tcas.Tracker;
   package Ty renames Tcas.Types;

   use type Ty.RA_Sense;
   use type Rg.RA_Kind;

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

   --  An active, corroborated intruder track. Fields the generator does
   --  not read stay at 0.
   function Make
     (Slant : Natural;
      RRate : Integer;
      Alt   : Integer;
      VRate : Integer) return Tr.Track is
     (State         => Tr.Active,
      Address       => 1,
      Slant         => Ty.Range_Ft (Slant),
      Range_Rate    => Ty.Range_Rate_Fps (RRate),
      Bearing       => 0,
      Altitude      => Ty.Altitude_Ft (Alt),
      Vertical_Rate => Ty.Vertical_Rate_Fpm (VRate),
      Age           => 0,
      Updates       => 2);

   function Tau_Of (T : Tr.Track) return Integer is
     (Integer (Rg.Effective_Tau (T)));

   function Sep
     (Own_Alt  : Integer;
      Own_Rate : Integer;
      Sense    : Ty.RA_Sense;
      T        : Tr.Track) return Integer is
     (Integer
        (Rg.Sense_Separation
           (Ty.Altitude_Ft (Own_Alt), Ty.Vertical_Rate_Fpm (Own_Rate),
            Sense, T)));

   function Sense_Of
     (SL       : Natural;
      T        : Tr.Track;
      Own_Alt  : Integer := 10_000;
      Own_Rate : Integer := 0) return Ty.RA_Sense is
     (Rg.Choose_Sense
        (Ty.Altitude_Ft (Own_Alt), Ty.Vertical_Rate_Fpm (Own_Rate),
         Ty.Sensitivity_Level (SL), T));

   procedure Expect
     (Name     : String;
      Actual   : Ty.RA_Sense;
      Expected : Ty.RA_Sense) is
   begin
      Check (Name & " (expected " & Ty.RA_Sense'Image (Expected)
             & ", got " & Ty.RA_Sense'Image (Actual) & ")",
             Actual = Expected);
   end Expect;

   --  Separation goal of a level, as a literal (rule T3).
   function ALIM_Of (SL : Natural) return Integer is
     (case SL is
        when 3 | 4 => 300,
        when 5     => 350,
        when 6     => 400,
        when others => 600);

   --  Group: effective tau

   procedure Test_Effective_Tau is
   begin
      Check ("tau: 10000 at -400 is 25",
             Tau_Of (Make (10_000, -400, 10_000, 0)) = 25);
      Check ("tau: 25.97 truncates to 25 (truncation)",
             Tau_Of (Make (10_000, -385, 10_000, 0)) = 25);
      Check ("tau: 26.04 is 26",
             Tau_Of (Make (10_000, -384, 10_000, 0)) = 26);
      Check ("tau: rate 0 is not closing, tau 0 (not 120)",
             Tau_Of (Make (10_000, 0, 10_000, 0)) = 0);
      Check ("tau: opening rate is not closing, tau 0",
             Tau_Of (Make (10_000, 300, 10_000, 0)) = 0);
      Check ("tau: slant 0 closing is 0",
             Tau_Of (Make (0, -100, 10_000, 0)) = 0);
      Check ("tau: slant 1 at -3000 truncates to 0 (truncation)",
             Tau_Of (Make (1, -3_000, 10_000, 0)) = 0);
      Check ("tau: 119 stays 119",
             Tau_Of (Make (119_000, -1_000, 10_000, 0)) = 119);
      Check ("tau: 121 clamps to 120",
             Tau_Of (Make (121_000, -1_000, 10_000, 0)) = 120);
      Check ("tau: 150000 at -1 clamps to 120",
             Tau_Of (Make (150_000, -1, 10_000, 0)) = 120);
   end Test_Effective_Tau;

   --  Group: separation of each sense (own at 10 000 ft)

   procedure Test_Separation is
      T : Tr.Track;
   begin
      --  tau 10: delay part 5, after part 5. Target term = 1500*5/60 =
      --  7500/60 = 125. Intruder level, own rate 0.
      T := Make (3_000, -300, 10_000, 0);
      Check ("sep tau 10: Upward is 125",
             Sep (10_000, 0, Ty.Upward, T) = 125);
      Check ("sep tau 10: Downward is 125",
             Sep (10_000, 0, Ty.Downward, T) = 125);

      --  tau 25: after part 20, term = 1500*20/60 = 500
      T := Make (10_000, -400, 10_000, 0);
      Check ("sep tau 25: Upward is 500",
             Sep (10_000, 0, Ty.Upward, T) = 500);
      Check ("sep tau 25: Downward is 500",
             Sep (10_000, 0, Ty.Downward, T) = 500);

      --  tau 3: closest approach before the delay ends, the target rate is
      --  never used. Own rate 1200: 1200*3/60 = 60.
      T := Make (900, -300, 10_000, 0);
      Check ("sep tau 3, own rate 1200: Upward is 60",
             Sep (10_000, 1_200, Ty.Upward, T) = 60);
      Check ("sep tau 3, own rate 1200: Downward is -60",
             Sep (10_000, 1_200, Ty.Downward, T) = -60);
      Check ("sep tau 3, own rate 0: the target is not used",
             Sep (10_000, 0, Ty.Upward, T) = 0
             and then Sep (10_000, 0, Ty.Downward, T) = 0);

      --  tau 5: exactly the delay, still no target term. 1200*5/60 = 100.
      T := Make (1_500, -300, 10_000, 0);
      Check ("sep tau 5, own rate 1200: Upward is 100",
             Sep (10_000, 1_200, Ty.Upward, T) = 100);
      Check ("sep tau 5, own rate 0: both are 0",
             Sep (10_000, 0, Ty.Upward, T) = 0
             and then Sep (10_000, 0, Ty.Downward, T) = 0);

      --  tau 6: one second of target rate: 1500*1/60 = 25
      T := Make (1_800, -300, 10_000, 0);
      Check ("sep tau 6: Upward is 25",
             Sep (10_000, 0, Ty.Upward, T) = 25);
      Check ("sep tau 6: Downward is 25",
             Sep (10_000, 0, Ty.Downward, T) = 25);

      --  tau 7, own rate 100: own term 100*5/60 = 8.33 -> 8; target term
      --  1500*2/60 = 50. Upward 10000+8+50 = 10058, sep 58. Downward
      --  10000+8-50 = 9958, sep 42.
      T := Make (2_800, -400, 10_000, 0);
      Check ("sep tau 7, own +100: Upward is 58",
             Sep (10_000, 100, Ty.Upward, T) = 58);
      Check ("sep tau 7, own +100: Downward is 42",
             Sep (10_000, 100, Ty.Downward, T) = 42);

      --  own rate -100: -100*5/60 = -8.33 -> -8 (truncation, floor -9).
      --  Upward 10000-8+50 = 10042, sep 42. Downward 9942, sep 58.
      Check ("sep tau 7, own -100: Upward is 42 (truncation)",
             Sep (10_000, -100, Ty.Upward, T) = 42);
      Check ("sep tau 7, own -100: Downward is 58 (truncation)",
             Sep (10_000, -100, Ty.Downward, T) = 58);

      --  intruder descending 600 ft/min at tau 10: 10000 - 100 = 9900.
      --  Upward 10125-9900 = 225. Downward 9900-9875 = 25.
      T := Make (3_000, -300, 10_000, -600);
      Check ("sep intruder -600: Upward is 225",
             Sep (10_000, 0, Ty.Upward, T) = 225);
      Check ("sep intruder -600: Downward is 25",
             Sep (10_000, 0, Ty.Downward, T) = 25);

      --  intruder -95 ft/min: -950/60 = -15.83 -> -15 (truncation), 9985.
      --  Upward 10125-9985 = 140. Downward 9985-9875 = 110.
      T := Make (3_000, -300, 10_000, -95);
      Check ("sep intruder -95: Upward is 140 (truncation)",
             Sep (10_000, 0, Ty.Upward, T) = 140);
      Check ("sep intruder -95: Downward is 110 (truncation)",
             Sep (10_000, 0, Ty.Downward, T) = 110);

      --  intruder climbing 600: 10100. Upward 10125-10100 = 25.
      --  Downward 10100-9875 = 225.
      T := Make (3_000, -300, 10_000, 600);
      Check ("sep intruder +600: Upward is 25",
             Sep (10_000, 0, Ty.Upward, T) = 25);
      Check ("sep intruder +600: Downward is 225",
             Sep (10_000, 0, Ty.Downward, T) = 225);

      --  not closing: tau 0, the current geometry
      T := Make (3_000, 0, 10_200, 0);
      Check ("sep not closing, intruder 200 above: Upward is -200",
             Sep (10_000, 0, Ty.Upward, T) = -200);
      Check ("sep not closing, intruder 200 above: Downward is 200",
             Sep (10_000, 0, Ty.Downward, T) = 200);

      T := Make (3_000, 0, 10_000, 0);
      Check ("sep not closing, level: both are 0",
             Sep (10_000, 0, Ty.Upward, T) = 0
             and then Sep (10_000, 0, Ty.Downward, T) = 0);

      --  ceiling: own at 126 700 ft, intruder level, tau 25. Upward
      --  would reach 127 200, clamped to 126 700, sep 0. Downward
      --  126 200, sep 500.
      T := Make (10_000, -400, 126_700, 0);
      Check ("sep ceiling: Upward clamps to 0",
             Sep (126_700, 0, Ty.Upward, T) = 0);
      Check ("sep ceiling: Downward is 500",
             Sep (126_700, 0, Ty.Downward, T) = 500);

      --  floor: own at -1 200 ft. Upward -700, sep 500. Downward would
      --  reach -1 700, clamped to -1 200, sep 0.
      T := Make (10_000, -400, -1_200, 0);
      Check ("sep floor: Upward is 500",
             Sep (-1_200, 0, Ty.Upward, T) = 500);
      Check ("sep floor: Downward clamps to 0",
             Sep (-1_200, 0, Ty.Downward, T) = 0);
   end Test_Separation;

   --  Group: choice of the sense (SL 4, ALIM 300, unless stated)

   procedure Test_Choose_Sense is
   begin
      --  tau 25, gain 500, intruder level-ish. Up = d + 500, Down = 500 - d
      --  with d = 10000 - intruder altitude (intruder rate 0).

      --  decision 1: both reach 300, own above or level -> Upward
      Expect ("d1: level, both 500",
              Sense_Of (4, Make (10_000, -400, 10_000, 0)), Ty.Upward);
      Expect ("d1: intruder 200 below, up 700 down 300",
              Sense_Of (4, Make (10_000, -400, 9_800, 0)), Ty.Upward);

      --  decision 3: only Upward reaches 300 (up 701, down 299)
      Expect ("d3: intruder 201 below, only Upward reaches",
              Sense_Of (4, Make (10_000, -400, 9_799, 0)), Ty.Upward);

      --  decision 2: both reach 300, own below -> Downward
      Expect ("d2: intruder 200 above, up 300 down 700",
              Sense_Of (4, Make (10_000, -400, 10_200, 0)), Ty.Downward);
      Expect ("d2: intruder 1 above, up 499 down 501",
              Sense_Of (4, Make (10_000, -400, 10_001, 0)), Ty.Downward);

      --  decision 4: only Downward reaches 300 (up 299, down 701)
      Expect ("d4: intruder 201 above, only Downward reaches",
              Sense_Of (4, Make (10_000, -400, 10_201, 0)), Ty.Downward);

      --  rule R5: the non-crossing sense wins even when the other gives
      --  more separation.
      --    intruder 9900 climbing 600: projected 9900+250 = 10150.
      --    up 10500-10150 = 350, down 10150-9500 = 650. Own is above NOW.
      Expect ("R5: own above, Upward kept although Downward gives 650",
              Sense_Of (4, Make (10_000, -400, 9_900, 600)), Ty.Upward);
      --    intruder 10100 descending 600: projected 10100-250 = 9850.
      --    up 10500-9850 = 650, down 9850-9500 = 350. Own is below NOW.
      Expect ("R5: own below, Downward kept although Upward gives 650",
              Sense_Of (4, Make (10_000, -400, 10_100, -600)), Ty.Downward);

      --  crossing is chosen when only that sense reaches the goal
      --    intruder 10100 descending 1200: projected 10100-500 = 9600.
      --    up 10500-9600 = 900, down 9600-9500 = 100 (below the goal).
      Expect ("crossing: own below, only Upward reaches",
              Sense_Of (4, Make (10_000, -400, 10_100, -1_200)), Ty.Upward);
      --    intruder 9900 climbing 1200: projected 9900+500 = 10400.
      --    up 10500-10400 = 100, down 10400-9500 = 900.
      Expect ("crossing: own above, only Downward reaches",
              Sense_Of (4, Make (10_000, -400, 9_900, 1_200)), Ty.Downward);

      --  tau 10, gain 125: neither sense reaches 300.
      --  decision 5 and 6: the larger separation wins, ties go Upward
      Expect ("d5: tie 125 and 125 goes Upward",
              Sense_Of (4, Make (3_000, -300, 10_000, 0)), Ty.Upward);
      Expect ("d5: intruder 50 below, up 175 down 75",
              Sense_Of (4, Make (3_000, -300, 9_950, 0)), Ty.Upward);
      Expect ("d6: intruder 50 above, up 75 down 175",
              Sense_Of (4, Make (3_000, -300, 10_050, 0)), Ty.Downward);
      Expect ("d6: intruder 1 above, up 124 down 126",
              Sense_Of (4, Make (3_000, -300, 10_001, 0)), Ty.Downward);

      --  not closing (tau 0): the current geometry decides
      Expect ("not closing: intruder above, Downward",
              Sense_Of (4, Make (3_000, 0, 10_100, 0)), Ty.Downward);
      Expect ("not closing: intruder below, Upward",
              Sense_Of (4, Make (3_000, 0, 9_900, 0)), Ty.Upward);
      Expect ("not closing: level, tie goes Upward",
              Sense_Of (4, Make (3_000, 0, 10_000, 0)), Ty.Upward);

      --  ceiling and floor: never advise a climb into the ceiling
      Expect ("ceiling: own at 126700, Upward is 0, so Downward",
              Sense_Of (4, Make (10_000, -400, 126_700, 0),
                        Own_Alt => 126_700),
              Ty.Downward);
      Expect ("floor: own at -1200, Downward is 0, so Upward",
              Sense_Of (4, Make (10_000, -400, -1_200, 0),
                        Own_Alt => -1_200),
              Ty.Upward);
   end Test_Choose_Sense;

   --  The goal ALIM of each level flips the answer for one geometry:
   --  tau 25, intruder 9 900 climbing 600 (up 350, down 650, own above).
   --    SL 3, 4 (300): both reach, own above    -> Upward
   --    SL 5    (350): up 350 >= 350, both reach -> Upward
   --    SL 6    (400): up 350 < 400, only Down   -> Downward
   --    SL 7    (600): up 350 < 600, down 650    -> Downward
   procedure Test_ALIM_Table is
      T : constant Tr.Track := Make (10_000, -400, 9_900, 600);
   begin
      Expect ("ALIM: SL3 (300)", Sense_Of (3, T), Ty.Upward);
      Expect ("ALIM: SL4 (300)", Sense_Of (4, T), Ty.Upward);
      Expect ("ALIM: SL5 (350)", Sense_Of (5, T), Ty.Upward);
      Expect ("ALIM: SL6 (400)", Sense_Of (6, T), Ty.Downward);
      Expect ("ALIM: SL7 (600)", Sense_Of (7, T), Ty.Downward);
   end Test_ALIM_Table;

   --  Group: generate

   procedure Test_Generate is
      Up   : constant Rg.Advisory :=
        Rg.Generate (10_000, 0, 4, Make (10_000, -400, 10_000, 0));
      Down : constant Rg.Advisory :=
        Rg.Generate (10_000, 0, 4, Make (10_000, -400, 10_200, 0));
   begin
      Check ("generate up: kind is Climb",        Up.Kind = Rg.Climb);
      Check ("generate up: sense is Upward",      Up.Sense = Ty.Upward);
      Check ("generate up: target is +1500",
             Integer (Up.Target_Rate) = 1_500);

      Check ("generate down: kind is Descend",    Down.Kind = Rg.Descend);
      Check ("generate down: sense is Downward",  Down.Sense = Ty.Downward);
      Check ("generate down: target is -1500",
             Integer (Down.Target_Rate) = -1_500);
   end Test_Generate;

   --  Group: sweep (rule T5)

   --  Every combination of the values below goes through Choose_Sense and
   --  Generate. For each one the decision is re-derived from the two
   --  separations (checked by hand above) and compared. Properties:
   --    P1  the sense is never No_Sense
   --    P2  both reach ALIM: the non-crossing sense (Upward if own is
   --        above or level, else Downward)
   --    P3  exactly one reaches ALIM: that one
   --    P4  neither reaches ALIM: the larger separation, ties Upward
   --    P5  Generate matches the sense: Climb at +1500, Descend at -1500
   procedure Sweep is
      type Int_Array is array (Positive range <>) of Integer;

      Slants : constant Int_Array :=
        (0, 900, 3_000, 10_000, 30_000, 119_000);              --  6
      Rates  : constant Int_Array := (-3_000, -400, -300, 0, 300);  --  5
      ORates : constant Int_Array := (-1_500, 0, 1_500);       --  3
      Alts   : constant Int_Array :=
        (9_000, 9_700, 9_900, 10_000, 10_100, 10_300, 11_000); --  7
      VRates : constant Int_Array :=
        (-1_500, -600, 0, 600, 1_500);                         --  5

      Combos : Natural := 0;
      Bad    : Natural := 0;
   begin
      for SL in 3 .. 7 loop
         for Si in Slants'Range loop
            for Ri in Rates'Range loop
               for Oi in ORates'Range loop
                  for Ai in Alts'Range loop
                     for Vi in VRates'Range loop
                        declare
                           T : constant Tr.Track :=
                             Make (Slants (Si), Rates (Ri), Alts (Ai),
                                   VRates (Vi));

                           Up_Sep : constant Integer :=
                             Sep (10_000, ORates (Oi), Ty.Upward, T);
                           Dn_Sep : constant Integer :=
                             Sep (10_000, ORates (Oi), Ty.Downward, T);

                           Up_Ok : constant Boolean := Up_Sep >= ALIM_Of (SL);
                           Dn_Ok : constant Boolean := Dn_Sep >= ALIM_Of (SL);

                           Expected : constant Ty.RA_Sense :=
                             (if Up_Ok and then Dn_Ok
                              then (if Alts (Ai) <= 10_000
                                    then Ty.Upward else Ty.Downward)
                              elsif Up_Ok then Ty.Upward
                              elsif Dn_Ok then Ty.Downward
                              elsif Up_Sep >= Dn_Sep then Ty.Upward
                              else Ty.Downward);

                           S : constant Ty.RA_Sense :=
                             Sense_Of (SL, T, Own_Rate => ORates (Oi));

                           Gen : constant Rg.Advisory :=
                             Rg.Generate
                               (10_000, Ty.Vertical_Rate_Fpm (ORates (Oi)),
                                Ty.Sensitivity_Level (SL), T);

                           Ok : constant Boolean :=
                             S /= Ty.No_Sense
                             and then S = Expected
                             and then Gen.Sense = S
                             and then
                               (if Gen.Sense = Ty.Upward
                                then Gen.Kind = Rg.Climb
                                     and then Integer (Gen.Target_Rate)
                                                = 1_500
                                else Gen.Kind = Rg.Descend
                                     and then Integer (Gen.Target_Rate)
                                                = -1_500);
                        begin
                           Combos := Combos + 1;

                           if not Ok then
                              if Bad = 0 then
                                 Ada.Text_IO.Put_Line
                                   ("sweep first mismatch: SL"
                                    & Integer'Image (SL)
                                    & " slant" & Integer'Image (Slants (Si))
                                    & " rate" & Integer'Image (Rates (Ri))
                                    & " own rate"
                                    & Integer'Image (ORates (Oi))
                                    & " alt" & Integer'Image (Alts (Ai))
                                    & " vrate" & Integer'Image (VRates (Vi)));
                              end if;
                              Bad := Bad + 1;
                           end if;
                        end;
                     end loop;
                  end loop;
               end loop;
            end loop;
         end loop;
      end loop;

      Check ("sweep: all" & Natural'Image (Combos)
             & " combinations (mismatches" & Natural'Image (Bad) & ")",
             Combos = 15_750 and then Bad = 0);
   end Sweep;

begin
   Test_Effective_Tau;
   Test_Separation;
   Test_Choose_Sense;
   Test_ALIM_Table;
   Test_Generate;
   Sweep;

   Ada.Text_IO.Put_Line
     ("test_ra:" & Natural'Image (Total) & " checks,"
      & Natural'Image (Failed) & " failed");

   if Failed > 0 then
      Ada.Text_IO.Put_Line ("FAIL");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   else
      Ada.Text_IO.Put_Line ("OK");
   end if;
end Test_RA;