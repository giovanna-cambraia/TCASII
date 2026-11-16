--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : tests/unit/ada/test_threat.adb
--  Module      : Unit tests of the threat detector (plain Ada program)
--  Language    : Ada 2012 (the tests are not in the SPARK subset)
--  Assurance   : DO-178C Software Level B (target), verification artifact
--  Tests       : src/ada/threat/tcas-threat.ads and .adb
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Checks every behavior promised in tcas-threat.ads: the two tests
--    (range and vertical), the zero-threshold rule, the corroboration
--    rule and the order of decisions in Classify. GNATprove proves the
--    absence of runtime errors and the postconditions; these tests check
--    that the numbers are the right ones.
--
--  DESIGN RULES OF THE TESTS
--    T1  No oracle. The expected values are worked out by hand and written
--        next to each check. The sweep checks PROPERTIES that must hold
--        for every input (never an RA below SL 3, never an advisory from
--        one update, ...), not a second copy of the logic.
--    T2  Truncation toward zero is visible in the tau and in the projected
--        altitudes. The cases that show it are marked "truncation".
--    T3  Thresholds are written as literals taken from the current
--        Tcas.Config table (all [VERIFY]). When the table is corrected in
--        v2, these tests must change with it, and that is intended: a
--        threshold edit must never go unnoticed.
--    T4  Every test builds its own tracks (Make), so tests do not depend
--        on each other or on their order. The ownship is at 10 000 ft
--        unless a test says otherwise.
--    T5  Output: one line per failed check, then a summary. The exit
--        status is 1 if anything failed, else 0, so a script can use it.
--
--  THRESHOLDS IN USE (from Tcas.Config, [VERIFY])
--    SL  TA tau  RA tau  TA DMOD  RA DMOD  TA ZTHR  RA ZTHR
--     1     0       0        0        0        0        0
--     2    20       0     1 823       0      850        0
--     3    25      15     2 005    1 215      850      600
--     4    30      20     2 917    2 127      850      600
--     5    40      25     4 557    3 342      850      600
--     6    45      30     6 076    4 861      850      600
--     7    48      35     6 076    6 684    1 200      700
--    Proximate: 36 456 ft range and 1 200 ft altitude. Corroboration: 2.
--
--  TESTS BY GROUP
--    separation   : absolute vertical distance, extremes
--    range test   : zero threshold, DMOD boundary, tau boundary
--    vertical test: zero threshold, boundary, projection, own rate
--    classify     : each level against an RA and a TA geometry
--    corroboration: one update against two
--    boundaries   : tau, DMOD, vertical and proximate limits
--    special cases: free slot, standby, level 7 vertical limit
--    sweep        : 27 216 combinations checked against properties
--
--  USAGE (from the project root; no project file is needed)
--    mkdir -p build/obj_test build/bin
--    gnatmake tests/unit/ada/test_threat.adb -gnat2012 -gnatwa \
--        -D build/obj_test -o build/bin/test_threat.exe \
--        -Isrc/ada/common -Isrc/ada/common/types \
--        -Isrc/ada/common/config -Isrc/ada/common/units \
--        -Isrc/ada/tracker -Isrc/ada/threat
--    build/bin/test_threat.exe
--
--  NOTES
--    * tcas2.gpr does not list tests/, so the project build never sees
--      this program (it has a main procedure and is not part of the
--      product).
--    * If a check fails, recheck the hand arithmetic written next to it
--      before blaming the detector.

with Ada.Command_Line;
with Ada.Text_IO;

with Tcas.Threat;
with Tcas.Tracker;
with Tcas.Types;

procedure Test_Threat is

   package Th renames Tcas.Threat;
   package Tr renames Tcas.Tracker;
   package Ty renames Tcas.Types;

   use type Ty.Altitude_Ft;
   use type Ty.Threat_Class;

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

   --  Fixtures and helpers

   Own_Alt : constant Ty.Altitude_Ft := 10_000;

   --  An intruder track. Fields the detector does not read stay at 0.
   function Make
     (Slant   : Natural;
      RRate   : Integer;
      Alt     : Integer;
      VRate   : Integer;
      Updates : Natural;
      State   : Tr.Track_State := Tr.Active) return Tr.Track is
     (State         => State,
      Address       => 1,
      Slant         => Ty.Range_Ft (Slant),
      Range_Rate    => Ty.Range_Rate_Fps (RRate),
      Bearing       => 0,
      Altitude      => Ty.Altitude_Ft (Alt),
      Vertical_Rate => Ty.Vertical_Rate_Fpm (VRate),
      Age           => 0,
      Updates       => Tr.Hit_Count (Updates));

   --  Range test with plain numbers.
   function RT (T : Tr.Track; Tau : Natural; DMOD : Natural)
     return Boolean is
     (Th.Range_Test (T, Ty.Tau_S (Tau), Ty.Range_Ft (DMOD)));

   --  Vertical test with the ownship at Own_Alt.
   function VT (OwnRate : Integer; T : Tr.Track; ZTHR : Integer)
     return Boolean is
     (Th.Vertical_Test
        (Own_Alt, Ty.Vertical_Rate_Fpm (OwnRate), T,
         Ty.Altitude_Diff_Ft (ZTHR)));

   --  Classification at a sensitivity level, ownship at Own_Alt.
   function Cls
     (SL      : Natural;
      T       : Tr.Track;
      OwnRate : Integer := 0) return Ty.Threat_Class is
     (Th.Classify
        (Own_Alt, Ty.Vertical_Rate_Fpm (OwnRate),
         Ty.Sensitivity_Level (SL), T));

   procedure Expect
     (Name     : String;
      Actual   : Ty.Threat_Class;
      Expected : Ty.Threat_Class) is
   begin
      Check (Name & " (expected " & Ty.Threat_Class'Image (Expected)
             & ", got " & Ty.Threat_Class'Image (Actual) & ")",
             Actual = Expected);
   end Expect;

   type Class_By_SL is array (1 .. 7) of Ty.Threat_Class;

   --  Group: separation

   procedure Test_Separation is
   begin
      Check ("separation: 10000 and 10300",
             Integer (Th.Vertical_Separation (10_000, 10_300)) = 300);
      Check ("separation: symmetric",
             Integer (Th.Vertical_Separation (10_300, 10_000)) = 300);
      Check ("separation: equal altitudes",
             Integer (Th.Vertical_Separation (10_000, 10_000)) = 0);
      Check ("separation: both below zero",
             Integer (Th.Vertical_Separation (-1_200, -200)) = 1_000);
      Check ("separation: the widest pair does not overflow",
             Integer (Th.Vertical_Separation (-1_200, 126_700)) = 127_900);
      Check ("separation: the widest pair, reversed",
             Integer (Th.Vertical_Separation (126_700, -1_200)) = 127_900);
   end Test_Separation;

   --  Group: range test

   procedure Test_Range is
   begin
      --  a zero tau threshold never passes, even when the intruder is on
      --  top of us and closing fast
      Check ("range: zero threshold never passes",
             not RT (Make (100, -100, 10_000, 0, 2), 0, 5_000));

      --  DMOD boundary, intruder not closing (tau = No_Tau)
      Check ("range: slant 2005 passes by DMOD 2005",
             RT (Make (2_005, 50, 10_000, 0, 2), 25, 2_005));
      Check ("range: slant 2000 passes by DMOD 2005",
             RT (Make (2_000, 50, 10_000, 0, 2), 25, 2_005));
      Check ("range: slant 2006 fails, not closing",
             not RT (Make (2_006, 50, 10_000, 0, 2), 25, 2_005));
      Check ("range: rate 0 counts as not closing",
             not RT (Make (2_006, 0, 10_000, 0, 2), 25, 2_005));

      --  tau boundary with DMOD 0: slant 10 000
      --    rate -400: 10000 / 400 = 25
      --    rate -385: 10000 / 385 = 25.97 -> 25 (truncation)
      --    rate -384: 10000 / 384 = 26.04 -> 26
      --    rate -300: 10000 / 300 = 33.3  -> 33
      Check ("range: tau 25 passes a threshold of 25",
             RT (Make (10_000, -400, 10_000, 0, 2), 25, 0));
      Check ("range: tau 25.97 truncates to 25 and passes (truncation)",
             RT (Make (10_000, -385, 10_000, 0, 2), 25, 0));
      Check ("range: tau 26 fails a threshold of 25",
             not RT (Make (10_000, -384, 10_000, 0, 2), 25, 0));
      Check ("range: tau 33 fails a threshold of 25",
             not RT (Make (10_000, -300, 10_000, 0, 2), 25, 0));

      --  DMOD 0 with a slant of 0: on top of us
      Check ("range: slant 0 passes any DMOD",
             RT (Make (0, 0, 10_000, 0, 2), 25, 0));
   end Test_Range;

   --  Group: vertical test (ownship at 10 000 ft, rate 0 unless stated)

   procedure Test_Vertical is
   begin
      --  a zero threshold never passes, even at the same altitude
      Check ("vertical: zero threshold never passes",
             not VT (0, Make (1_000, -100, 10_000, 0, 2), 0));

      --  current separation, intruder not closing (no projection)
      Check ("vertical: separation 850 passes a threshold of 850",
             VT (0, Make (5_000, 0, 10_850, 0, 2), 850));
      Check ("vertical: separation 851 fails",
             not VT (0, Make (5_000, 0, 10_851, 0, 2), 850));
      Check ("vertical: separation 850 below passes",
             VT (0, Make (5_000, 0, 9_150, 0, 2), 850));
      Check ("vertical: separation 851 below fails",
             not VT (0, Make (5_000, 0, 9_149, 0, 2), 850));

      --  projection: slant 10 000 at -400 ft/s gives tau 25 s; the
      --  intruder at 11 000 ft (now 1 000 ft away) descends
      --    -960 ft/min: 11000 + (-960 * 25) / 60 = 11000 - 400 = 10 600
      --                 separation 600
      --    -950 ft/min: -950 * 25 = -23 750 / 60 = -395.8 -> -395
      --                 11000 - 395 = 10 605, separation 605 (truncation)
      Check ("vertical: projected separation 600 passes 600",
             VT (0, Make (10_000, -400, 11_000, -960, 2), 600));
      Check ("vertical: projected separation 605 fails 600 (truncation)",
             not VT (0, Make (10_000, -400, 11_000, -950, 2), 600));

      --  the same geometry, not closing: no projection is made
      Check ("vertical: no projection when not closing",
             not VT (0, Make (10_000, 0, 11_000, -1_200, 2), 600));

      --  the projection uses tau 119, but not tau 120 (= No_Tau)
      --    tau 119: intruder 12000 + (-1000 * 119) / 60 = 12000 - 1983
      --             = 10 017, separation 17
      --    tau 120: no projection, current separation 2 000
      Check ("vertical: tau 119 is projected",
             VT (0, Make (119_000, -1_000, 12_000, -1_000, 2), 600));
      Check ("vertical: tau 120 is the No_Tau value, not projected",
             not VT (0, Make (120_000, -1_000, 12_000, -1_000, 2), 600));

      --  the ownship vertical rate counts too: intruder at 11 000 ft
      --  level, tau 10 s (slant 3 000 at -300 ft/s)
      --    own +3000 ft/min: 10000 + 3000 * 10 / 60 = 10 500, sep 500
      --    own +2000 ft/min: 10000 + 20000 / 60 = 10 333, sep 667
      Check ("vertical: own climb of 3000 ft/min closes the gap",
             VT (3_000, Make (3_000, -300, 11_000, 0, 2), 600));
      Check ("vertical: own climb of 2000 ft/min is not enough",
             not VT (2_000, Make (3_000, -300, 11_000, 0, 2), 600));
      Check ("vertical: no own climb, gap stays 1000",
             not VT (0, Make (3_000, -300, 11_000, 0, 2), 600));
   end Test_Vertical;

   --  Group: classify, each level against two fixed geometries

   procedure Test_Classify_By_Level is
      --  RA geometry: slant 3000 at -300 ft/s (tau 10), 300 ft above
      T_RA : constant Tr.Track := Make (3_000, -300, 10_300, 0, 2);
      Exp_RA : constant Class_By_SL :=
        (1 => Ty.Other_Traffic,
         2 => Ty.Traffic_Advisory,        --  SL 2 has no RA, only a TA
         3 .. 7 => Ty.Resolution_Advisory);

      --  TA geometry: slant 3000 at -150 ft/s (tau 20), 750 ft above;
      --  750 is inside every TA vertical threshold and outside every RA
      --  one (600 and 700)
      T_TA : constant Tr.Track := Make (3_000, -150, 10_750, 0, 2);
      Exp_TA : constant Class_By_SL :=
        (1 => Ty.Other_Traffic,
         2 .. 7 => Ty.Traffic_Advisory);
   begin
      for SL in 1 .. 7 loop
         Expect ("RA geometry, SL" & Integer'Image (SL),
                 Cls (SL, T_RA), Exp_RA (SL));
         Expect ("TA geometry, SL" & Integer'Image (SL),
                 Cls (SL, T_TA), Exp_TA (SL));
      end loop;
   end Test_Classify_By_Level;

   --  Group: corroboration (rule R4)

   procedure Test_Corroboration is
   begin
      --  the RA geometry with one update: proximate at most
      Expect ("one update, SL3: proximate only",
              Cls (3, Make (3_000, -300, 10_300, 0, 1)),
              Ty.Proximate_Traffic);
      Expect ("one update, SL2: proximate only",
              Cls (2, Make (3_000, -300, 10_300, 0, 1)),
              Ty.Proximate_Traffic);
      Expect ("one update, SL7: proximate only",
              Cls (7, Make (3_000, -300, 10_300, 0, 1)),
              Ty.Proximate_Traffic);
      Expect ("one update, SL1: still standby",
              Cls (1, Make (3_000, -300, 10_300, 0, 1)),
              Ty.Other_Traffic);

      --  the second update lifts it
      Expect ("two updates, SL3: RA",
              Cls (3, Make (3_000, -300, 10_300, 0, 2)),
              Ty.Resolution_Advisory);

      --  the counter at its limit
      Expect ("255 updates, SL3: RA",
              Cls (3, Make (3_000, -300, 10_300, 0, 255)),
              Ty.Resolution_Advisory);

      --  a track with no update at all
      Expect ("zero updates, SL3: proximate only",
              Cls (3, Make (3_000, -300, 10_300, 0, 0)),
              Ty.Proximate_Traffic);
   end Test_Corroboration;

   --  Group: boundaries of the decision chain, at SL 4
   --    RA tau 20, RA DMOD 2 127, RA ZTHR 600
   --    TA tau 30, TA DMOD 2 917, TA ZTHR 850

   procedure Test_Boundaries is
   begin
      --  tau: slant 6000, above both DMODs, alt 300 above
      --    rate -300: 6000 / 300 = 20      -> RA (20 <= 20)
      --    rate -299: 6000 / 299 = 20.07   -> 20, still RA (truncation)
      --    rate -285: 6000 / 285 = 21.05   -> 21, no RA, TA (21 <= 30)
      --    rate -200: 6000 / 200 = 30      -> TA (30 <= 30)
      --    rate -199: 6000 / 199 = 30.15   -> 30, still TA (truncation)
      --    rate -190: 6000 / 190 = 31.58   -> 31, no TA, proximate
      Expect ("tau 20: RA",
              Cls (4, Make (6_000, -300, 10_300, 0, 2)),
              Ty.Resolution_Advisory);
      Expect ("tau 20.07 truncates to 20: RA (truncation)",
              Cls (4, Make (6_000, -299, 10_300, 0, 2)),
              Ty.Resolution_Advisory);
      Expect ("tau 21: TA",
              Cls (4, Make (6_000, -285, 10_300, 0, 2)),
              Ty.Traffic_Advisory);
      Expect ("tau 30: TA",
              Cls (4, Make (6_000, -200, 10_300, 0, 2)),
              Ty.Traffic_Advisory);
      Expect ("tau 30.15 truncates to 30: TA (truncation)",
              Cls (4, Make (6_000, -199, 10_300, 0, 2)),
              Ty.Traffic_Advisory);
      Expect ("tau 31: proximate",
              Cls (4, Make (6_000, -190, 10_300, 0, 2)),
              Ty.Proximate_Traffic);

      --  DMOD: intruder not closing, alt 300 above
      Expect ("slant 2127 not closing: RA by DMOD",
              Cls (4, Make (2_127, 0, 10_300, 0, 2)),
              Ty.Resolution_Advisory);
      Expect ("slant 2128 not closing: TA",
              Cls (4, Make (2_128, 0, 10_300, 0, 2)),
              Ty.Traffic_Advisory);
      Expect ("slant 2917 not closing: TA by DMOD",
              Cls (4, Make (2_917, 0, 10_300, 0, 2)),
              Ty.Traffic_Advisory);
      Expect ("slant 2918 not closing: proximate",
              Cls (4, Make (2_918, 0, 10_300, 0, 2)),
              Ty.Proximate_Traffic);

      --  vertical: slant 3000 at -300 ft/s (tau 10), rates 0
      Expect ("separation 600: RA",
              Cls (4, Make (3_000, -300, 10_600, 0, 2)),
              Ty.Resolution_Advisory);
      Expect ("separation 601: TA",
              Cls (4, Make (3_000, -300, 10_601, 0, 2)),
              Ty.Traffic_Advisory);
      Expect ("separation 850: TA",
              Cls (4, Make (3_000, -300, 10_850, 0, 2)),
              Ty.Traffic_Advisory);
      Expect ("separation 851: proximate",
              Cls (4, Make (3_000, -300, 10_851, 0, 2)),
              Ty.Proximate_Traffic);

      --  vertical rates change the class: intruder 1 000 ft above
      Expect ("separation 1000, rates 0: proximate",
              Cls (4, Make (3_000, -300, 11_000, 0, 2)),
              Ty.Proximate_Traffic);
      Expect ("intruder descending 3000 ft/min: RA (projected 500)",
              Cls (4, Make (3_000, -300, 11_000, -3_000, 2)),
              Ty.Resolution_Advisory);
      Expect ("own climb 3000 ft/min: RA (projected 500)",
              Cls (4, Make (3_000, -300, 11_000, 0, 2), 3_000),
              Ty.Resolution_Advisory);
      Expect ("own climb 1500 ft/min: TA (projected 750)",
              Cls (4, Make (3_000, -300, 11_000, 0, 2), 1_500),
              Ty.Traffic_Advisory);
   end Test_Boundaries;

   --  Group: proximate traffic

   procedure Test_Proximate is
   begin
      Expect ("proximate: slant 36456, separation 1200",
              Cls (4, Make (36_456, 0, 11_200, 0, 2)),
              Ty.Proximate_Traffic);
      Expect ("proximate: slant 36457 is other traffic",
              Cls (4, Make (36_457, 0, 11_200, 0, 2)),
              Ty.Other_Traffic);
      Expect ("proximate: separation 1201 is other traffic",
              Cls (4, Make (36_456, 0, 11_201, 0, 2)),
              Ty.Other_Traffic);
      Expect ("proximate: 1200 ft below",
              Cls (4, Make (20_000, 0, 8_800, 0, 2)),
              Ty.Proximate_Traffic);
      Expect ("proximate: 1201 ft below is other traffic",
              Cls (4, Make (20_000, 0, 8_799, 0, 2)),
              Ty.Other_Traffic);
      Expect ("proximate: far away and far above",
              Cls (4, Make (150_000, 0, 50_000, 0, 2)),
              Ty.Other_Traffic);
   end Test_Proximate;

   --  Group: special cases

   procedure Test_Special is
   begin
      --  a free slot is never a threat, whatever it holds
      Expect ("free slot, SL4",
              Cls (4, Make (100, -300, 10_100, 0, 255, Tr.Free)),
              Ty.Other_Traffic);
      Expect ("free slot, SL7",
              Cls (7, Make (0, -3_000, 10_000, 0, 255, Tr.Free)),
              Ty.Other_Traffic);

      --  standby: no alert even for a collision course
      Expect ("standby: collision course",
              Cls (1, Make (0, -3_000, 10_000, 0, 255)),
              Ty.Other_Traffic);

      --  level 7 has a wider TA vertical limit (1 200 ft, [VERIFY]):
      --  slant 3000 at -150 ft/s (tau 20), 1 200 ft above
      Expect ("separation 1200, SL7: TA",
              Cls (7, Make (3_000, -150, 11_200, 0, 2)),
              Ty.Traffic_Advisory);
      Expect ("separation 1200, SL6: proximate",
              Cls (6, Make (3_000, -150, 11_200, 0, 2)),
              Ty.Proximate_Traffic);
      Expect ("separation 1201, SL7: other traffic",
              Cls (7, Make (36_457, 0, 11_201, 0, 2)),
              Ty.Other_Traffic);

      --  level 7 RA vertical limit is 700 ft, not 600
      Expect ("separation 700, SL7: RA",
              Cls (7, Make (3_000, -300, 10_700, 0, 2)),
              Ty.Resolution_Advisory);
      Expect ("separation 700, SL6: TA (RA limit is 600)",
              Cls (6, Make (3_000, -300, 10_700, 0, 2)),
              Ty.Traffic_Advisory);

      --  an aircraft on the ground-level altitude edge of the type range
      Expect ("intruder at the lowest altitude, far below",
              Cls (4, Make (3_000, -300, -1_200, 0, 2)),
              Ty.Other_Traffic);
      Expect ("intruder at the highest altitude, far above",
              Cls (4, Make (3_000, -300, 126_700, 0, 2)),
              Ty.Other_Traffic);
   end Test_Special;

   --  Group: sweep, checked against properties (rule T1)

   --  Every combination of the values below goes through Classify twice,
   --  with one update and with two. Properties that must hold for ALL of
   --  them:
   --    P1  one update never gives more than Proximate_Traffic
   --    P2  the second update never lowers the class
   --    P3  an RA exists only from SL 3 up
   --    P4  SL 1 is always Other_Traffic
   --    P5  a zero range threshold and a zero vertical threshold never
   --        pass, whatever the geometry
   procedure Sweep is
      type Int_Array is array (Positive range <>) of Integer;

      Slants : constant Int_Array :=
        (0, 500, 1_215, 1_216, 2_005, 3_000, 6_076, 6_077,
         20_000, 36_456, 36_457, 150_000);                     --  12
      Rates  : constant Int_Array :=
        (-3_000, -300, -150, -1, 0, 300);                      --   6
      Alts   : constant Int_Array :=
        (9_000, 9_400, 9_700, 10_000, 10_300, 10_700, 10_750,
         11_200, 11_201);                                      --   9
      VRates : constant Int_Array := (-1_200, 0, 1_200);       --   3
      ORates : constant Int_Array := (-1_000, 0);              --   2

      Combos : Natural := 0;
      Bad    : Natural := 0;
   begin
      for SL in 1 .. 7 loop
         for Si in Slants'Range loop
            for Ri in Rates'Range loop
               for Ai in Alts'Range loop
                  for Vi in VRates'Range loop
                     for Oi in ORates'Range loop
                        declare
                           T1 : constant Tr.Track :=
                             Make (Slants (Si), Rates (Ri), Alts (Ai),
                                   VRates (Vi), 1);
                           T2 : constant Tr.Track :=
                             Make (Slants (Si), Rates (Ri), Alts (Ai),
                                   VRates (Vi), 2);
                           C1 : constant Ty.Threat_Class :=
                             Cls (SL, T1, ORates (Oi));
                           C2 : constant Ty.Threat_Class :=
                             Cls (SL, T2, ORates (Oi));
                           Ok : constant Boolean :=
                             C1 <= Ty.Proximate_Traffic
                             and then C1 <= C2
                             and then (if C2 = Ty.Resolution_Advisory
                                       then SL >= 3)
                             and then (if SL = 1
                                       then C1 = Ty.Other_Traffic
                                            and then C2 = Ty.Other_Traffic)
                             and then not RT (T2, 0, 5_000)
                             and then not VT (ORates (Oi), T2, 0);
                        begin
                           Combos := Combos + 1;

                           if not Ok then
                              if Bad = 0 then
                                 Ada.Text_IO.Put_Line
                                   ("sweep first mismatch: SL"
                                    & Integer'Image (SL)
                                    & " slant" & Integer'Image (Slants (Si))
                                    & " rate" & Integer'Image (Rates (Ri))
                                    & " alt" & Integer'Image (Alts (Ai))
                                    & " vrate" & Integer'Image (VRates (Vi))
                                    & " own rate"
                                    & Integer'Image (ORates (Oi)));
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
             Combos = 27_216 and then Bad = 0);
   end Sweep;

begin
   Test_Separation;
   Test_Range;
   Test_Vertical;
   Test_Classify_By_Level;
   Test_Corroboration;
   Test_Boundaries;
   Test_Proximate;
   Test_Special;
   Sweep;

   Ada.Text_IO.Put_Line
     ("test_threat:" & Natural'Image (Total) & " checks,"
      & Natural'Image (Failed) & " failed");

   if Failed > 0 then
      Ada.Text_IO.Put_Line ("FAIL");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   else
      Ada.Text_IO.Put_Line ("OK");
   end if;
end Test_Threat;