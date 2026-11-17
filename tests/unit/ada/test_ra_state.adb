--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : tests/unit/ada/test_ra_state.adb
--  Module      : Unit tests of the RA life cycle (plain Ada program)
--  Language    : Ada 2012 (the tests are not in the SPARK subset)
--  Assurance   : DO-178C Software Level B (target), verification artifact
--  Tests       : src/ada/ra/tcas-ra_state.ads and .adb
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Checks every behavior promised in tcas-ra_state.ads: issuing an RA,
--    keeping it, reversing it once, holding it while the threat is quiet,
--    and clearing it after RA_Clear_Cycles quiet cycles. This unit is the
--    first one with memory across cycles, so most tests are SCRIPTS: a
--    list of cycles with the state checked after every one. GNATprove
--    proves the absence of runtime errors and the postconditions; these
--    tests check that the sequences are the right ones.
--
--  DESIGN RULES OF THE TESTS
--    T1  Every scenario starts with Init, so tests do not depend on each
--        other or on their order.
--    T2  After EVERY step of a scenario the whole state is checked (active,
--        sense, age, quiet count, reversal flag, address), not only the
--        event, so a wrong counter cannot hide behind a correct event.
--    T3  Values are written as literals taken from Tcas.Config (all
--        [VERIFY]): RA_Clear_Cycles = 2, response delay 5 s, target rate
--        1 500 ft/min, ALIM 300 ft at SL 3 and 4. When the table is
--        corrected in v2, these tests must change with it, and that is
--        intended.
--    T4  The geometries are worked out by hand and written next to each
--        group (see the table below). The sweep re-checks the contract of
--        the specification (the postconditions and a few extra
--        properties) on 3 888 scripts; it is a consistency check, not an
--        independent oracle, so the hand-worked scenarios are the oracle.
--    T5  Output: one line per failed check, then a summary. The exit
--        status is 1 if anything failed, else 0, so a script can use it.
--
--  GEOMETRIES (own at 10 000 ft, own rate 0, SL 4: ALIM 300, tau 10 gives
--  a gain of 1500 * 5 / 60 = 125 ft to each sense)
--    RA_Level : slant 3 000 at -300 ft/s (tau 10), intruder at 10 000 ft.
--               Classify gives an RA. Up = 125, Down = 125: neither
--               reaches 300, tie, so Generate says Climb (+1 500).
--    Quiet    : slant 150 000, rate 0. Classify gives Other_Traffic.
--    Intruder at 10 200 ft (same slant and rate as RA_Level):
--               Up = 10125 - 10200 = -75, Down = 10200 - 9875 = 325.
--               Down reaches 300, Up does not: a reversal is due.
--    Intruder at 9 800 ft: Up = 325, Down = -75: after a reversal to
--               Descend, the mirror case. Another reversal is due by the
--               numbers but is refused (one per RA).
--    Boundary : intruder at 10 174 ft gives Down = 299 (no reversal);
--               at 10 175 ft gives Down = 300 (reversal).
--    TA_Geom  : slant 3 000 at -150 ft/s (tau 20), intruder at 10 750 ft:
--               a traffic advisory, not an RA (750 is above the RA
--               vertical limit of 600).
--
--  TESTS BY GROUP
--    consistency  : Is_Consistent accepts good states and rejects ten bad
--    init         : empty state, dirty state
--    life cycle   : issue, maintain, clear, quiet after clear
--    flicker      : one quiet cycle then the threat returns
--    reversal     : once, second refused, boundary, not-an-RA, re-issue
--    other tracks : different aircraft, lost track, new RA after clear
--    not an RA    : traffic advisory, one update, SL 1 and 2
--    levels       : every level from 3 to 7 issues a Climb
--    saturation   : Age at 65 535 in Maintained and Clearing
--    sweep        : 3 888 scripts of 4 cycles
--
--  USAGE (from the project root; no project file is needed)
--    mkdir -p build/obj_test build/bin
--    gnatmake tests/unit/ada/test_ra_state.adb -gnat2012 -gnatwa \
--        -D build/obj_test -o build/bin/test_ra_state.exe \
--        -Isrc/ada/common -Isrc/ada/common/types \
--        -Isrc/ada/common/config -Isrc/ada/common/units \
--        -Isrc/ada/tracker -Isrc/ada/threat -Isrc/ada/ra
--    build/bin/test_ra_state.exe
--
--  NOTES
--    * tcas2.gpr does not list tests/, so the project build never sees
--      this program (it has a main procedure and is not part of the
--      product).
--    * If a check fails, recheck the hand arithmetic in the header before
--      blaming the state machine.

with Ada.Command_Line;
with Ada.Text_IO;

with Tcas.RA;
with Tcas.RA_State;
with Tcas.Tracker;
with Tcas.Types;

procedure Test_RA_State is

   package Rg renames Tcas.RA;
   package Rs renames Tcas.RA_State;
   package Tr renames Tcas.Tracker;
   package Ty renames Tcas.Types;

   use type Ty.RA_Sense;
   use type Ty.Vertical_Rate_Fpm;
   use type Ty.Mode_S_Address;
   use type Ty.Range_Rate_Fps;
   use type Rg.RA_Kind;
   use type Rg.Advisory;
   use type Rs.RA_Event;
   use type Rs.State;

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

   S  : Rs.State;
   Ev : Rs.RA_Event := Rs.No_Event;

   No_Adv : constant Rg.Advisory :=
     (Kind => Rg.No_Advisory, Sense => Ty.No_Sense, Target_Rate => 0);
   Climb_Adv : constant Rg.Advisory :=
     (Kind => Rg.Climb, Sense => Ty.Upward, Target_Rate => 1_500);
   Descend_Adv : constant Rg.Advisory :=
     (Kind => Rg.Descend, Sense => Ty.Downward, Target_Rate => -1_500);

   Inactive_State : constant Rs.State :=
     (Active    => False,
      Adv       => No_Adv,
      Address   => 0,
      Age       => 0,
      Clear_Run => 0,
      Reversed  => False);

   --  A state built field by field (the tests need inconsistent ones too).
   function Mk
     (Active : Boolean;
      Adv    : Rg.Advisory;
      Addr   : Natural;
      Age    : Natural;
      Run    : Natural;
      Rev    : Boolean) return Rs.State is
     (Active    => Active,
      Adv       => Adv,
      Address   => Ty.Mode_S_Address (Addr),
      Age       => Rs.Cycle_Count (Age),
      Clear_Run => Rs.Cycle_Count (Run),
      Reversed  => Rev);

   --  An intruder track. Fields the state machine does not read stay 0.
   function Make
     (Addr    : Natural;
      Slant   : Natural;
      RRate   : Integer;
      Alt     : Integer;
      VRate   : Integer;
      Updates : Natural := 2;
      State   : Tr.Track_State := Tr.Active) return Tr.Track is
     (State         => State,
      Address       => Ty.Mode_S_Address (Addr),
      Slant         => Ty.Range_Ft (Slant),
      Range_Rate    => Ty.Range_Rate_Fps (RRate),
      Bearing       => 0,
      Altitude      => Ty.Altitude_Ft (Alt),
      Vertical_Rate => Ty.Vertical_Rate_Fpm (VRate),
      Age           => 0,
      Updates       => Tr.Hit_Count (Updates));

   --  The named geometries of the header (aircraft address 1).
   RA_Level : constant Tr.Track := Make (1, 3_000, -300, 10_000, 0);
   Quiet    : constant Tr.Track := Make (1, 150_000, 0, 10_000, 0);
   TA_Geom  : constant Tr.Track := Make (1, 3_000, -150, 10_750, 0);

   --  One cycle: own at 10 000 ft, rate 0, SL 4 unless stated.
   procedure Do_Step
     (Item  : Tr.Track;
      SL    : Natural := 4;
      Alt   : Integer := 10_000;
      Rate  : Integer := 0) is
   begin
      Rs.Step
        (S, Ty.Altitude_Ft (Alt), Ty.Vertical_Rate_Fpm (Rate),
         Ty.Sensitivity_Level (SL), Item, Ev);
   end Do_Step;

   procedure Expect_Event (Name : String; Expected : Rs.RA_Event) is
   begin
      Check (Name & " (expected " & Rs.RA_Event'Image (Expected)
             & ", got " & Rs.RA_Event'Image (Ev) & ")",
             Ev = Expected);
   end Expect_Event;

   --  Checks every field of the state (rule T2) and the invariant.
   procedure Check_State
     (Name   : String;
      Active : Boolean;
      Sense  : Ty.RA_Sense;
      Age    : Natural;
      Run    : Natural;
      Rev    : Boolean;
      Addr   : Natural) is
   begin
      Check (Name & ": consistent",  Rs.Is_Consistent (S));
      Check (Name & ": active",      S.Active = Active);
      Check (Name & ": sense",       S.Adv.Sense = Sense);
      Check (Name & ": age",         Natural (S.Age) = Age);
      Check (Name & ": quiet count", Natural (S.Clear_Run) = Run);
      Check (Name & ": reversed",    S.Reversed = Rev);
      Check (Name & ": address",     Natural (S.Address) = Addr);
   end Check_State;

   --  Group: consistency (the invariant of the specification)

   procedure Test_Consistency is
   begin
      Check ("consistent: inactive", Rs.Is_Consistent (Inactive_State));
      Check ("consistent: climb",
             Rs.Is_Consistent (Mk (True, Climb_Adv, 1, 1, 0, False)));
      Check ("consistent: climb, one quiet cycle",
             Rs.Is_Consistent (Mk (True, Climb_Adv, 1, 3, 1, False)));
      Check ("consistent: descend after a reversal",
             Rs.Is_Consistent (Mk (True, Descend_Adv, 1, 3, 0, True)));

      Check ("inconsistent: active with no advisory",
             not Rs.Is_Consistent (Mk (True, No_Adv, 1, 1, 0, False)));
      Check ("inconsistent: Upward with kind Descend",
             not Rs.Is_Consistent
                   (Mk (True, (Rg.Descend, Ty.Upward, -1_500),
                        1, 1, 0, False)));
      Check ("inconsistent: Downward with kind Climb",
             not Rs.Is_Consistent
                   (Mk (True, (Rg.Climb, Ty.Downward, 1_500),
                        1, 1, 0, False)));
      Check ("inconsistent: climb at the wrong rate",
             not Rs.Is_Consistent
                   (Mk (True, (Rg.Climb, Ty.Upward, 1_000),
                        1, 1, 0, False)));
      Check ("inconsistent: descend with a positive rate",
             not Rs.Is_Consistent
                   (Mk (True, (Rg.Descend, Ty.Downward, 1_500),
                        1, 1, 0, False)));
      Check ("inconsistent: quiet count at the clear limit",
             not Rs.Is_Consistent (Mk (True, Climb_Adv, 1, 1, 2, False)));
      Check ("inconsistent: inactive with an advisory",
             not Rs.Is_Consistent (Mk (False, Climb_Adv, 1, 1, 0, False)));
      Check ("inconsistent: inactive with the reversal flag",
             not Rs.Is_Consistent (Mk (False, No_Adv, 0, 0, 0, True)));
      Check ("inconsistent: inactive with a quiet count",
             not Rs.Is_Consistent (Mk (False, No_Adv, 0, 0, 1, False)));
      Check ("inconsistent: inactive with a target rate",
             not Rs.Is_Consistent
                   (Mk (False, (Rg.No_Advisory, Ty.No_Sense, 5),
                        0, 0, 0, False)));
   end Test_Consistency;

   --  Group: init

   procedure Test_Init is
   begin
      Rs.Init (S);
      Check_State ("init", False, Ty.No_Sense, 0, 0, False, 0);
      Check ("init: equals the inactive state", S = Inactive_State);

      --  a dirty state is wiped, whatever it held
      S := Mk (True, Descend_Adv, 99, 500, 1, True);
      Rs.Init (S);
      Check_State ("init after dirty", False, Ty.No_Sense, 0, 0, False, 0);
      Check ("init after dirty: equals the inactive state",
             S = Inactive_State);
   end Test_Init;

   --  Group: life cycle, issue, maintain, clear

   procedure Test_Life_Cycle is
   begin
      Rs.Init (S);

      Do_Step (RA_Level);
      Expect_Event ("life: first cycle", Rs.Issued);
      Check_State ("life: issued", True, Ty.Upward, 1, 0, False, 1);
      Check ("life: advisory is a Climb at +1500", S.Adv = Climb_Adv);

      for K in 2 .. 5 loop
         Do_Step (RA_Level);
         Expect_Event ("life: maintained" & Integer'Image (K),
                       Rs.Maintained);
         Check_State ("life: maintained" & Integer'Image (K),
                      True, Ty.Upward, K, 0, False, 1);
      end loop;

      Do_Step (Quiet);
      Expect_Event ("life: first quiet cycle", Rs.Clearing);
      Check_State ("life: clearing", True, Ty.Upward, 6, 1, False, 1);
      Check ("life: advisory held while clearing", S.Adv = Climb_Adv);

      Do_Step (Quiet);
      Expect_Event ("life: second quiet cycle", Rs.Cleared);
      Check_State ("life: cleared", False, Ty.No_Sense, 0, 0, False, 0);
      Check ("life: cleared state equals the inactive state",
             S = Inactive_State);

      Do_Step (Quiet);
      Expect_Event ("life: quiet after clear", Rs.No_Event);
      Check_State ("life: still inactive", False, Ty.No_Sense, 0, 0,
                   False, 0);
   end Test_Life_Cycle;

   --  Group: flicker (one quiet cycle, then the threat returns)

   procedure Test_Flicker is
   begin
      Rs.Init (S);
      Do_Step (RA_Level);
      Expect_Event ("flicker: issue", Rs.Issued);

      Do_Step (Quiet);
      Expect_Event ("flicker: one quiet cycle", Rs.Clearing);
      Check_State ("flicker: clearing", True, Ty.Upward, 2, 1, False, 1);

      Do_Step (RA_Level);
      Expect_Event ("flicker: threat returns", Rs.Maintained);
      Check_State ("flicker: count reset", True, Ty.Upward, 3, 0, False, 1);

      --  the count starts again from zero: one quiet cycle is not enough
      Do_Step (Quiet);
      Expect_Event ("flicker: one quiet cycle again", Rs.Clearing);
      Check_State ("flicker: clearing again", True, Ty.Upward, 4, 1,
                   False, 1);

      Do_Step (Quiet);
      Expect_Event ("flicker: two quiet cycles in a row", Rs.Cleared);
      Check_State ("flicker: cleared", False, Ty.No_Sense, 0, 0, False, 0);
   end Test_Flicker;

   --  Group: reversal

   procedure Test_Reversal is
      Above   : constant Tr.Track := Make (1, 3_000, -300, 10_200, 0);
      Below   : constant Tr.Track := Make (1, 3_000, -300, 9_800, 0);
      Edge_No : constant Tr.Track := Make (1, 3_000, -300, 10_174, 0);
      Edge_Ye : constant Tr.Track := Make (1, 3_000, -300, 10_175, 0);
      Far_Up  : constant Tr.Track := Make (1, 150_000, 0, 10_200, 0);
      Mid     : constant Tr.Track := Make (1, 3_000, -300, 10_100, 0);
   begin
      --  the sense is reversed once, to Descend
      Rs.Init (S);
      Do_Step (RA_Level);
      Expect_Event ("reversal: issue", Rs.Issued);
      Check ("reversal: starts as a Climb", S.Adv = Climb_Adv);

      Do_Step (Above);
      Expect_Event ("reversal: intruder above", Rs.Reversed);
      Check_State ("reversal: reversed", True, Ty.Downward, 2, 0, True, 1);
      Check ("reversal: advisory is a Descend at -1500",
             S.Adv = Descend_Adv);

      --  by the numbers a second reversal is due, but it is refused
      Do_Step (Below);
      Expect_Event ("reversal: second one refused", Rs.Maintained);
      Check_State ("reversal: still Downward", True, Ty.Downward, 3, 0,
                   True, 1);
      Check ("reversal: advisory unchanged", S.Adv = Descend_Adv);

      --  after the RA clears, the reversal is available again
      Do_Step (Quiet);
      Expect_Event ("reversal: quiet 1", Rs.Clearing);
      Check_State ("reversal: clearing", True, Ty.Downward, 4, 1, True, 1);
      Do_Step (Quiet);
      Expect_Event ("reversal: quiet 2", Rs.Cleared);
      Check_State ("reversal: cleared", False, Ty.No_Sense, 0, 0, False, 0);

      Do_Step (RA_Level);
      Expect_Event ("reversal: new RA", Rs.Issued);
      Check_State ("reversal: new RA is a fresh Climb", True, Ty.Upward,
                   1, 0, False, 1);

      --  boundary: the opposite sense reaches 299 ft, not enough
      Rs.Init (S);
      Do_Step (RA_Level);
      Do_Step (Edge_No);
      Expect_Event ("boundary: opposite sense at 299 ft", Rs.Maintained);
      Check_State ("boundary: still Upward", True, Ty.Upward, 2, 0,
                   False, 1);

      --  boundary: the opposite sense reaches 300 ft, enough
      Rs.Init (S);
      Do_Step (RA_Level);
      Do_Step (Edge_Ye);
      Expect_Event ("boundary: opposite sense at 300 ft", Rs.Reversed);
      Check_State ("boundary: now Downward", True, Ty.Downward, 2, 0,
                   True, 1);

      --  neither sense reaches the goal: no reversal (Up 25, Down 225)
      Rs.Init (S);
      Do_Step (RA_Level);
      Do_Step (Mid);
      Expect_Event ("no reversal: opposite sense gives only 225",
                    Rs.Maintained);
      Check_State ("no reversal: still Upward", True, Ty.Upward, 2, 0,
                   False, 1);

      --  the numbers call for a reversal, but the track is no longer an
      --  RA: the quiet count runs, the sense is held
      Rs.Init (S);
      Do_Step (RA_Level);
      Do_Step (Far_Up);
      Expect_Event ("reversal: not an RA any more", Rs.Clearing);
      Check_State ("reversal: held as Upward", True, Ty.Upward, 2, 1,
                   False, 1);
   end Test_Reversal;

   --  Group: other tracks (different aircraft, lost track)

   procedure Test_Other_Tracks is
      Other  : constant Tr.Track := Make (2, 3_000, -300, 10_000, 0);
      Lost   : constant Tr.Track :=
        Make (1, 3_000, -300, 10_000, 0, 2, Tr.Free);
   begin
      --  a different aircraft at RA level does not take over
      Rs.Init (S);
      Do_Step (RA_Level);
      Expect_Event ("other: issue on aircraft 1", Rs.Issued);

      Do_Step (Other);
      Expect_Event ("other: aircraft 2 is not the followed one",
                    Rs.Clearing);
      Check_State ("other: still following aircraft 1", True, Ty.Upward,
                   2, 1, False, 1);
      Check ("other: advisory held", S.Adv = Climb_Adv);

      Do_Step (Other);
      Expect_Event ("other: second quiet cycle for aircraft 1", Rs.Cleared);
      Check_State ("other: cleared", False, Ty.No_Sense, 0, 0, False, 0);

      --  the new RA starts on the NEXT cycle, not on the clearing one
      Do_Step (Other);
      Expect_Event ("other: aircraft 2 now starts an RA", Rs.Issued);
      Check_State ("other: following aircraft 2", True, Ty.Upward, 1, 0,
                   False, 2);

      --  a lost track (the slot became free) counts as quiet
      Rs.Init (S);
      Do_Step (RA_Level);
      Do_Step (Lost);
      Expect_Event ("lost: first cycle without the track", Rs.Clearing);
      Check_State ("lost: held", True, Ty.Upward, 2, 1, False, 1);
      Do_Step (Lost);
      Expect_Event ("lost: second cycle without the track", Rs.Cleared);
      Check_State ("lost: cleared", False, Ty.No_Sense, 0, 0, False, 0);

      --  the track comes back in time: the count resets
      Rs.Init (S);
      Do_Step (RA_Level);
      Do_Step (Lost);
      Do_Step (RA_Level);
      Expect_Event ("lost: track returns in time", Rs.Maintained);
      Check_State ("lost: count reset", True, Ty.Upward, 3, 0, False, 1);
   end Test_Other_Tracks;

   --  Group: situations that must not start an RA

   procedure Test_Not_An_RA is
      One_Update : constant Tr.Track :=
        Make (1, 3_000, -300, 10_000, 0, 1);
   begin
      Rs.Init (S);

      Do_Step (TA_Geom);
      Expect_Event ("not RA: traffic advisory only", Rs.No_Event);
      Check_State ("not RA: TA", False, Ty.No_Sense, 0, 0, False, 0);

      Do_Step (One_Update);
      Expect_Event ("not RA: one update only", Rs.No_Event);
      Check_State ("not RA: one update", False, Ty.No_Sense, 0, 0,
                   False, 0);

      Do_Step (RA_Level, SL => 2);
      Expect_Event ("not RA: SL 2 has no RA", Rs.No_Event);
      Check_State ("not RA: SL 2", False, Ty.No_Sense, 0, 0, False, 0);

      Do_Step (RA_Level, SL => 1);
      Expect_Event ("not RA: SL 1 is standby", Rs.No_Event);
      Check_State ("not RA: SL 1", False, Ty.No_Sense, 0, 0, False, 0);
   end Test_Not_An_RA;

   --  Group: every level that allows an RA

   procedure Test_Levels is
   begin
      for SL in 3 .. 7 loop
         Rs.Init (S);
         Do_Step (RA_Level, SL => SL);
         Expect_Event ("level" & Integer'Image (SL) & ": issued",
                       Rs.Issued);
         Check ("level" & Integer'Image (SL) & ": Climb at +1500",
                S.Adv = Climb_Adv);
      end loop;
   end Test_Levels;

   --  Group: saturation of the age counter

   procedure Test_Saturation is
   begin
      S := Mk (True, Climb_Adv, 1, 65_535, 0, False);
      Do_Step (RA_Level);
      Expect_Event ("saturation: maintained at the top", Rs.Maintained);
      Check_State ("saturation: age stops at 65535", True, Ty.Upward,
                   65_535, 0, False, 1);

      S := Mk (True, Climb_Adv, 1, 65_535, 0, False);
      Do_Step (Quiet);
      Expect_Event ("saturation: clearing at the top", Rs.Clearing);
      Check_State ("saturation: age stops at 65535 while clearing", True,
                   Ty.Upward, 65_535, 1, False, 1);
   end Test_Saturation;

   --  Group: sweep of 4-cycle scripts (rule T4)

   --  The six items a script can use in each cycle:
   --    1 RA_Level, aircraft 1        4 Quiet, aircraft 1
   --    2 intruder at 10 200 ft       5 RA_Level, aircraft 2
   --    3 intruder at  9 800 ft       6 free slot (lost track)
   function Menu (I : Positive) return Tr.Track is
     (case I is
        when 1      => Make (1, 3_000, -300, 10_000, 0),
        when 2      => Make (1, 3_000, -300, 10_200, 0),
        when 3      => Make (1, 3_000, -300, 9_800, 0),
        when 4      => Make (1, 150_000, 0, 10_000, 0),
        when 5      => Make (2, 3_000, -300, 10_000, 0),
        when others => Make (1, 3_000, -300, 10_000, 0, 2, Tr.Free));

   --  The contract of the specification, plus a few extra properties, for
   --  ONE step. Old is the state before, New_S the state after.
   --  Clear_Run is checked with the literals of RA_Clear_Cycles = 2 (T3).
   function Step_Ok
     (Old   : Rs.State;
      New_S : Rs.State;
      E     : Rs.RA_Event;
      Item  : Tr.Track) return Boolean is
     (Rs.Is_Consistent (New_S)
      --  the event and the state agree
      and then (if E in Rs.No_Event | Rs.Cleared then not New_S.Active)
      and then
        (if E in Rs.Issued | Rs.Maintained | Rs.Reversed | Rs.Clearing
         then New_S.Active)
      --  where each event can come from
      and then (if E in Rs.Issued | Rs.No_Event then not Old.Active)
      and then
        (if E in Rs.Maintained | Rs.Reversed | Rs.Clearing | Rs.Cleared
         then Old.Active)
      --  a new RA starts from scratch
      and then
        (if E = Rs.Issued
         then Natural (New_S.Age) = 1
              and then not New_S.Reversed
              and then New_S.Address = Item.Address)
      --  the advisory changes only on Issued, Reversed and Cleared
      and then
        (if E in Rs.Maintained | Rs.Clearing
         then New_S.Adv = Old.Adv and then New_S.Address = Old.Address)
      --  one reversal per RA
      and then
        (if E = Rs.Reversed
         then New_S.Reversed and then not Old.Reversed
              and then New_S.Adv.Sense /= Old.Adv.Sense)
      --  the age counts every cycle of an active RA
      and then
        (if E in Rs.Maintained | Rs.Reversed | Rs.Clearing
         then Natural (New_S.Age) = Natural (Old.Age) + 1)
      --  the quiet count: reset by the threat, one per quiet cycle,
      --  clearing exactly at the second one
      and then
        (if E in Rs.Maintained | Rs.Reversed
         then Natural (New_S.Clear_Run) = 0)
      and then
        (if E = Rs.Clearing
         then Natural (Old.Clear_Run) = 0
              and then Natural (New_S.Clear_Run) = 1)
      and then (if E = Rs.Cleared then Natural (Old.Clear_Run) = 1));

   type Script is array (1 .. 4) of Positive;

   --  Runs one script from Init. True when every step passes Step_Ok and
   --  the RA was never reversed twice.
   function Run_Script (Items : Script; SL : Natural) return Boolean is
      Local   : Rs.State;
      Old     : Rs.State;
      E       : Rs.RA_Event := Rs.No_Event;
      Ok      : Boolean := True;
      Revs    : Natural := 0;
   begin
      Rs.Init (Local);

      for K in Items'Range loop
         Old := Local;
         Rs.Step
           (Local, 10_000, 0, Ty.Sensitivity_Level (SL),
            Menu (Items (K)), E);

         Ok := Ok and then Step_Ok (Old, Local, E, Menu (Items (K)));

         if E = Rs.Issued then
            Revs := 0;
         elsif E = Rs.Reversed then
            Revs := Revs + 1;
         end if;

         Ok := Ok and then Revs <= 1;
      end loop;

      return Ok;
   end Run_Script;

   procedure Sweep is
      SLs    : constant array (1 .. 3) of Natural := (3, 4, 7);
      Combos : Natural := 0;
      Bad    : Natural := 0;
   begin
      for L in SLs'Range loop
         for A in 1 .. 6 loop
            for B in 1 .. 6 loop
               for C in 1 .. 6 loop
                  for D in 1 .. 6 loop
                     Combos := Combos + 1;

                     if not Run_Script ((A, B, C, D), SLs (L)) then
                        if Bad = 0 then
                           Ada.Text_IO.Put_Line
                             ("sweep first mismatch: SL"
                              & Integer'Image (SLs (L))
                              & " script"
                              & Integer'Image (A) & Integer'Image (B)
                              & Integer'Image (C) & Integer'Image (D));
                        end if;
                        Bad := Bad + 1;
                     end if;
                  end loop;
               end loop;
            end loop;
         end loop;
      end loop;

      Check ("sweep: all" & Natural'Image (Combos)
             & " scripts (mismatches" & Natural'Image (Bad) & ")",
             Combos = 3_888 and then Bad = 0);
   end Sweep;

begin
   Test_Consistency;
   Test_Init;
   Test_Life_Cycle;
   Test_Flicker;
   Test_Reversal;
   Test_Other_Tracks;
   Test_Not_An_RA;
   Test_Levels;
   Test_Saturation;
   Sweep;

   Ada.Text_IO.Put_Line
     ("test_ra_state:" & Natural'Image (Total) & " checks,"
      & Natural'Image (Failed) & " failed");

   if Failed > 0 then
      Ada.Text_IO.Put_Line ("FAIL");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   else
      Ada.Text_IO.Put_Line ("OK");
   end if;
end Test_RA_State;