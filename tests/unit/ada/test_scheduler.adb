--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : tests/unit/ada/test_scheduler.adb
--  Module      : Unit tests of the cycle scheduler (plain Ada program)
--  Language    : Ada 2012 (the tests are not in the SPARK subset)
--  Assurance   : DO-178C Software Level B (target), verification artifact
--  Tests       : src/ada/scheduler/tcas-scheduler.ads and .adb
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Checks every behavior promised in tcas-scheduler.ads: the choice of
--    the most urgent intruder, and the five steps of one cycle (aging,
--    replies, level, intruder to follow, RA life cycle). This is the first
--    unit that wires the others together, so most tests are SCRIPTS of
--    cycles driven through Run_Cycle, with replies that go through the
--    REAL tracker: the rates and the corroboration come from the tracker's
--    own arithmetic. GNATprove proves the absence of runtime errors and
--    the postconditions; these tests check that the wiring is right.
--
--  DESIGN RULES OF THE TESTS
--    T1  Every scenario starts with Init, so tests do not depend on each
--        other or on their order.
--    T2  After EVERY cycle of a scenario the whole output is checked
--        (level, event, advisory, selected track, its class, dropped and
--        rejected counts) and the invariant of the RA state, not only the
--        event.
--    T3  Values are written as literals taken from Tcas.Config (all
--        [VERIFY]): the SL 4 thresholds (RA tau 20, RA DMOD 2 127, RA ZTHR
--        600, ALIM 300, TA tau 30, TA DMOD 2 917, TA ZTHR 850), the
--        response delay 5 s, the target rate 1 500 ft/min, RA_Clear_Cycles
--        2, Max_Track_Age_Ms 5 000. When the table is corrected in v2,
--        these tests must change with it, and that is intended.
--    T4  The geometries are worked out by hand (see below). The sweep
--        re-checks the contract of the specification on 7 776 scripts; it
--        is a consistency check of the wiring, not an independent oracle,
--        so the hand-worked scenarios are the oracle.
--    T5  Output: one line per failed check, then a summary. The exit
--        status is 1 if anything failed, else 0, so a script can use it.
--
--  GEOMETRIES (ownship at 4 000 ft, rate 0, pressure valid, radio
--  altitude not valid: Select_Level gives SL 4)
--    Replies come as (address, slant, altitude). The tracker derives the
--    range rate from two successive replies one cycle (1 000 ms) apart:
--    rate = (new slant - old slant) * 1000 / 1000 ft/s.
--    Approach : slant 3 300, then 3 000 at 4 000 ft. Rate -300 ft/s, tau
--               10, intruder level. RA at SL 4. Up = 125, Down = 125 (the
--               gain of 1500 * 5 / 60 = 125 ft), neither reaches ALIM 300,
--               tie, so the advisory is a Climb at +1 500.
--    Away     : slant 150 000. The range rate clamps to +3 000 ft/s: not
--               closing, far away, Other_Traffic: a quiet cycle.
--    One update: a track with one update is Proximate_Traffic at most
--               (corroboration, rule R4 of Tcas.Threat).
--    SL 2     : with the pressure altitude not valid the level is 2: the
--               same approach is a traffic advisory, never an RA.
--
--  TESTS BY GROUP
--    most urgent  : class, tau, slant, index, free slots, own altitude,
--                   level, one-update tracks, not-closing intruders
--    init         : empty state, dirty state, empty cycle
--    life cycle   : insert, issue, maintain, move away, clear, quiet
--    fail safe    : pressure altitude not valid (no RA; active RA clears)
--    own rate     : invalid rate counts as 0, valid rate changes the RA
--    aging        : 5 000 ms kept, 5 001 ms dropped, maximum elapsed
--    full table   : 60 replies, Rejected, refresh while full, 256 replies
--    two aircraft : the RA keeps its own aircraft, a new one starts after
--    sweep        : 7 776 scripts of 4 cycles
--
--  USAGE (from the project root; no project file is needed)
--    mkdir -p build/obj_test build/bin
--    gnatmake tests/unit/ada/test_scheduler.adb -gnat2012 -gnatwa \
--        -D build/obj_test -o build/bin/test_scheduler.exe \
--        -Isrc/ada/common -Isrc/ada/common/types \
--        -Isrc/ada/common/config -Isrc/ada/common/units \
--        -Isrc/ada/tracker -Isrc/ada/threat -Isrc/ada/ra \
--        -Isrc/ada/scheduler
--    build/bin/test_scheduler.exe
--
--  NOTES
--    * tcas2.gpr does not list tests/, so the project build never sees
--      this program (it has a main procedure and is not part of the
--      product).
--    * If a check fails, recheck the hand arithmetic in the header and in
--      the comments before blaming the scheduler.
--    * Three tests document behavior that is an open item for v2:
--      a traffic advisory from an untrusted pressure altitude, a
--      not-closing intruder ranking as more urgent than a closing one
--      (its effective tau is 0), and a Descend advised to an ownship that
--      is already climbing at 3 000 ft/min.
--  ===========================================================================

with Ada.Command_Line;
with Ada.Text_IO;

with Tcas.RA;
with Tcas.RA_State;
with Tcas.Scheduler;
with Tcas.Tracker;
with Tcas.Types;

procedure Test_Scheduler is

   package Rg renames Tcas.RA;
   package Rs renames Tcas.RA_State;
   package Sc renames Tcas.Scheduler;
   package Tr renames Tcas.Tracker;
   package Ty renames Tcas.Types;

   use type Rg.Advisory;
   use type Rs.RA_Event;
   use type Tr.Track_State;
   use type Ty.Threat_Class;
   use type Ty.Vertical_Rate_Fpm;

   --  ------------------------------------------------------------------
   --  Test bookkeeping
   --  ------------------------------------------------------------------

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

   --  ------------------------------------------------------------------
   --  Constants
   --  ------------------------------------------------------------------

   No_Adv : constant Rg.Advisory :=
     (Kind => Rg.No_Advisory, Sense => Ty.No_Sense, Target_Rate => 0);
   Climb_Adv : constant Rg.Advisory :=
     (Kind => Rg.Climb, Sense => Ty.Upward, Target_Rate => 1_500);
   Descend_Adv : constant Rg.Advisory :=
     (Kind => Rg.Descend, Sense => Ty.Downward, Target_Rate => -1_500);

   --  Normal ownship: 4 000 ft, rate 0, pressure valid, radio not valid.
   Own_Normal : constant Sc.Ownship_Data :=
     (Pressure   => (Valid => True,  Value => 4_000),
      Radio      => (Valid => False, Value => 0),
      Rate_Valid => True,
      Rate       => 0);

   --  Pressure altitude NOT valid (the value is kept at 4 000 ft so the
   --  numbers of the geometries still apply; the level is 2).
   Own_Bad_Pressure : constant Sc.Ownship_Data :=
     (Pressure   => (Valid => False, Value => 4_000),
      Radio      => (Valid => False, Value => 0),
      Rate_Valid => True,
      Rate       => 0);

   --  Rate not valid: the value 3 000 must be ignored (counts as 0).
   Own_Rate_Invalid : constant Sc.Ownship_Data :=
     (Pressure   => (Valid => True,  Value => 4_000),
      Radio      => (Valid => False, Value => 0),
      Rate_Valid => False,
      Rate       => 3_000);

   --  Rate valid: the ownship climbs at 3 000 ft/min.
   Own_Rate_Valid : constant Sc.Ownship_Data :=
     (Pressure   => (Valid => True,  Value => 4_000),
      Radio      => (Valid => False, Value => 0),
      Rate_Valid => True,
      Rate       => 3_000);

   --  ------------------------------------------------------------------
   --  Helpers
   --  ------------------------------------------------------------------

   function Meas
     (Addr  : Natural;
      Slant : Natural;
      Alt   : Integer) return Tr.Measurement is
     (Address  => Ty.Mode_S_Address (Addr),
      Slant    => Ty.Range_Ft (Slant),
      Bearing  => 0,
      Altitude => Ty.Altitude_Ft (Alt));

   --  A track for the Most_Urgent tests. The vertical rate is 0.
   function Make
     (Slant   : Natural;
      RRate   : Integer;
      Alt     : Integer;
      Updates : Natural := 2;
      State   : Tr.Track_State := Tr.Active) return Tr.Track is
     (State         => State,
      Address       => 1,
      Slant         => Ty.Range_Ft (Slant),
      Range_Rate    => Ty.Range_Rate_Fps (RRate),
      Bearing       => 0,
      Altitude      => Ty.Altitude_Ft (Alt),
      Vertical_Rate => 0,
      Age           => 0,
      Updates       => Tr.Hit_Count (Updates));

   --  The replies of the cycle being built (the first N are passed on).
   Buf : Sc.Measurement_Array (1 .. 300) :=
     (others => Meas (0, 0, 0));

   --  The cycle state and the output of the last cycle.
   C : Sc.Cycle_State;
   O : Sc.Cycle_Output;

   procedure Put_Reply
     (I    : Positive;
      Addr : Natural;
      Slant : Natural;
      Alt  : Integer) is
   begin
      Buf (I) := Meas (Addr, Slant, Alt);
   end Put_Reply;

   --  Runs one cycle with the first N replies of Buf.
   procedure Cyc
     (Own     : Sc.Ownship_Data;
      N       : Natural;
      Elapsed : Natural := 1_000) is
   begin
      Sc.Run_Cycle (C, Own, Buf (1 .. N), Ty.Time_Ms (Elapsed), O);
   end Cyc;

   --  Checks the whole output of a cycle (rule T2).
   procedure Check_Out
     (Name     : String;
      Level    : Natural;
      Event    : Rs.RA_Event;
      Adv      : Rg.Advisory;
      Selected : Natural;
      Class    : Ty.Threat_Class;
      Dropped  : Natural := 0;
      Rejected : Natural := 0) is
   begin
      Check (Name & ": level", Natural (O.Level) = Level);
      Check (Name & ": event (expected " & Rs.RA_Event'Image (Event)
             & ", got " & Rs.RA_Event'Image (O.Event) & ")",
             O.Event = Event);
      Check (Name & ": advisory", O.Adv = Adv);
      Check (Name & ": advisory is the one in the state",
             O.Adv = C.RA.Adv);
      Check (Name & ": selected track", O.Selected = Selected);
      Check (Name & ": class (expected " & Ty.Threat_Class'Image (Class)
             & ", got " & Ty.Threat_Class'Image (O.Selected_Class) & ")",
             O.Selected_Class = Class);
      Check (Name & ": dropped", O.Dropped = Dropped);
      Check (Name & ": rejected", O.Rejected = Rejected);
      Check (Name & ": RA state consistent", Rs.Is_Consistent (C.RA));
   end Check_Out;

   --  Checks the life cycle state after a cycle.
   procedure Check_RA
     (Name   : String;
      Active : Boolean;
      Addr   : Natural;
      Age    : Natural;
      Run    : Natural) is
   begin
      Check (Name & ": RA active", C.RA.Active = Active);
      Check (Name & ": RA address", Natural (C.RA.Address) = Addr);
      Check (Name & ": RA age", Natural (C.RA.Age) = Age);
      Check (Name & ": RA quiet count", Natural (C.RA.Clear_Run) = Run);
   end Check_RA;

   --  Runs the two cycles that issue the standard RA (aircraft 1).
   --  Cycle 1 inserts, cycle 2 refreshes with a rate of -300 ft/s.
   procedure Issue_Standard_RA is
   begin
      Sc.Init (C);
      Put_Reply (1, 1, 3_300, 4_000);
      Cyc (Own_Normal, 1);
      Put_Reply (1, 1, 3_000, 4_000);
      Cyc (Own_Normal, 1);
   end Issue_Standard_RA;

   --  ------------------------------------------------------------------
   --  Group: most urgent
   --  ------------------------------------------------------------------

   Tab : Tr.Track_Table;

   --  Most urgent track with the ownship at 4 000 ft, rate 0, SL 4.
   function Urgent (T : Tr.Track_Table) return Natural is
     (Sc.Most_Urgent (T, 4_000, 0, 4));

   procedure Test_Most_Urgent is
      --  classes at SL 4, own at 4 000 ft:
      --    RA_T    : tau 10, level            -> Resolution_Advisory
      --    RA_Fast : tau 7 (7.5 truncated)    -> Resolution_Advisory
      --    TA_T    : tau 20, 750 ft above     -> Traffic_Advisory
      --              (750 is above the RA limit of 600)
      --    Prox_T  : 20 000 ft, not closing   -> Proximate_Traffic
      --    Other_T : 100 000 ft, not closing  -> Other_Traffic
      RA_T    : constant Tr.Track := Make (3_000, -300, 4_000);
      RA_Fast : constant Tr.Track := Make (3_000, -400, 4_000);
      TA_T    : constant Tr.Track := Make (3_000, -150, 4_750);
      Prox_T  : constant Tr.Track := Make (20_000, 0, 4_000);
      Other_T : constant Tr.Track := Make (100_000, 0, 4_000);

      One_Update : constant Tr.Track :=
        Make (3_000, -300, 4_000, Updates => 1);   --  Proximate only
   begin
      --  empty table and a single track
      Tr.Init (Tab);
      Check ("urgent: empty table", Urgent (Tab) = Tr.No_Track);

      Tab (5) := Other_T;
      Check ("urgent: single track in slot 5", Urgent (Tab) = 5);

      --  class decides first: the RA wins wherever it sits
      Tr.Init (Tab);
      Tab (1) := Other_T;
      Tab (2) := Prox_T;
      Tab (3) := TA_T;
      Tab (4) := RA_T;
      Tab (5) := TA_T;
      Check ("urgent: RA beats TA, proximate and other", Urgent (Tab) = 4);

      Tr.Init (Tab);
      Tab (1) := RA_T;
      Tab (2) := Other_T;
      Tab (3) := TA_T;
      Check ("urgent: RA in slot 1 stays", Urgent (Tab) = 1);

      Tr.Init (Tab);
      Tab (1) := TA_T;
      Tab (2) := Prox_T;
      Check ("urgent: TA beats proximate (TA first)", Urgent (Tab) = 1);

      Tr.Init (Tab);
      Tab (1) := Prox_T;
      Tab (2) := TA_T;
      Check ("urgent: TA beats proximate (TA second)", Urgent (Tab) = 2);

      Tr.Init (Tab);
      Tab (1) := Other_T;
      Tab (2) := Prox_T;
      Check ("urgent: proximate beats other", Urgent (Tab) = 2);

      --  same class: the smaller tau wins
      Tr.Init (Tab);
      Tab (1) := RA_T;
      Tab (2) := RA_Fast;
      Check ("urgent: tau 7 beats tau 10", Urgent (Tab) = 2);

      Tr.Init (Tab);
      Tab (1) := RA_Fast;
      Tab (2) := RA_T;
      Check ("urgent: tau 7 beats tau 10 (swapped)", Urgent (Tab) = 1);

      --  complete tie: the lowest index stays (rule R6)
      Tr.Init (Tab);
      Tab (3)  := RA_T;
      Tab (7)  := RA_T;
      Tab (40) := RA_T;
      Check ("urgent: complete tie keeps the lowest index",
             Urgent (Tab) = 3);

      --  same class, same tau: the smaller slant wins
      --    3000/299 = 10.03 and 2999/299 = 10.03 both truncate to 10
      Tr.Init (Tab);
      Tab (1) := Make (3_000, -299, 4_000);
      Tab (2) := Make (2_999, -299, 4_000);
      Check ("urgent: equal tau 10, slant 2999 beats 3000",
             Urgent (Tab) = 2);

      Tr.Init (Tab);
      Tab (1) := Make (2_999, -299, 4_000);
      Tab (2) := Make (3_000, -299, 4_000);
      Check ("urgent: equal tau 10, slant 2999 beats 3000 (swapped)",
             Urgent (Tab) = 1);

      --  tau 10 both, but different slants (3000/300 and 2000/200)
      Tr.Init (Tab);
      Tab (1) := RA_T;
      Tab (2) := Make (2_000, -200, 4_000);
      Check ("urgent: equal tau 10, slant 2000 beats 3000",
             Urgent (Tab) = 2);

      --  a free slot is never chosen, whatever it holds
      Tr.Init (Tab);
      Tab (1) := Make (3_000, -300, 4_000, State => Tr.Free);
      Tab (2) := Other_T;
      Check ("urgent: free slot with RA-like fields is ignored",
             Urgent (Tab) = 2);

      Tab := (others => Make (3_000, -300, 4_000, State => Tr.Free));
      Check ("urgent: all slots free", Urgent (Tab) = Tr.No_Track);

      --  corroboration: one update is proximate at most
      Tr.Init (Tab);
      Tab (1) := One_Update;
      Tab (2) := Other_T;
      Check ("urgent: one-update track (proximate) beats other",
             Urgent (Tab) = 1);

      Tr.Init (Tab);
      Tab (1) := Other_T;
      Tab (2) := One_Update;
      Check ("urgent: one-update track (proximate) beats other (swapped)",
             Urgent (Tab) = 2);

      Tr.Init (Tab);
      Tab (1) := One_Update;
      Tab (2) := TA_T;
      Check ("urgent: a real TA beats a one-update RA geometry",
             Urgent (Tab) = 2);

      --  a not-closing intruder has effective tau 0, so among proximate
      --  tracks it ranks above a closing one (documented behavior, open
      --  item for v2). Closing: 30 000 ft at -600 ft/s is tau 50.
      Tr.Init (Tab);
      Tab (1) := Make (30_000, -600, 4_000);
      Tab (2) := Make (20_000, 0, 4_000);
      Check ("urgent: not closing (tau 0) ranks above closing (tau 50)",
             Urgent (Tab) = 2);

      Tr.Init (Tab);
      Tab (1) := Make (20_000, 0, 4_000);
      Tab (2) := Make (30_000, -600, 4_000);
      Check ("urgent: not closing in slot 1 stays",
             Urgent (Tab) = 1);

      --  the ownship altitude is used: the intruder level with us is the
      --  threat. Slot 1 is at 4 000 ft, slot 2 at 20 000 ft.
      Tr.Init (Tab);
      Tab (1) := Make (3_000, -300, 4_000);
      Tab (2) := Make (3_000, -300, 20_000);
      Check ("urgent: own at 4000 ft, slot 1 is the threat",
             Sc.Most_Urgent (Tab, 4_000, 0, 4) = 1);
      Check ("urgent: own at 20000 ft, slot 2 is the threat",
             Sc.Most_Urgent (Tab, 20_000, 0, 4) = 2);

      --  the level is used: at SL 1 everything is Other_Traffic, so the
      --  tau decides (50 against an effective 0).
      Tr.Init (Tab);
      Tab (1) := Make (30_000, -600, 4_000);
      Tab (2) := Make (100_000, 0, 4_000);
      Check ("urgent: SL 4, proximate beats other",
             Sc.Most_Urgent (Tab, 4_000, 0, 4) = 1);
      Check ("urgent: SL 1, everything is other, tau decides",
             Sc.Most_Urgent (Tab, 4_000, 0, 1) = 2);
   end Test_Most_Urgent;

   --  ------------------------------------------------------------------
   --  Group: init and the empty cycle
   --  ------------------------------------------------------------------

   procedure Test_Init is
   begin
      --  a state that holds an RA, then Init wipes it
      Issue_Standard_RA;
      Check ("init: before, an RA is active", C.RA.Active);

      Sc.Init (C);
      Check ("init: no tracks", Tr.Count (C.Table) = 0);
      Check ("init: every slot is free",
             (for all I in Ty.Track_Index => C.Table (I).State = Tr.Free));
      Check ("init: no RA", not C.RA.Active);
      Check ("init: no advisory", C.RA.Adv = No_Adv);
      Check ("init: state consistent", Rs.Is_Consistent (C.RA));

      --  a cycle with no replies on an empty table
      Cyc (Own_Normal, 0);
      Check_Out ("empty cycle", 4, Rs.No_Event, No_Adv, 0,
                 Ty.Other_Traffic);
      Check ("empty cycle: still no tracks", Tr.Count (C.Table) = 0);
      Check_RA ("empty cycle", False, 0, 0, 0);
   end Test_Init;

   --  ------------------------------------------------------------------
   --  Group: life cycle
   --  ------------------------------------------------------------------

   procedure Test_Life_Cycle is
   begin
      Sc.Init (C);

      --  cycle 1: the first reply creates a track, one update only
      Put_Reply (1, 1, 3_300, 4_000);
      Cyc (Own_Normal, 1);
      Check_Out ("life 1", 4, Rs.No_Event, No_Adv, 1, Ty.Proximate_Traffic);
      Check ("life 1: one track", Tr.Count (C.Table) = 1);
      Check_RA ("life 1", False, 0, 0, 0);

      --  cycle 2: the second reply gives the rates (-300 ft/s), the track
      --  is corroborated and an RA starts, a Climb by the tie rule
      Put_Reply (1, 1, 3_000, 4_000);
      Cyc (Own_Normal, 1);
      Check_Out ("life 2", 4, Rs.Issued, Climb_Adv, 1,
                 Ty.Resolution_Advisory);
      Check_RA ("life 2", True, 1, 1, 0);

      --  cycles 3 and 4: the same aircraft, same advisory
      Put_Reply (1, 1, 2_700, 4_000);
      Cyc (Own_Normal, 1);
      Check_Out ("life 3", 4, Rs.Maintained, Climb_Adv, 1,
                 Ty.Resolution_Advisory);
      Check_RA ("life 3", True, 1, 2, 0);

      Put_Reply (1, 1, 2_400, 4_000);
      Cyc (Own_Normal, 1);
      Check_Out ("life 4", 4, Rs.Maintained, Climb_Adv, 1,
                 Ty.Resolution_Advisory);
      Check_RA ("life 4", True, 1, 3, 0);

      --  cycle 5: the intruder moves away (rate clamps to +3 000 ft/s):
      --  first quiet cycle, the advisory is held
      Put_Reply (1, 1, 150_000, 4_000);
      Cyc (Own_Normal, 1);
      Check_Out ("life 5", 4, Rs.Clearing, Climb_Adv, 1, Ty.Other_Traffic);
      Check_RA ("life 5", True, 1, 4, 1);

      --  cycle 6: second quiet cycle, the RA ends
      Put_Reply (1, 1, 150_000, 4_000);
      Cyc (Own_Normal, 1);
      Check_Out ("life 6", 4, Rs.Cleared, No_Adv, 1, Ty.Other_Traffic);
      Check_RA ("life 6", False, 0, 0, 0);

      --  cycle 7: nothing more to report
      Put_Reply (1, 1, 150_000, 4_000);
      Cyc (Own_Normal, 1);
      Check_Out ("life 7", 4, Rs.No_Event, No_Adv, 1, Ty.Other_Traffic);
      Check_RA ("life 7", False, 0, 0, 0);
   end Test_Life_Cycle;

   --  ------------------------------------------------------------------
   --  Group: fail safe (pressure altitude not valid)
   --  ------------------------------------------------------------------

   procedure Test_Fail_Safe is
   begin
      --  from the start: SL 2, the approach is a TA, never an RA
      Sc.Init (C);

      Put_Reply (1, 1, 3_300, 4_000);
      Cyc (Own_Bad_Pressure, 1);
      Check_Out ("failsafe 1", 2, Rs.No_Event, No_Adv, 1,
                 Ty.Proximate_Traffic);

      Put_Reply (1, 1, 3_000, 4_000);
      Cyc (Own_Bad_Pressure, 1);
      Check_Out ("failsafe 2", 2, Rs.No_Event, No_Adv, 1,
                 Ty.Traffic_Advisory);

      Put_Reply (1, 1, 2_700, 4_000);
      Cyc (Own_Bad_Pressure, 1);
      Check_Out ("failsafe 3", 2, Rs.No_Event, No_Adv, 1,
                 Ty.Traffic_Advisory);
      Check_RA ("failsafe 3", False, 0, 0, 0);

      --  an RA that is active when the pressure altitude is lost: SL 2,
      --  no longer an RA, the advisory is held for one cycle, then clears
      Issue_Standard_RA;
      Check_RA ("failsafe active", True, 1, 1, 0);

      Put_Reply (1, 1, 2_700, 4_000);
      Cyc (Own_Bad_Pressure, 1);
      Check_Out ("failsafe clearing", 2, Rs.Clearing, Climb_Adv, 1,
                 Ty.Traffic_Advisory);
      Check_RA ("failsafe clearing", True, 1, 2, 1);

      Put_Reply (1, 1, 2_400, 4_000);
      Cyc (Own_Bad_Pressure, 1);
      Check_Out ("failsafe cleared", 2, Rs.Cleared, No_Adv, 1,
                 Ty.Traffic_Advisory);
      Check_RA ("failsafe cleared", False, 0, 0, 0);
   end Test_Fail_Safe;

   --  ------------------------------------------------------------------
   --  Group: own vertical rate
   --  ------------------------------------------------------------------

   procedure Test_Own_Rate is
   begin
      --  intruder 1 000 ft above, same approach (tau 10).
      --  Rate not valid: the value 3 000 counts as 0. Separation stays
      --  1 000 ft, above the RA (600) and TA (850) limits: proximate.
      Sc.Init (C);
      Put_Reply (1, 1, 3_300, 5_000);
      Cyc (Own_Rate_Invalid, 1);
      Check_Out ("rate invalid 1", 4, Rs.No_Event, No_Adv, 1,
                 Ty.Proximate_Traffic);

      Put_Reply (1, 1, 3_000, 5_000);
      Cyc (Own_Rate_Invalid, 1);
      Check_Out ("rate invalid 2", 4, Rs.No_Event, No_Adv, 1,
                 Ty.Proximate_Traffic);

      --  Rate valid at 3 000 ft/min: in the 10 s to closest approach the
      --  ownship reaches 4 500 ft, 500 ft below the intruder, inside the
      --  RA limit of 600: an RA. The sense: climbing makes it worse, so a
      --  Descend is advised (Up -625, Down 875, ALIM 300).
      Sc.Init (C);
      Put_Reply (1, 1, 3_300, 5_000);
      Cyc (Own_Rate_Valid, 1);
      Check_Out ("rate valid 1", 4, Rs.No_Event, No_Adv, 1,
                 Ty.Proximate_Traffic);

      Put_Reply (1, 1, 3_000, 5_000);
      Cyc (Own_Rate_Valid, 1);
      Check_Out ("rate valid 2", 4, Rs.Issued, Descend_Adv, 1,
                 Ty.Resolution_Advisory);
      Check_RA ("rate valid 2", True, 1, 1, 0);
   end Test_Own_Rate;

   --  ------------------------------------------------------------------
   --  Group: aging and lost tracks
   --  ------------------------------------------------------------------

   procedure Test_Aging is
   begin
      --  5 000 ms of silence keeps the track (the limit is "more than"),
      --  and the stale data still gives an RA
      Issue_Standard_RA;

      Cyc (Own_Normal, 0, Elapsed => 5_000);
      Check_Out ("aging 5000", 4, Rs.Maintained, Climb_Adv, 1,
                 Ty.Resolution_Advisory);
      Check ("aging 5000: track kept", Tr.Count (C.Table) = 1);
      Check_RA ("aging 5000", True, 1, 2, 0);

      --  one more millisecond and the track is dropped: the followed
      --  aircraft is lost, a quiet cycle, the advisory is held
      Cyc (Own_Normal, 0, Elapsed => 1);
      Check_Out ("aging 5001", 4, Rs.Clearing, Climb_Adv, 0,
                 Ty.Other_Traffic, Dropped => 1);
      Check ("aging 5001: no tracks", Tr.Count (C.Table) = 0);
      Check_RA ("aging 5001", True, 1, 3, 1);

      Cyc (Own_Normal, 0);
      Check_Out ("aging cleared", 4, Rs.Cleared, No_Adv, 0,
                 Ty.Other_Traffic);
      Check_RA ("aging cleared", False, 0, 0, 0);

      --  the maximum elapsed time drops a fresh track at once
      Issue_Standard_RA;
      Cyc (Own_Normal, 0, Elapsed => 3_600_000);
      Check_Out ("aging maximum", 4, Rs.Clearing, Climb_Adv, 0,
                 Ty.Other_Traffic, Dropped => 1);
      Check ("aging maximum: no tracks", Tr.Count (C.Table) = 0);
   end Test_Aging;

   --  ------------------------------------------------------------------
   --  Group: full table
   --  ------------------------------------------------------------------

   procedure Test_Full_Table is
   begin
      --  60 different aircraft fill the table. Every one is at 100 000 ft:
      --  other traffic, so the first slot is the most urgent (complete tie)
      Sc.Init (C);
      for I in 1 .. 60 loop
         Put_Reply (I, I, 100_000, 4_000);
      end loop;
      Cyc (Own_Normal, 60);
      Check_Out ("full 60", 4, Rs.No_Event, No_Adv, 1, Ty.Other_Traffic);
      Check ("full 60: table is full", Tr.Count (C.Table) = 60);

      --  five new aircraft find no slot
      for I in 1 .. 5 loop
         Put_Reply (I, 60 + I, 100_000, 4_000);
      end loop;
      Cyc (Own_Normal, 5, Elapsed => 10);
      Check_Out ("full 5 new", 4, Rs.No_Event, No_Adv, 1, Ty.Other_Traffic,
                 Rejected => 5);
      Check ("full 5 new: still 60 tracks", Tr.Count (C.Table) = 60);

      --  a known aircraft is still refreshed while a new one is rejected
      Put_Reply (1, 1, 100_000, 4_000);
      Put_Reply (2, 61, 100_000, 4_000);
      Cyc (Own_Normal, 2, Elapsed => 10);
      Check_Out ("full mixed", 4, Rs.No_Event, No_Adv, 1, Ty.Other_Traffic,
                 Rejected => 1);
      Check ("full mixed: still 60 tracks", Tr.Count (C.Table) = 60);
      Check ("full mixed: aircraft 1 refreshed (two updates)",
             Natural (C.Table (1).Updates) = 2);
      Check ("full mixed: aircraft 61 not in the table",
             Tr.Find (C.Table, 61) = Tr.No_Track);

      --  the largest number of replies in one cycle: 256 different
      --  aircraft, 60 fit and 196 are rejected
      Sc.Init (C);
      for I in 1 .. 256 loop
         Put_Reply (I, I, 100_000, 4_000);
      end loop;
      Cyc (Own_Normal, 256);
      Check_Out ("full 256", 4, Rs.No_Event, No_Adv, 1, Ty.Other_Traffic,
                 Rejected => 196);
      Check ("full 256: 60 tracks", Tr.Count (C.Table) = 60);
   end Test_Full_Table;

   --  ------------------------------------------------------------------
   --  Group: two aircraft
   --  ------------------------------------------------------------------

   procedure Test_Two_Aircraft is
   begin
      Sc.Init (C);

      --  cycle 1: aircraft 1 at 3 300 ft, aircraft 2 at 2 400 ft; one
      --  update each, both proximate, the nearer one (2) is selected
      Put_Reply (1, 1, 3_300, 4_000);
      Put_Reply (2, 2, 2_400, 4_000);
      Cyc (Own_Normal, 2);
      Check_Out ("two 1", 4, Rs.No_Event, No_Adv, 2, Ty.Proximate_Traffic);
      Check ("two 1: two tracks", Tr.Count (C.Table) = 2);

      --  cycle 2: both become RA threats (rates -300 and -400 ft/s, tau 10
      --  and 5). The smaller tau wins: the RA starts on aircraft 2
      Put_Reply (1, 1, 3_000, 4_000);
      Put_Reply (2, 2, 2_000, 4_000);
      Cyc (Own_Normal, 2);
      Check_Out ("two 2", 4, Rs.Issued, Climb_Adv, 2,
                 Ty.Resolution_Advisory);
      Check_RA ("two 2", True, 2, 1, 0);

      --  cycle 3: aircraft 1 now has tau 0 (more urgent), but the RA keeps
      --  following aircraft 2
      Put_Reply (1, 1, 700, 4_000);
      Put_Reply (2, 2, 1_600, 4_000);
      Cyc (Own_Normal, 2);
      Check_Out ("two 3", 4, Rs.Maintained, Climb_Adv, 2,
                 Ty.Resolution_Advisory);
      Check_RA ("two 3", True, 2, 2, 0);

      --  cycle 4: aircraft 2 moves away, first quiet cycle
      Put_Reply (1, 1, 400, 4_000);
      Put_Reply (2, 2, 150_000, 4_000);
      Cyc (Own_Normal, 2);
      Check_Out ("two 4", 4, Rs.Clearing, Climb_Adv, 2, Ty.Other_Traffic);
      Check_RA ("two 4", True, 2, 3, 1);

      --  cycle 5: second quiet cycle, the RA ends. Aircraft 1 does not
      --  take over in this same cycle
      Put_Reply (1, 1, 100, 4_000);
      Put_Reply (2, 2, 150_000, 4_000);
      Cyc (Own_Normal, 2);
      Check_Out ("two 5", 4, Rs.Cleared, No_Adv, 2, Ty.Other_Traffic);
      Check_RA ("two 5", False, 0, 0, 0);

      --  cycle 6: now aircraft 1 (still an RA threat, inside the DMOD
      --  distance) starts a new RA
      Put_Reply (1, 1, 100, 4_000);
      Cyc (Own_Normal, 1);
      Check_Out ("two 6", 4, Rs.Issued, Climb_Adv, 1,
                 Ty.Resolution_Advisory);
      Check_RA ("two 6", True, 1, 1, 0);
   end Test_Two_Aircraft;

   --  ------------------------------------------------------------------
   --  Group: sweep of 4-cycle scripts (rule T4)
   --  ------------------------------------------------------------------

   --  The six reply sets a script can use in each cycle:
   --    1 no replies                      4 aircraft 2 at   2 000 ft
   --    2 aircraft 1 at   3 000 ft        5 aircraft 1 at 3 000 and
   --    3 aircraft 1 at 150 000 ft                aircraft 2 at 2 000
   --    6 aircraft 1 at 3 000 ft, 4 300 ft altitude (300 ft above)
   procedure Load (Choice : Positive; N : out Natural) is
   begin
      case Choice is
         when 1 =>
            N := 0;
         when 2 =>
            Put_Reply (1, 1, 3_000, 4_000);
            N := 1;
         when 3 =>
            Put_Reply (1, 1, 150_000, 4_000);
            N := 1;
         when 4 =>
            Put_Reply (1, 2, 2_000, 4_000);
            N := 1;
         when 5 =>
            Put_Reply (1, 1, 3_000, 4_000);
            Put_Reply (2, 2, 2_000, 4_000);
            N := 2;
         when others =>
            Put_Reply (1, 1, 3_000, 4_300);
            N := 1;
      end case;
   end Load;

   --  The contract of the specification, plus extra properties, for ONE
   --  cycle that has just run. Old_Active and Old_Addr are the RA state
   --  before the cycle; N is the number of replies given.
   function Cycle_Ok
     (Old_Active : Boolean;
      Old_Addr   : Ty.Mode_S_Address;
      Own        : Sc.Ownship_Data;
      N          : Natural) return Boolean
   is
      Own_Alt  : constant Ty.Altitude_Ft := Own.Pressure.Value;
      Own_Rate : constant Ty.Vertical_Rate_Fpm :=
        (if Own.Rate_Valid then Own.Rate else 0);
      Sel      : constant Natural := O.Selected;
   begin
      return
        Rs.Is_Consistent (C.RA)
        --  the level is never standby; an untrusted pressure altitude
        --  gives the fail-safe level
        and then Natural (O.Level) >= 2
        and then (if not Own.Pressure.Valid then Natural (O.Level) = 2)
        --  the reported advisory is the one in the state
        and then O.Adv = C.RA.Adv
        --  counters inside their bounds
        and then O.Dropped <= 60
        and then O.Rejected <= N
        and then Tr.Count (C.Table) <= 60
        --  the selected track, if any, is an active one
        and then Sel <= 60
        and then
          (if Sel /= 0
           then C.Table (Ty.Track_Index (Sel)).State = Tr.Active)
        --  an RA starts only at level 3 or more
        and then (if O.Event = Rs.Issued then Natural (O.Level) >= 3)
        --  which events can follow which state
        and then
          (if Old_Active
           then O.Event in Rs.Maintained | Rs.Reversed
                           | Rs.Clearing | Rs.Cleared
           else O.Event in Rs.Issued | Rs.No_Event)
        and then
          (if O.Event in Rs.Cleared | Rs.No_Event then not C.RA.Active)
        and then
          (if O.Event in Rs.Issued | Rs.Maintained | Rs.Reversed
                         | Rs.Clearing
           then C.RA.Active)
        --  the followed aircraft stays the same while the RA lasts
        and then
          (if Old_Active and then C.RA.Active
           then Natural (C.RA.Address) = Natural (Old_Addr))
        --  rule R5: an active RA follows its own aircraft, otherwise the
        --  most urgent track is chosen
        and then
          (if Old_Active
           then Sel = Tr.Find (C.Table, Old_Addr)
           else Sel = Sc.Most_Urgent (C.Table, Own_Alt, Own_Rate, O.Level))
        --  the class agrees with the event
        and then
          (if Old_Active
           then (O.Event in Rs.Maintained | Rs.Reversed)
                = (O.Selected_Class = Ty.Resolution_Advisory)
           else (O.Event = Rs.Issued)
                = (O.Selected_Class = Ty.Resolution_Advisory));
   end Cycle_Ok;

   type Script is array (1 .. 4) of Positive;

   --  Runs one script from Init. Pattern 1 has 1 000 ms in every cycle,
   --  pattern 2 has 6 000 ms in cycle 3 (so a track can age out).
   function Run_Script
     (Items   : Script;
      Own     : Sc.Ownship_Data;
      Pattern : Positive) return Boolean
   is
      Ok         : Boolean := True;
      N          : Natural := 0;
      Elapsed    : Natural := 1_000;
      Old_Active : Boolean;
      Old_Addr   : Ty.Mode_S_Address;
   begin
      Sc.Init (C);

      for K in Items'Range loop
         Load (Items (K), N);
         Elapsed := (if Pattern = 2 and then K = 3 then 6_000 else 1_000);

         Old_Active := C.RA.Active;
         Old_Addr   := C.RA.Address;

         Sc.Run_Cycle (C, Own, Buf (1 .. N), Ty.Time_Ms (Elapsed), O);

         Ok := Ok and then Cycle_Ok (Old_Active, Old_Addr, Own, N);
      end loop;

      return Ok;
   end Run_Script;

   procedure Sweep is
      Owns : constant array (1 .. 3) of Sc.Ownship_Data :=
        (1 => Own_Normal,
         2 => Own_Bad_Pressure,
         3 => Own_Rate_Invalid);

      Combos : Natural := 0;
      Bad    : Natural := 0;
   begin
      for P in 1 .. 2 loop
         for Oi in Owns'Range loop
            for I1 in 1 .. 6 loop
               for I2 in 1 .. 6 loop
                  for I3 in 1 .. 6 loop
                     for I4 in 1 .. 6 loop
                        Combos := Combos + 1;

                        if not Run_Script
                                 ((I1, I2, I3, I4), Owns (Oi), P)
                        then
                           if Bad = 0 then
                              Ada.Text_IO.Put_Line
                                ("sweep first mismatch: pattern"
                                 & Integer'Image (P)
                                 & " own" & Integer'Image (Oi)
                                 & " script" & Integer'Image (I1)
                                 & Integer'Image (I2)
                                 & Integer'Image (I3)
                                 & Integer'Image (I4));
                           end if;
                           Bad := Bad + 1;
                        end if;
                     end loop;
                  end loop;
               end loop;
            end loop;
         end loop;
      end loop;

      Check ("sweep: all" & Natural'Image (Combos)
             & " scripts (mismatches" & Natural'Image (Bad) & ")", 
             Combos = 7_776 and then Bad = 0);
   end Sweep;

begin
   Test_Most_Urgent;
   Test_Init;
   Test_Life_Cycle;
   Test_Fail_Safe;
   Test_Own_Rate;
   Test_Aging;
   Test_Full_Table;
   Test_Two_Aircraft;
   Sweep;

   Ada.Text_IO.Put_Line
     ("test_scheduler:" & Natural'Image (Total) & " checks,"
      & Natural'Image (Failed) & " failed");

   if Failed > 0 then
      Ada.Text_IO.Put_Line ("FAIL");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   else
      Ada.Text_IO.Put_Line ("OK");
   end if;
end Test_Scheduler;