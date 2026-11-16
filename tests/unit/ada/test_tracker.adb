--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : tests/unit/ada/test_tracker.adb
--  Module      : Unit tests of the intruder track table (plain Ada program)
--  Language    : Ada 2012 (the tests are not in the SPARK subset)
--  Assurance   : DO-178C Software Level B (target), verification artifact
--  Tests       : src/ada/tracker/tcas-tracker.ads and .adb
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Checks every behavior promised in tcas-tracker.ads: how a track is
--    created and refreshed, how the rates are derived, what happens when
--    the table is full, and when a track is dropped for age. GNATprove
--    proves the absence of runtime errors and the postconditions; these
--    tests check that the numbers are the right ones.
--
--  DESIGN RULES OF THE TESTS
--    T1  Expected rates are computed by the test with its OWN arithmetic
--        (Long_Long_Integer, a plain divide, a plain clamp). The tracker's
--        functions are never used to compute an expectation.
--    T2  The truncation cases are chosen so that a floor division would
--        give a DIFFERENT answer. Ada "/" truncates toward zero; a change
--        to any other rounding fails a test.
--    T3  Limits are written as literals taken from the specification
--        (3 000 ft/s, 10 000 ft/min, 5 000 ms, 60 tracks, 255 hits), not
--        read from the code under test.
--    T4  Every test starts with Init, so tests do not depend on each other
--        or on their order.
--    T5  Output: one line per failed check, then a summary. The exit
--        status is 1 if anything failed, else 0, so a script can use it.
--
--  TESTS BY GROUP
--    init       : empty table, dirty table, free slot with address 0
--    insert     : every field of a new track, two aircraft, slot order
--    rates      : known values, age zero, truncation, saturation
--    hits       : the hit counter stops at 255
--    full table : 60 inserts, Table_Full, refresh while full, recovery
--    aging      : boundary at 5 000 ms, maximum elapsed, slot reuse
--    sweep      : 5 400 combinations of position change and elapsed time
--
--  USAGE (from the project root; no project file is needed)
--    mkdir -p build/obj_test build/bin
--    gnatmake tests/unit/ada/test_tracker.adb -gnat2012 -gnatwa \
--        -D build/obj_test -o build/bin/test_tracker.exe \
--        -Isrc/ada/common -Isrc/ada/common/types \
--        -Isrc/ada/common/config -Isrc/ada/common/units \
--        -Isrc/ada/tracker
--    build/bin/test_tracker.exe
--
--  NOTES
--    * tcas2.gpr does not list tests/, so the project build never sees
--      this program (it has a main procedure and is not part of the
--      product).
--    * The expected numbers in the rate tests are worked out by hand and
--      written next to each check. If one fails, recheck the arithmetic in
--      the test before blaming the tracker.
--  ===========================================================================

with Ada.Command_Line;
with Ada.Text_IO;

with Tcas.Tracker;
with Tcas.Types;

procedure Test_Tracker is

   package Tr renames Tcas.Tracker;
   package Ty renames Tcas.Types;

   use type Tr.Update_Result;
   use type Tr.Track_State;
   use type Tr.Track_Table;

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
   --  Fixtures and helpers
   --  ------------------------------------------------------------------

   Table   : Tr.Track_Table;
   Result  : Tr.Update_Result := Tr.Inserted;
   Dropped : Natural := 0;

   function Meas
     (Addr    : Natural;
      Slant   : Natural;
      Bearing : Natural;
      Alt     : Integer) return Tr.Measurement is
     (Address  => Ty.Mode_S_Address (Addr),
      Slant    => Ty.Range_Ft (Slant),
      Bearing  => Ty.Bearing_Cdeg (Bearing),
      Altitude => Ty.Altitude_Ft (Alt));

   --  Feeds one measurement; the result is left in Result.
   procedure Feed
     (Addr    : Natural;
      Slant   : Natural;
      Bearing : Natural;
      Alt     : Integer) is
   begin
      Tr.Update (Table, Meas (Addr, Slant, Bearing, Alt), Result);
   end Feed;

   --  Advances the clock; the number of dropped tracks is left in Dropped.
   procedure Age (Ms : Natural) is
   begin
      Tr.Age_Tracks (Table, Ty.Time_Ms (Ms), Dropped);
   end Age;

   function Slot (Addr : Natural) return Natural is
     (Tr.Find (Table, Ty.Mode_S_Address (Addr)));

   --  Checks every field of the track with this address (rule T3: the
   --  expected values are literals chosen by the caller).
   procedure Check_Track
     (Name    : String;
      Addr    : Natural;
      Slant   : Integer;
      Bearing : Integer;
      Alt     : Integer;
      RRate   : Integer;
      VRate   : Integer;
      Age_Ms  : Integer;
      Hits    : Integer)
   is
      I : constant Natural := Slot (Addr);
   begin
      Check (Name & ": found", I /= Tr.No_Track);

      if I /= Tr.No_Track then
         declare
            T : Tr.Track renames Table (Ty.Track_Index (I));
         begin
            Check (Name & ": active",    T.State = Tr.Active);
            Check (Name & ": address",   Natural (T.Address) = Addr);
            Check (Name & ": slant",     Integer (T.Slant) = Slant);
            Check (Name & ": bearing",   Integer (T.Bearing) = Bearing);
            Check (Name & ": altitude",  Integer (T.Altitude) = Alt);
            Check (Name & ": range rate",
                   Integer (T.Range_Rate) = RRate);
            Check (Name & ": vertical rate",
                   Integer (T.Vertical_Rate) = VRate);
            Check (Name & ": age",       Integer (T.Age) = Age_Ms);
            Check (Name & ": updates",   Integer (T.Updates) = Hits);
         end;
      end if;
   end Check_Track;

   --  ------------------------------------------------------------------
   --  Group: init
   --  ------------------------------------------------------------------

   procedure Test_Init is
   begin
      Table := (others =>
                  (State         => Tr.Active,
                   Address       => 9,
                   Slant         => 1,
                   Range_Rate    => 1,
                   Bearing       => 1,
                   Altitude      => 1,
                   Vertical_Rate => 1,
                   Age           => 1,
                   Updates       => 1));
      Check ("init: dirty table has 60 tracks", Tr.Count (Table) = 60);

      Tr.Init (Table);
      Check ("init: count is 0", Tr.Count (Table) = 0);
      Check ("init: every slot is free",
             (for all I in Ty.Track_Index => Table (I).State = Tr.Free));
      Check ("init: dirty address is gone", Slot (9) = Tr.No_Track);
      Check ("init: free slot is not found by address 0",
             Slot (0) = Tr.No_Track);
   end Test_Init;

   --  ------------------------------------------------------------------
   --  Group: insert
   --  ------------------------------------------------------------------

   procedure Test_Insert is
   begin
      Tr.Init (Table);

      Feed (16#ABCDEF#, 30_000, 9_000, 10_000);
      Check ("insert: result is Inserted", Result = Tr.Inserted);
      Check ("insert: count is 1", Tr.Count (Table) = 1);
      Check_Track ("insert", 16#ABCDEF#, 30_000, 9_000, 10_000, 0, 0, 0, 1);

      --  the first track takes the first slot, the second one the next
      Check ("insert: first track is in slot 1",
             Slot (16#ABCDEF#) = 1);

      Feed (2, 5_000, 35_999, -1_200);
      Check ("insert: second aircraft is Inserted", Result = Tr.Inserted);
      Check ("insert: count is 2", Tr.Count (Table) = 2);
      Check ("insert: second track is in slot 2", Slot (2) = 2);
      Check_Track ("insert second", 2, 5_000, 35_999, -1_200, 0, 0, 0, 1);
      Check_Track ("insert: first track untouched",
                   16#ABCDEF#, 30_000, 9_000, 10_000, 0, 0, 0, 1);
   end Test_Insert;

   --  ------------------------------------------------------------------
   --  Group: rates
   --  ------------------------------------------------------------------

   procedure Test_Rates is
   begin
      --  known values
      --    range    : (29 000 - 30 000) * 1 000 / 1 000 ms = -1 000 ft/s
      --    vertical : (10 100 - 10 000) * 60 000 / 1 000 ms = 6 000 ft/min
      Tr.Init (Table);
      Feed (100, 30_000, 9_000, 10_000);
      Age (1_000);
      Check_Track ("aged", 100, 30_000, 9_000, 10_000, 0, 0, 1_000, 1);

      Feed (100, 29_000, 9_100, 10_100);
      Check ("rates: result is Updated", Result = Tr.Updated);
      Check ("rates: count is still 1", Tr.Count (Table) = 1);
      Check_Track ("rates climb", 100, 29_000, 9_100, 10_100,
                   -1_000, 6_000, 0, 2);

      --  a descent: vertical = (10 000 - 10 100) * 60 000 / 1 000 = -6 000
      Age (1_000);
      Feed (100, 29_000, 9_100, 10_000);
      Check_Track ("rates descent", 100, 29_000, 9_100, 10_000,
                   0, -6_000, 0, 3);

      --  Age 0: two updates with no time between them keep the old rates
      Tr.Init (Table);
      Feed (9, 30_000, 0, 10_000);
      Age (1_000);
      Feed (9, 29_000, 0, 10_100);
      Check_Track ("age zero, before", 9, 29_000, 0, 10_100,
                   -1_000, 6_000, 0, 2);
      Feed (9, 20_000, 5, 0);
      Check_Track ("age zero, after", 9, 20_000, 5, 0,
                   -1_000, 6_000, 0, 3);

      --  truncation toward zero (rule T2)
      --    range : -1 000 000 / 3 000 = -333.33 -> -333 (floor: -334)
      Tr.Init (Table);
      Feed (5, 30_000, 0, 10_000);
      Age (3_000);
      Feed (5, 29_000, 0, 10_000);
      Check_Track ("truncation range", 5, 29_000, 0, 10_000,
                   -333, 0, 0, 2);

      --    range    : -1 * 1 000 / 4 999 = -0.2 -> 0 (floor: -1)
      --    vertical : 1 * 60 000 / 4 999 = 12.002 -> 12
      Tr.Init (Table);
      Feed (6, 30_000, 0, 10_000);
      Age (4_999);
      Feed (6, 29_999, 0, 10_001);
      Check_Track ("truncation small positive", 6, 29_999, 0, 10_001,
                   0, 12, 0, 2);

      --    range    : +1 * 1_000 / 4_999 = 0.2 -> 0
      --    vertical : -1 * 60_000 / 4_999 = -12.002 -> -12 (floor: -13)
      Age (4_999);
      Feed (6, 30_000, 0, 10_000);
      Check_Track ("truncation small negative", 6, 30_000, 0, 10_000,
                   0, -12, 0, 3);

      --  exact limit: 3 * 1_000 / 1 ms = 3 000 ft/s, not clamped
      Tr.Init (Table);
      Feed (8, 100, 0, 0);
      Age (1);
      Feed (8, 103, 0, 0);
      Check_Track ("range rate at the limit", 8, 103, 0, 0,
                   3_000, 0, 0, 2);

      --  saturation: the changes below give 1.5E8 and 7.7E9 before the
      --  limit, so both rates clamp
      Tr.Init (Table);
      Feed (7, 150_000, 0, -1_200);
      Age (1);
      Feed (7, 0, 0, 126_700);
      Check_Track ("saturate negative range, positive climb",
                   7, 0, 0, 126_700, -3_000, 10_000, 0, 2);

      Age (1);
      Feed (7, 150_000, 0, -1_200);
      Check_Track ("saturate positive range, negative descent",
                   7, 150_000, 0, -1_200, 3_000, -10_000, 0, 3);
   end Test_Rates;

   --  ------------------------------------------------------------------
   --  Group: hits
   --  ------------------------------------------------------------------

   procedure Test_Hits is
   begin
      Tr.Init (Table);

      for K in 1 .. 300 loop
         Feed (10, 1_000, 0, 0);
      end loop;

      Check_Track ("hits stop at 255", 10, 1_000, 0, 0, 0, 0, 0, 255);
   end Test_Hits;

   --  ------------------------------------------------------------------
   --  Group: full table
   --  ------------------------------------------------------------------

   procedure Test_Full_Table is
      All_Inserted : Boolean := True;
   begin
      Tr.Init (Table);

      for A in 1 .. 60 loop
         Feed (A, 1_000 + A, 0, 0);
         All_Inserted := All_Inserted and then Result = Tr.Inserted;
      end loop;

      Check ("full: 60 inserts all succeed", All_Inserted);
      Check ("full: count is 60", Tr.Count (Table) = 60);

      declare
         Before : constant Tr.Track_Table := Table;
      begin
         Feed (61, 2_000, 0, 0);
         Check ("full: 61st aircraft is Table_Full",
                Result = Tr.Table_Full);
         Check ("full: table is unchanged", Table = Before);
         Check ("full: 61st aircraft is not found",
                Slot (61) = Tr.No_Track);
         Check ("full: count is still 60", Tr.Count (Table) = 60);
      end;

      --  a known aircraft can still be refreshed when the table is full
      Feed (30, 5_000, 0, 0);
      Check ("full: refresh of a known aircraft is Updated",
             Result = Tr.Updated);
      Check ("full: count is still 60 after refresh",
             Tr.Count (Table) = 60);

      --  recovery: everything ages out, then a new aircraft fits
      Age (5_001);
      Check ("full: all 60 dropped", Dropped = 60);
      Check ("full: count is 0 after aging", Tr.Count (Table) = 0);

      Feed (61, 2_000, 0, 0);
      Check ("full: recovery insert is Inserted", Result = Tr.Inserted);
      Check ("full: recovery count is 1", Tr.Count (Table) = 1);
   end Test_Full_Table;

   --  ------------------------------------------------------------------
   --  Group: aging
   --  ------------------------------------------------------------------

   procedure Test_Aging is
   begin
      --  an empty table stays empty, nothing is created
      Tr.Init (Table);
      Age (1_000);
      Check ("aging: empty table drops nothing", Dropped = 0);
      Check ("aging: empty table stays empty", Tr.Count (Table) = 0);

      --  the clock adds up
      Tr.Init (Table);
      Feed (1, 1_000, 0, 0);
      Age (1_200);
      Check_Track ("aging 1200", 1, 1_000, 0, 0, 0, 0, 1_200, 1);
      Age (300);
      Check_Track ("aging 1500", 1, 1_000, 0, 0, 0, 0, 1_500, 1);

      --  boundary: 5 000 ms stays, 5 001 ms is dropped
      Tr.Init (Table);
      Feed (1, 1_000, 0, 0);
      Age (5_000);
      Check ("aging: 5000 ms is kept", Dropped = 0);
      Check ("aging: 5000 ms, count is 1", Tr.Count (Table) = 1);
      Age (1);
      Check ("aging: 5001 ms is dropped", Dropped = 1);
      Check ("aging: 5001 ms, count is 0", Tr.Count (Table) = 0);
      Check ("aging: dropped track is not found",
             Slot (1) = Tr.No_Track);
      Check ("aging: dropped slot is free", Table (1).State = Tr.Free);

      --  the largest elapsed time drops a fresh track at once
      Tr.Init (Table);
      Feed (1, 1_000, 0, 0);
      Age (3_600_000);
      Check ("aging: maximum elapsed drops the track", Dropped = 1);
      Check ("aging: maximum elapsed, count is 0", Tr.Count (Table) = 0);

      --  a refresh resets the clock of that track only
      Tr.Init (Table);
      Feed (1, 1_000, 0, 0);
      Feed (2, 2_000, 0, 0);
      Age (4_000);
      Feed (1, 1_100, 0, 0);
      Age (2_000);
      Check ("aging: only the stale track is dropped", Dropped = 1);
      Check ("aging: refreshed track survives", Slot (1) /= Tr.No_Track);
      Check ("aging: stale track is gone", Slot (2) = Tr.No_Track);
      --  range rate: (1 100 - 1 000) * 1 000 / 4 000 = 25 ft/s
      Check_Track ("aging: refreshed track keeps its age",
                   1, 1_100, 0, 0, 25, 0, 2_000, 2);

      --  a freed slot is reused by the next new aircraft
      Tr.Init (Table);
      Feed (1, 1_000, 0, 0);
      Feed (2, 2_000, 0, 0);
      Feed (3, 3_000, 0, 0);
      Check ("reuse: slots 1, 2, 3 in order",
             Slot (1) = 1 and then Slot (2) = 2 and then Slot (3) = 3);

      Age (3_000);
      Feed (1, 1_000, 0, 0);
      Feed (3, 3_000, 0, 0);
      Age (3_000);
      Check ("reuse: exactly one track dropped", Dropped = 1);
      Check ("reuse: count is 2", Tr.Count (Table) = 2);
      Check ("reuse: aircraft 2 is gone", Slot (2) = Tr.No_Track);

      Feed (4, 4_000, 0, 0);
      Check ("reuse: new aircraft is Inserted", Result = Tr.Inserted);
      Check ("reuse: new aircraft takes the freed slot 2", Slot (4) = 2);
      Check ("reuse: aircraft 1 stays in slot 1", Slot (1) = 1);
      Check ("reuse: aircraft 3 stays in slot 3", Slot (3) = 3);
   end Test_Aging;

   --  ------------------------------------------------------------------
   --  Group: sweep of position changes and elapsed times (rule T1)
   --  ------------------------------------------------------------------

   function Clamp
     (V   : Long_Long_Integer;
      Lim : Long_Long_Integer) return Long_Long_Integer is
     (if V < -Lim then -Lim
      elsif V > Lim then Lim
      else V);

   --  Runs every combination of old slant, new slant, old altitude, new
   --  altitude and elapsed time below, and compares both rates with an
   --  expectation built from a plain divide and a plain clamp. The elapsed
   --  times stay at or below 5 000 ms, so the track is never dropped.
   procedure Sweep_Rates is
      Ages   : constant array (1 .. 6) of Integer :=
        (1, 7, 999, 1_000, 4_999, 5_000);
      Slants : constant array (1 .. 5) of Integer :=
        (0, 1, 75_000, 149_999, 150_000);
      Alts   : constant array (1 .. 6) of Integer :=
        (-1_200, -1, 0, 1, 20_000, 126_700);

      Combos : Natural := 0;
      Bad    : Natural := 0;
   begin
      for A in Ages'Range loop
         for S0 in Slants'Range loop
            for S1 in Slants'Range loop
               for H0 in Alts'Range loop
                  for H1 in Alts'Range loop
                     declare
                        ER : constant Long_Long_Integer :=
                          Clamp
                            ((Long_Long_Integer (Slants (S1) - Slants (S0))
                              * 1_000) / Long_Long_Integer (Ages (A)),
                             3_000);
                        EV : constant Long_Long_Integer :=
                          Clamp
                            ((Long_Long_Integer (Alts (H1) - Alts (H0))
                              * 60_000) / Long_Long_Integer (Ages (A)),
                             10_000);
                     begin
                        Combos := Combos + 1;

                        Tr.Init (Table);
                        Feed (77, Slants (S0), 0, Alts (H0));
                        Age (Ages (A));
                        Feed (77, Slants (S1), 0, Alts (H1));

                        declare
                           I : constant Natural := Slot (77);
                           Good : Boolean := False;
                        begin
                           if I /= Tr.No_Track then
                              declare
                                 T : Tr.Track renames
                                   Table (Ty.Track_Index (I));
                              begin
                                 Good :=
                                   Long_Long_Integer (T.Range_Rate) = ER
                                   and then
                                   Long_Long_Integer (T.Vertical_Rate) = EV;
                              end;
                           end if;

                           if not Good then
                              if Bad = 0 then
                                 Ada.Text_IO.Put_Line
                                   ("sweep first mismatch: age"
                                    & Integer'Image (Ages (A))
                                    & " slant"
                                    & Integer'Image (Slants (S0))
                                    & " ->"
                                    & Integer'Image (Slants (S1))
                                    & " alt"
                                    & Integer'Image (Alts (H0))
                                    & " ->"
                                    & Integer'Image (Alts (H1)));
                              end if;
                              Bad := Bad + 1;
                           end if;
                        end;
                     end;
                  end loop;
               end loop;
            end loop;
         end loop;
      end loop;

      Check ("sweep: all" & Natural'Image (Combos)
             & " combinations (mismatches" & Natural'Image (Bad) & ")",
             Combos = 5_400 and then Bad = 0);
   end Sweep_Rates;

begin
   Test_Init;
   Test_Insert;
   Test_Rates;
   Test_Hits;
   Test_Full_Table;
   Test_Aging;
   Sweep_Rates;

   Ada.Text_IO.Put_Line
     ("test_tracker:" & Natural'Image (Total) & " checks,"
      & Natural'Image (Failed) & " failed");

   if Failed > 0 then
      Ada.Text_IO.Put_Line ("FAIL");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   else
      Ada.Text_IO.Put_Line ("OK");
   end if;
end Test_Tracker;