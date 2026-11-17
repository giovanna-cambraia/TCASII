--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/scheduler/tcas-scheduler.adb
--  Module      : One surveillance and threat cycle (body)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Implements Most_Urgent, Init and Run_Cycle declared in
--    tcas-scheduler.ads. The specification holds the contract (rules,
--    order of the five steps, order of preference); this body holds the
--    code that makes it true. Read the specification first.
--
--  DESIGN RULES (as in the specification, with the way each is met here)
--    R1  No state: every subprogram works only on its parameters and on
--        the Cycle_State that the caller owns.
--    R2  Fixed order. Run_Cycle has the five steps in sequence. The only
--        loop that depends on data is the reply loop, bounded by the
--        length of Replies (at most Max_Replies_Per_Cycle, from the
--        precondition).
--    R3  Fail safe is delegated to Tcas.Sensitivity: an untrusted
--        pressure altitude gives SL 2 and the postcondition follows from
--        the postcondition of Select_Level. Own_Alt is read from the
--        pressure source whether it is valid or not (open item for v2).
--    R4  Own vertical rate is 0 unless Rate_Valid is True.
--    R5  While an RA is active the followed aircraft is found by its
--        address (Tr.Find). Only when no RA is active is Most_Urgent
--        used. A lost track becomes an empty slot, which the state
--        machine counts as a quiet cycle.
--    R6  Strict comparisons in More_Urgent, so on a complete tie the
--        track met FIRST (the lower index) stays the best.
--    R7  Counters are bounded by construction: Dropped from the
--        postcondition of Age_Tracks, Rejected by a loop invariant.
--
--  STRUCTURE OF THE CODE
--    Local helpers first (Empty_Track, More_Urgent), then Most_Urgent,
--    Init and Run_Cycle. Most_Urgent and Run_Cycle have no early exit:
--    Most_Urgent visits all 60 slots every time, so its cost is
--    constant. Run_Cycle assigns Output once, at the end.
--
--  PROOF OBLIGATIONS (discharged by GNATprove, tcas2.gpr level 2)
--    * Absence of runtime errors: every table index (Best, Idx) is in
--      1 .. 60 where it is used, the Rejected counter stays below the
--      number of replies, every call meets its precondition (Step needs
--      Is_Consistent, which the loop invariant keeps true).
--    * The loop invariants of Most_Urgent and of the reply loop.
--    * The postconditions of Most_Urgent, Init and Run_Cycle.
--
--  SPARK NOTES
--    * SPARK_Mode is On.
--    * The with and use clauses of the specification apply to this body,
--      so Tcas.Config, Tcas.RA, Tcas.RA_State, Tcas.Sensitivity,
--      Tcas.Tracker and Tcas.Types are visible without repeating them.
--    * No deviations are requested by this body.

pragma SPARK_Mode (On);

with Tcas.Threat;

package body Tcas.Scheduler is

   use type Tr.Update_Result;

   --  A slot with nothing in it. Passed to the RA state machine when the
   --  followed aircraft is no longer in the table.
   Empty_Track : constant Tr.Track :=
     (State         => Tr.Free,
      Address       => 0,
      Slant         => 0,
      Range_Rate    => 0,
      Bearing       => 0,
      Altitude      => 0,
      Vertical_Rate => 0,
      Age           => 0,
      Updates       => 0);

   --  Order of preference (local to this body)

   --  True when Item is STRICTLY more urgent than Best_Item (see ORDER OF
   --  PREFERENCE in the specification): higher class, then smaller tau,
   --  then smaller slant. A complete tie is False, so the earlier track
   --  stays the best (rule R6).
   function More_Urgent
     (Class      : Threat_Class;
      Item       : Tr.Track;
      Best_Class : Threat_Class;
      Best_Item  : Tr.Track) return Boolean
   with
     Global => null;

   function More_Urgent
     (Class      : Threat_Class;
      Item       : Tr.Track;
      Best_Class : Threat_Class;
      Best_Item  : Tr.Track) return Boolean
   is
      Tau      : constant Tau_S := Tcas.RA.Effective_Tau (Item);
      Best_Tau : constant Tau_S := Tcas.RA.Effective_Tau (Best_Item);
   begin
      return Class > Best_Class
        or else
          (Class = Best_Class
           and then
             (Tau < Best_Tau
              or else
                (Tau = Best_Tau and then Item.Slant < Best_Item.Slant)));
   end More_Urgent;

   --  Selection of the intruder

   function Most_Urgent
     (Table    : Tr.Track_Table;
      Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level) return Natural
   is
      Best       : Natural      := Tr.No_Track;
      Best_Class : Threat_Class := Other_Traffic;
   begin
      for I in Track_Index loop
         pragma Loop_Invariant (Best <= Max_Tracks);
         pragma Loop_Invariant
           (if Best /= Tr.No_Track
            then Table (Best).State = Tr.Active
            else
              (for all J in Track_Index range 1 .. I - 1 =>
                 Table (J).State = Tr.Free));

         if Table (I).State = Tr.Active then
            declare
               Class : constant Threat_Class :=
                 Tcas.Threat.Classify (Own_Alt, Own_Rate, Own_SL, Table (I));
            begin
               if Best = Tr.No_Track
                 or else More_Urgent
                           (Class, Table (I), Best_Class, Table (Best))
               then
                  Best       := I;
                  Best_Class := Class;
               end if;
            end;
         end if;
      end loop;

      return Best;
   end Most_Urgent;

   --  Init

   procedure Init (C : out Cycle_State) is
   begin
      Tr.Init (C.Table);
      Rs.Init (C.RA);
   end Init;

   --  One cycle

   procedure Run_Cycle
     (C       : in out Cycle_State;
      Own     : Ownship_Data;
      Replies : Measurement_Array;
      Elapsed : Time_Ms;
      Output  : out Cycle_Output)
   is
      Own_Alt  : constant Altitude_Ft := Own.Pressure.Value;
      Own_Rate : constant Vertical_Rate_Fpm :=
        (if Own.Rate_Valid then Own.Rate else 0);

      Dropped  : Natural;
      Rejected : Natural := 0;
   begin
      --  step 1: age the tracks and drop the stale ones
      Tr.Age_Tracks (C.Table, Elapsed, Dropped);

      --  step 2: feed every valid reply to the table; a reply that finds
      --  no free slot is counted and dropped (rule R7 of the spec)
      for I in Replies'Range loop
         pragma Loop_Invariant (Rejected <= I - Replies'First);
         pragma Loop_Invariant (Rs.Is_Consistent (C.RA));

         declare
            Result : Tr.Update_Result;
         begin
            Tr.Update (C.Table, Replies (I), Result);

            if Result = Tr.Table_Full then
               Rejected := Rejected + 1;
            end if;
         end;
      end loop;

      --  steps 3 to 5: level, intruder to follow, RA life cycle
      declare
         Level : constant Sensitivity_Level :=
           Tcas.Sensitivity.Select_Level (Own.Pressure, Own.Radio);

         --  rule R5: an active RA follows its own aircraft; otherwise the
         --  most urgent track is chosen
         Idx : constant Natural :=
           (if C.RA.Active
            then Tr.Find (C.Table, C.RA.Address)
            else Most_Urgent (C.Table, Own_Alt, Own_Rate, Level));

         --  a lost track becomes an empty slot (quiet cycle)
         Item : constant Tr.Track :=
           (if Idx /= Tr.No_Track then C.Table (Idx) else Empty_Track);

         Class : constant Threat_Class :=
           (if Idx /= Tr.No_Track
            then Tcas.Threat.Classify (Own_Alt, Own_Rate, Level, Item)
            else Other_Traffic);

         Event : Rs.RA_Event;
      begin
         Rs.Step (C.RA, Own_Alt, Own_Rate, Level, Item, Event);

         Output :=
           (Level          => Level,
            Event          => Event,
            Adv            => C.RA.Adv,
            Selected       => Idx,
            Selected_Class => Class,
            Dropped        => Dropped,
            Rejected       => Rejected);
      end;
   end Run_Cycle;

end Tcas.Scheduler;