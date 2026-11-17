--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/scheduler/tcas-scheduler.ads
--  Module      : One surveillance and threat cycle (specification)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Runs ONE cycle of the core logic, in the order of the tracker
--    specification and of the architecture document (section 4C):
--      1 age the tracks and drop the stale ones      Tcas.Tracker
--      2 feed every valid Mode S reply to the table  Tcas.Tracker
--      3 choose the sensitivity level                Tcas.Sensitivity
--      4 choose the intruder to follow               Most_Urgent
--      5 advance the RA life cycle                   Tcas.RA_State
--    The caller (the cyclic executive, once per 1 000 ms) provides the
--    decoded ownship data and the valid replies of the cycle, and gets
--    back one Cycle_Output with everything the display layer needs.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  No package state. Everything the cycle remembers is in
--        Cycle_State, which the CALLER owns, as in Tcas.Tracker and
--        Tcas.RA_State. Every subprogram has Global => null.
--    R2  Fixed order. The steps above always run in that order, with no
--        loop that depends on data except the reply loop, bounded by
--        Max_Replies_Per_Cycle (ASM-004).
--    R3  Fail safe. An untrusted pressure altitude gives SL 2
--        (Tcas.Sensitivity), so no new RA can start from it; an active RA
--        then clears after Tcas.Config.RA_Clear_Cycles quiet cycles.
--        Traffic advisories can still be computed from the untrusted
--        value (open item for v2).
--    R4  Missing own vertical rate is treated as level flight (rate 0).
--    R5  One threat at a time. While an RA is active the followed
--        aircraft is the one in State.Address (rule R2 of
--        Tcas.RA_State). Only when no RA is active is a new aircraft
--        chosen by Most_Urgent.
--    R6  A lost track is a quiet cycle. When the followed aircraft is no
--        longer in the table, the cycle passes an empty slot to the RA
--        state machine, which counts it as quiet.
--    R7  A full table is not an error. Replies that find no free slot
--        are counted in Output.Rejected and dropped. The policy "nearest
--        range first" of the architecture document is NOT implemented
--        (open item for v2).
--
--  ORDER OF PREFERENCE IN Most_Urgent (the first difference wins)
--    1 higher threat class (RA, then TA, then proximate, then other)
--    2 smaller effective tau (Tcas.RA.Effective_Tau)
--    3 smaller slant range
--    4 lower table index
--  Free slots are never chosen. When every slot is free the result is
--  Tcas.Tracker.No_Track.
--
--  CALLING ORDER (the caller is the cyclic executive)
--    Init (C)                                  once, at start-up
--    Run_Cycle (C, Own, Replies, Elapsed, Out) once per 1 000 ms cycle
--
--  TIMING NOTE
--    The reply loop dominates: up to 256 replies, each one a search of
--    the 60-slot table. The worst case is a fixed number of comparisons,
--    so the WCET analysis can bound it (RISK-001 of the architecture
--    document).
--
--  KNOWN SIMPLIFICATIONS (open items for v2)
--    No "nearest range first" policy when the table is full; traffic
--    advisories computed from an untrusted pressure altitude; own
--    vertical rate treated as 0 when invalid; one threat only; no
--    coordination with the intruder.
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * The body is in tcas-scheduler.adb; GNATprove proves the
--      postconditions and the absence of runtime errors there.
--    * No deviations are requested by this specification.

pragma SPARK_Mode (On);

with Tcas.Config;
with Tcas.RA;
with Tcas.RA_State;
with Tcas.Sensitivity;
with Tcas.Tracker;
with Tcas.Types; use Tcas.Types;

package Tcas.Scheduler is

   package Tr renames Tcas.Tracker;
   package Rs renames Tcas.RA_State;

   use type Tr.Track_State;
   use type Tcas.RA.Advisory;

   --  Inputs

   --  Decoded ownship data. The two altitudes come from
   --  Tcas.A429_Decoder (Valid is True when the status is Decoded).
   type Ownship_Data is record
      Pressure   : Tcas.Sensitivity.Altitude_Source;
      Radio      : Tcas.Sensitivity.Altitude_Source;
      Rate_Valid : Boolean           := False;
      Rate       : Vertical_Rate_Fpm := 0;
   end record;

   --  The valid Mode S replies of this cycle, already parsed. The caller
   --  never passes more than Max_Replies_Per_Cycle.
   type Measurement_Array is array (Positive range <>) of Tr.Measurement;

   --  State and output

   --  Everything the cycle remembers. Call Init before the first cycle: a
   --  zero-filled value is NOT assumed valid.
   type Cycle_State is record
      Table : Tr.Track_Table;
      RA    : Rs.State;
   end record;

   --  What one cycle reports.
   type Cycle_Output is record
      Level          : Sensitivity_Level := SL_Fail_Safe;
      Event          : Rs.RA_Event       := Rs.No_Event;
      Adv            : Tcas.RA.Advisory  :=
        (Kind => Tcas.RA.No_Advisory, Sense => No_Sense, Target_Rate => 0);
      Selected       : Natural           := Tr.No_Track;
      Selected_Class : Threat_Class      := Other_Traffic;
      Dropped        : Natural           := 0;   --  tracks aged out
      Rejected       : Natural           := 0;   --  replies with no slot
   end record;

   --  Selection of the intruder (public so that the tests can check it)

   --  Index of the most urgent active track, or No_Track when every slot
   --  is free (see ORDER OF PREFERENCE).
   function Most_Urgent
     (Table    : Tr.Track_Table;
      Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level) return Natural
   with
     Global => null,
     Post   =>
       Most_Urgent'Result <= Max_Tracks
       and then
         (if Most_Urgent'Result /= Tr.No_Track
          then Table (Most_Urgent'Result).State = Tr.Active
          else (for all I in Track_Index => Table (I).State = Tr.Free));

   --  Operations

   --  Empty table, no advisory. Call before the first cycle.
   procedure Init (C : out Cycle_State)
   with
     Global => null,
     Post   =>
       Tr.Count (C.Table) = 0
       and then not C.RA.Active
       and then Rs.Is_Consistent (C.RA);

   --  One cycle (see PURPOSE for the five steps).
   --    Own     : decoded ownship data of this cycle
   --    Replies : the valid replies of this cycle
   --    Elapsed : time since the previous cycle, in milliseconds
   procedure Run_Cycle
     (C       : in out Cycle_State;
      Own     : Ownship_Data;
      Replies : Measurement_Array;
      Elapsed : Time_Ms;
      Output  : out Cycle_Output)
   with
     Global => null,
     Pre    =>
       Rs.Is_Consistent (C.RA)
       and then Replies'Length <= Tcas.Config.Max_Replies_Per_Cycle,
     Post   =>
       Rs.Is_Consistent (C.RA)
       --  the level is never standby, and an untrusted pressure altitude
       --  gives the fail-safe level (rule R3)
       and then Output.Level >= SL_Fail_Safe
       and then
         (if not Own.Pressure.Valid then Output.Level = SL_Fail_Safe)
       --  the reported advisory is the one in the state
       and then Output.Adv = C.RA.Adv
       --  counters stay inside their bounds
       and then Output.Dropped <= Max_Tracks
       and then Output.Rejected <= Replies'Length
       --  the selected track, if any, is an active one
       and then Output.Selected <= Max_Tracks
       and then
         (if Output.Selected /= Tr.No_Track
          then C.Table (Output.Selected).State = Tr.Active);

end Tcas.Scheduler;