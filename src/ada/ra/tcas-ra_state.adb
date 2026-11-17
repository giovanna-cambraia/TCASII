--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/ra/tcas-ra_state.adb
--  Module      : Resolution advisory life cycle (body)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Implements Init and Step declared in tcas-ra_state.ads. The
--    specification holds the contract (rules, order of decisions, the
--    invariant Is_Consistent); this body holds the code that makes it
--    true. Read the specification first.
--
--  DESIGN RULES (as in the specification, with the way each is met here)
--    R1  No package state: Step works only on its parameters and on the
--        State value that the caller owns.
--    R2  The track is classified INSIDE Step (Tcas.Threat.Classify), so
--        the caller cannot pass a wrong classification. Classify promises
--        that an RA has an active track and Own_SL >= 3, which is exactly
--        what Tcas.RA.Generate requires.
--    R3  Sticky advisory: the advisory is assigned only in the Issued and
--        Reversed branches. Maintained and Clearing never touch it.
--    R4  One reversal per RA: Reversal_Due is False once State.Reversed
--        is set, and the Reversed branch sets it.
--    R5  Counters go through Next, which saturates at the top of the
--        range and never wraps.
--    R6  Every branch ends in a consistent state (Is_Consistent). The
--        clearing branch is taken before Clear_Run could reach the limit,
--        so the invariant "Clear_Run < RA_Clear_Cycles" holds.
--    R7  Constant cost: no loop. Every value used by the decisions is
--        computed before the chain, except the reversal test, which is
--        computed only where the state is known to be active (its
--        precondition needs a real sense).
--
--  STRUCTURE OF THE CODE
--    Local helpers first (Inactive, Opposite, Make_Advisory, Next,
--    Reversal_Due), then Init and Step. Step has ONE exit (it only
--    assigns its out parameter and its in out parameter) and a flat
--    if / elsif chain, one branch per decision, in the order of the
--    specification, so each condition is a separate, testable branch for
--    MC/DC coverage (DO-178C Table A-7).
--
--  PROOF OBLIGATIONS (discharged by GNATprove, tcas2.gpr level 2)
--    * Absence of runtime errors: every counter goes through Next, the
--      table index Own_SL is in the index range.
--    * Preconditions of the calls: Generate (from the postcondition of
--      Classify), Sense_Separation (from Is_Consistent on an active
--      state), Opposite and Make_Advisory (from the postcondition of
--      Opposite).
--    * The postconditions of Init and Step, including Is_Consistent on
--      every branch.
--
--  USAGE NOTE (for the scheduler)
--    While an RA is active, the caller must pass THE FOLLOWED aircraft
--    (found with Tcas.Tracker.Find on State.Address). If it passes a
--    different aircraft, or a free slot because the track was lost, the
--    cycle counts as quiet. After RA_Clear_Cycles such cycles the RA is
--    cleared, and a new one can start on the next cycle.
--
--  SPARK NOTES
--    * SPARK_Mode is On.
--    * The with and use clauses of the specification apply to this body,
--      so Tcas.Config, Tcas.RA, Tcas.Tracker and Tcas.Types are visible
--      without repeating them here.
--    * No deviations are requested by this body.

pragma SPARK_Mode (On);

with Tcas.Threat;

package body Tcas.RA_State is

   use type Tr.Track_State;

   --  Helpers (local to this body)

   --  The state with no advisory. Used by Init and by the Cleared branch.
   Inactive : constant State :=
     (Active    => False,
      Adv       => (Kind        => Tcas.RA.No_Advisory,
                    Sense       => No_Sense,
                    Target_Rate => 0),
      Address   => 0,
      Age       => 0,
      Clear_Run => 0,
      Reversed  => False);

   --  The other sense. Only meaningful for Upward and Downward.
   function Opposite (Sense : RA_Sense) return RA_Sense is
     (if Sense = Upward then Downward else Upward)
   with
     Global => null,
     Pre    => Sense /= No_Sense,
     Post   => Opposite'Result /= No_Sense
               and then Opposite'Result /= Sense;

   --  The advisory for a given sense: Upward is a Climb at +target rate,
   --  Downward is a Descend at -target rate. The same mapping as
   --  Tcas.RA.Generate, which always picks its own sense; a reversal has
   --  to force the opposite one.
   function Make_Advisory (Sense : RA_Sense) return Tcas.RA.Advisory
   with
     Global => null,
     Pre    => Sense /= No_Sense,
     Post   =>
       Make_Advisory'Result.Sense = Sense
       and then
         (if Sense = Upward
          then Make_Advisory'Result.Kind = Tcas.RA.Climb
               and then Make_Advisory'Result.Target_Rate
                          = Tcas.Config.RA_Target_Rate_Fpm
          else Make_Advisory'Result.Kind = Tcas.RA.Descend
               and then Make_Advisory'Result.Target_Rate
                          = -Tcas.Config.RA_Target_Rate_Fpm);

   function Make_Advisory (Sense : RA_Sense) return Tcas.RA.Advisory is
      Result : Tcas.RA.Advisory;
   begin
      if Sense = Upward then
         Result :=
           (Kind        => Tcas.RA.Climb,
            Sense       => Upward,
            Target_Rate => Tcas.Config.RA_Target_Rate_Fpm);
      else
         Result :=
           (Kind        => Tcas.RA.Descend,
            Sense       => Downward,
            Target_Rate => -Tcas.Config.RA_Target_Rate_Fpm);
      end if;

      return Result;
   end Make_Advisory;

   --  A counter plus one, stopping at the top of the range (rule R5).
   function Next (C : Cycle_Count) return Cycle_Count is
     (if C < Cycle_Count'Last then C + 1 else C)
   with Global => null;

   --  True when the current sense no longer reaches the separation goal
   --  (ALIM) and the opposite sense does, and no reversal was used yet.
   --  The caller guarantees a real sense (the state is active).
   function Reversal_Due
     (S        : State;
      Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track) return Boolean
   with
     Global => null,
     Pre    => S.Adv.Sense /= No_Sense,
     Post   => (if Reversal_Due'Result then not S.Reversed);

   function Reversal_Due
     (S        : State;
      Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track) return Boolean
   is
      ALIM : constant Altitude_Diff_Ft :=
        Tcas.Config.Thresholds (Own_SL).RA_ALIM;

      Current : constant Altitude_Diff_Ft :=
        Tcas.RA.Sense_Separation (Own_Alt, Own_Rate, S.Adv.Sense, Item);

      Other : constant Altitude_Diff_Ft :=
        Tcas.RA.Sense_Separation
          (Own_Alt, Own_Rate, Opposite (S.Adv.Sense), Item);
   begin
      return not S.Reversed and then Current < ALIM and then Other >= ALIM;
   end Reversal_Due;

   --  Init

   procedure Init (S : out State) is
   begin
      S := Inactive;
   end Init;

   --  Step

   procedure Step
     (S        : in out State;
      Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track;
      Event    : out RA_Event)
   is
      --  rule R2: the track is classified here, not by the caller
      Class : constant Threat_Class :=
        Tcas.Threat.Classify (Own_Alt, Own_Rate, Own_SL, Item);

      Is_RA : constant Boolean := Class = Resolution_Advisory;

      --  the followed aircraft: an active track with the stored address
      Same : constant Boolean :=
        Item.State = Tr.Active and then Item.Address = S.Address;

      --  quiet cycles counted so far plus this one (rule R5)
      New_Run : constant Cycle_Count := Next (S.Clear_Run);
   begin
      Event := No_Event;

      if not S.Active and then Is_RA then
         --  decision 1: a new RA starts
         S := (Active    => True,
               Adv       =>
                 Tcas.RA.Generate (Own_Alt, Own_Rate, Own_SL, Item),
               Address   => Item.Address,
               Age       => 1,
               Clear_Run => 0,
               Reversed  => False);
         Event := Issued;

      elsif not S.Active then
         null;   --  decision 2: nothing active, nothing to start

      elsif Same and then Is_RA
        and then Reversal_Due (S, Own_Alt, Own_Rate, Own_SL, Item)
      then
         --  decision 3: the current sense fails, the opposite one works
         S.Adv       := Make_Advisory (Opposite (S.Adv.Sense));
         S.Reversed  := True;
         S.Age       := Next (S.Age);
         S.Clear_Run := 0;
         Event := Reversed;

      elsif Same and then Is_RA then
         --  decision 4: the same advisory continues
         S.Age       := Next (S.Age);
         S.Clear_Run := 0;
         Event := Maintained;

      elsif New_Run >= Tcas.Config.RA_Clear_Cycles then
         --  decision 5: enough quiet cycles, the advisory ends
         S := Inactive;
         Event := Cleared;

      else
         --  decision 6: quiet, but not for long enough: hold and count
         S.Clear_Run := New_Run;
         S.Age       := Next (S.Age);
         Event := Clearing;
      end if;
   end Step;

end Tcas.RA_State;