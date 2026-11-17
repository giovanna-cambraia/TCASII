--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/ra/tcas-ra_state.ads
--  Module      : Resolution advisory life cycle (specification)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Tcas.RA chooses a sense for ONE cycle and forgets it. This package is
--    the memory across cycles. Once an RA is issued it keeps the same
--    advisory while the threat lasts, so the pilot is not told to climb
--    one second and descend the next. It also decides when to reverse the
--    sense (once) and when to clear the advisory.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  No package state. The life cycle is a value (RA_State) that the
--        CALLER owns, as in Tcas.Tracker. Every subprogram has
--        Global => null.
--    R2  One threat at a time. The state follows one aircraft, by its Mode
--        S address. A second aircraft that reaches RA level is ignored
--        until the first RA is cleared.
--    R3  Sticky advisory. While the same aircraft stays an RA, the
--        advisory does not change, except by a reversal.
--    R4  At most one reversal per RA. A reversal happens only when the
--        current sense no longer reaches the separation goal (ALIM) and
--        the opposite sense does.
--    R5  Clearing needs RA_Clear_Cycles consecutive cycles in which the
--        followed aircraft is no longer an RA. A single quiet cycle only
--        starts the count; the advisory is held meanwhile.
--    R6  Is_Consistent is an invariant, stated as a pre and a
--        postcondition of Step: an active state always carries a real
--        advisory (matching sense, kind and rate), an inactive state
--        carries none, and the counters stay in their bounds.
--    R7  Counters saturate and never wrap.
--
--  ORDER OF DECISIONS IN Step (the first match wins)
--    1 no active RA, the track is an RA                    Issued
--    2 no active RA, anything else                         No_Event
--    3 active, same aircraft, RA, a reversal is due        Reversed
--    4 active, same aircraft, RA, no reversal              Maintained
--    5 active, not an RA, quiet cycles reached the limit   Cleared
--    6 active, not an RA, quiet cycles below the limit     Clearing
--  "Same aircraft" means the track is active and its address equals
--  State.Address. A track that is another aircraft, or free, counts as
--  "not an RA" for the followed aircraft (decisions 5 and 6).
--
--  KNOWN SIMPLIFICATIONS (open items for v2)
--    One threat only; no coordination between aircraft (Mode S);
--    reversal test uses the separation goal only; only Climb and Descend;
--    RA_Clear_Cycles and the one-reversal limit are [VERIFY] values.
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * The body is in tcas-ra_state.adb; GNATprove proves the
--      postconditions and the absence of runtime errors there.
--    * No deviations are requested by this specification.

pragma SPARK_Mode (On);

with Tcas.Config;
with Tcas.RA;
with Tcas.Tracker;
with Tcas.Types; use Tcas.Types;

use type Tcas.RA.RA_Kind;
use type Tcas.RA.Advisory;

package Tcas.RA_State is

   package Tr renames Tcas.Tracker;

   --  Number of cycles counted, saturating at 65 535.
   type Cycle_Count is range 0 .. 65_535;

   --  What happened in one cycle.
   type RA_Event is
     (No_Event,     --  nothing to report
      Issued,       --  a new RA started
      Maintained,   --  the RA continues with the same sense
      Reversed,     --  the sense changed (once per RA)
      Clearing,     --  the threat is quiet, advisory held, counting
      Cleared);     --  the RA ended

   --  The life cycle of the advisory.
   type State is record
      Active    : Boolean          := False;
      Adv       : Tcas.RA.Advisory := (Kind        => Tcas.RA.No_Advisory,
                                       Sense       => No_Sense,
                                       Target_Rate => 0);
      Address   : Mode_S_Address   := 0;   --  the aircraft being followed
      Age       : Cycle_Count      := 0;   --  cycles since the RA started
      Clear_Run : Cycle_Count      := 0;   --  consecutive quiet cycles
      Reversed  : Boolean          := False;  --  a reversal was used
   end record;

   --  True when the state obeys its invariant (rule R6).
   function Is_Consistent (S : State) return Boolean is
     (if S.Active
      then S.Adv.Sense /= No_Sense
           and then
             (if S.Adv.Sense = Upward
              then S.Adv.Kind = Tcas.RA.Climb
                   and then S.Adv.Target_Rate
                              = Tcas.Config.RA_Target_Rate_Fpm
              else S.Adv.Kind = Tcas.RA.Descend
                   and then S.Adv.Target_Rate
                              = -Tcas.Config.RA_Target_Rate_Fpm)
           and then S.Clear_Run < Tcas.Config.RA_Clear_Cycles
      else S.Adv.Kind = Tcas.RA.No_Advisory
           and then S.Adv.Sense = No_Sense
           and then S.Adv.Target_Rate = 0
           and then not S.Reversed
           and then S.Clear_Run = 0)
   with Global => null;

   --  Starts with no advisory. Call before the first Step.
   procedure Init (S : out State)
   with
     Global => null,
     Post   => not S.Active and then Is_Consistent (S);

   --  One cycle of the life cycle. Item is the intruder that the caller
   --  wants followed (normally the most urgent one). Step classifies it
   --  with Tcas.Threat.Classify, so the caller never passes a
   --  classification.
   procedure Step
     (S        : in out State;
      Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track;
      Event    : out RA_Event)
   with
     Global => null,
     Pre    => Is_Consistent (S),
     Post   =>
       Is_Consistent (S)
       --  an event of "no RA" kinds leaves an inactive state
       and then (if Event in No_Event | Cleared then not S.Active)
       --  an event of "RA" kinds leaves an active state
       and then
         (if Event in Issued | Maintained | Reversed | Clearing
          then S.Active)
       --  a new RA starts from scratch
       and then
         (if Event = Issued
          then S.Age = 1 and then not S.Reversed
               and then S.Address = Item.Address)
       --  the advisory changes ONLY on Issued, Reversed or Cleared (R3)
       and then
         (if Event in Maintained | Clearing
          then S.Adv = S'Old.Adv and then S.Address = S'Old.Address)
       --  one reversal per RA (rule R4)
       and then
         (if Event = Reversed
          then S.Reversed and then not S'Old.Reversed
               and then S.Adv.Sense /= S'Old.Adv.Sense);

end Tcas.RA_State;