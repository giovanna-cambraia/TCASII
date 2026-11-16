--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/threat/tcas-threat.adb
--  Module      : Threat detector, classification of one track (body)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Implements the functions declared in tcas-threat.ads. The
--    specification holds the contract (rules, the two tests, the order of
--    decisions); this body holds the code that makes it true. Read the
--    specification first.
--
--  DESIGN RULES (as in the specification, with the way each is met here)
--    R1  No state: every function works only on its parameters.
--    R2  No threshold literal: every number comes from Tcas.Config.
--    R3  A threshold of 0 never passes. Each test starts its boolean
--        expression with "threshold > 0 and then ...", so a zero
--        threshold gives False before anything else is looked at.
--    R4  Corroboration is a named constant (Corroborated) used by both
--        advisory branches, so the rule exists in one place.
--    R5  The RA branch tests Own_SL >= 3 explicitly. The postcondition
--        "an RA needs SL 3 or more" is then true by construction and does
--        not depend on the contents of the threshold table. The zero RA
--        rows for SL 1 and 2 stay as a second line of defense.
--    R6  Constant cost. Both tests compute every value they might need
--        (tau, separations, both projected altitudes) before the decision,
--        and combine them with short-circuit operators. Nothing is skipped
--        by a loop or an early return.
--
--  STRUCTURE OF THE CODE
--    Every function has ONE exit (a single return at the end). Classify
--    is a flat if / elsif chain, one branch per decision, in the order of
--    the specification, so each condition is a separate, testable branch
--    for MC/DC coverage (DO-178C Table A-7). The failing branches contain
--    only "null": the result keeps its starting value, Other_Traffic.
--
--  PROOF OBLIGATIONS (discharged by GNATprove, tcas2.gpr level 2)
--    * Absence of runtime errors: the negation in Vertical_Separation
--      (Altitude_Diff_Ft is symmetric), the table index in Classify
--      (Own_SL is in the index range), every conversion in Tcas.Units.
--    * The postconditions of Vertical_Separation, Range_Test,
--      Vertical_Test and Classify.
--
--  SPARK NOTES
--    * SPARK_Mode is On.
--    * The with and use clauses of the specification apply to this body,
--      so Tcas.Config, Tcas.Tracker and Tcas.Types are visible without
--      repeating them here.
--    * No deviations are requested by this body.
--  ===========================================================================

pragma SPARK_Mode (On);

with Tcas.Units; use Tcas.Units;

package body Tcas.Threat is

   --  Building blocks

   function Vertical_Separation (A, B : Altitude_Ft)
     return Altitude_Diff_Ft
   is
      D : constant Altitude_Diff_Ft := Altitude_Difference (A, B);
   begin
      return (if D < 0 then -D else D);
   end Vertical_Separation;

   function Range_Test
     (Item          : Tr.Track;
      Tau_Threshold : Tau_S;
      DMOD          : Range_Ft) return Boolean
   is
      Tau : constant Tau_S := Compute_Tau (Item.Slant, Item.Range_Rate);
   begin
      --  a zero threshold never passes (rule R3); an intruder that is not
      --  closing has Tau = No_Tau, which is above every tau threshold, so
      --  it can pass only by the DMOD distance
      return Tau_Threshold > 0
        and then (Item.Slant <= DMOD or else Tau <= Tau_Threshold);
   end Range_Test;

   function Vertical_Test
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Item     : Tr.Track;
      ZTHR     : Altitude_Diff_Ft) return Boolean
   is
      Tau      : constant Tau_S := Compute_Tau (Item.Slant, Item.Range_Rate);
      Now_Sep  : constant Altitude_Diff_Ft :=
        Vertical_Separation (Own_Alt, Item.Altitude);
      Own_Proj : constant Altitude_Ft :=
        Project_Altitude (Own_Alt, Own_Rate, Tau);
      Int_Proj : constant Altitude_Ft :=
        Project_Altitude (Item.Altitude, Item.Vertical_Rate, Tau);
   begin
      --  a zero threshold never passes (rule R3); the projection counts
      --  only when the aircraft are closing (Tau /= No_Tau)
      return ZTHR > 0
        and then
          (Now_Sep <= ZTHR
           or else
             (Tau /= No_Tau
              and then Vertical_Separation (Own_Proj, Int_Proj) <= ZTHR));
   end Vertical_Test;

   --  Classification

   function Classify
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track) return Threat_Class
   is
      Row : constant Tcas.Config.Level_Thresholds :=
        Tcas.Config.Thresholds (Own_SL);

      --  rule R4: one reply is not enough for an advisory
      Corroborated : constant Boolean :=
        Item.Updates >= Tcas.Config.Min_Hits_For_Advisory;

      Result : Threat_Class := Other_Traffic;
   begin
      if Item.State /= Tr.Active then
         null;   --  decision 1: a free slot is Other_Traffic
      elsif Own_SL = 1 then
         null;   --  decision 2: standby, no alert and no advisory
      elsif Corroborated
        and then Own_SL >= 3
        and then Range_Test (Item, Row.RA_Tau, Row.RA_DMOD)
        and then Vertical_Test (Own_Alt, Own_Rate, Item, Row.RA_ZTHR)
      then
         Result := Resolution_Advisory;                      --  decision 3
      elsif Corroborated
        and then Range_Test (Item, Row.TA_Tau, Row.TA_DMOD)
        and then Vertical_Test (Own_Alt, Own_Rate, Item, Row.TA_ZTHR)
      then
         Result := Traffic_Advisory;                         --  decision 4
      elsif Item.Slant <= Tcas.Config.Proximate_Range_Ft
        and then Vertical_Separation (Own_Alt, Item.Altitude)
                   <= Tcas.Config.Proximate_Alt_Ft
      then
         Result := Proximate_Traffic;                        --  decision 5
      else
         null;   --  decision 6: Other_Traffic
      end if;

      return Result;
   end Classify;

end Tcas.Threat;