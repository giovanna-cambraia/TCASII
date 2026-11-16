--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/threat/tcas-sensitivity.adb
--  Module      : Sensitivity level selection (body)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Implements Select_Level declared in tcas-sensitivity.ads. The
--    specification holds the contract (rules, order of decisions); this
--    body holds the code that makes it true. Read the specification first.
--
--  DESIGN RULES (as in the specification, with the way each is met here)
--    R1  No state: the function works only on its parameters.
--    R2  No altitude literal: every boundary comes from Tcas.Config.
--    R3  The result starts at SL_Fail_Safe (SL 2). Only the branches for
--        SL 3 to SL 7 assign a different value, so the fail-safe level is
--        the default of every path that does not name another level. The
--        postconditions "never below SL 2" and "untrusted pressure
--        altitude gives SL 2" are true by construction.
--    R4  A missing radio altitude gives SL 2 only when the pressure
--        altitude is in the low band (decision 4). Higher up, a missing
--        radio altitude is normal and the pressure altitude decides.
--    R5  SL 1 is never assigned anywhere in this body.
--    R6  Constant cost: no loop, one flat chain of comparisons.
--
--  STRUCTURE OF THE CODE
--    One exit (a single return at the end). A flat if / elsif chain, one
--    branch per decision, in the order of the specification, so each
--    condition is a separate, testable branch for MC/DC coverage
--    (DO-178C Table A-7). The branches that keep SL 2 contain only
--    "null" and a comment that names the decision.
--
--  PROOF OBLIGATIONS (discharged by GNATprove, tcas2.gpr level 2)
--    * Absence of runtime errors: only comparisons and assignments of
--      literals inside the range of Sensitivity_Level.
--    * The postconditions of Select_Level.
--
--  SPARK NOTES
--    * SPARK_Mode is On.
--    * The with and use clauses of the specification apply to this body,
--      so Tcas.Config and Tcas.Types are visible without repeating them.
--    * No deviations are requested by this body.

pragma SPARK_Mode (On);

package body Tcas.Sensitivity is

   function Select_Level
     (Pressure : Altitude_Source;
      Radio    : Altitude_Source) return Sensitivity_Level
   is
      Result : Sensitivity_Level := SL_Fail_Safe;
   begin
      if not Pressure.Valid then
         null;   --  decision 1: untrusted pressure altitude, SL 2
      elsif Radio.Valid
        and then Radio.Value <= Tcas.Config.SL2_Upper_Ft
      then
         null;   --  decision 2: radio altitude at or below the SL 2 limit
      elsif Radio.Valid
        and then Radio.Value <= Tcas.Config.SL3_Upper_Ft
      then
         Result := 3;                                        --  decision 3
      elsif not Radio.Valid
        and then Pressure.Value <= Tcas.Config.SL3_Upper_Ft
      then
         null;   --  decision 4: low band, radio altitude missing, SL 2
      elsif Pressure.Value <= Tcas.Config.SL4_Upper_Ft then
         Result := 4;                                        --  decision 5
      elsif Pressure.Value <= Tcas.Config.SL5_Upper_Ft then
         Result := 5;                                        --  decision 6
      elsif Pressure.Value <= Tcas.Config.SL6_Upper_Ft then
         Result := 6;                                        --  decision 7
      else
         Result := 7;                                        --  decision 8
      end if;

      return Result;
   end Select_Level;

end Tcas.Sensitivity;