--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/ra/tcas-ra.adb
--  Module      : Resolution advisory generation, sense selection (body)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Implements the functions declared in tcas-ra.ads. The specification
--    holds the contract (rules, pilot response model, order of decisions);
--    this body holds the code that makes it true. Read the specification
--    first.
--
--  DESIGN RULES (as in the specification, with the way each is met here)
--    R1  No state: every function works only on its parameters.
--    R2  No number in this unit: the delay, the target rate and the
--        separation goal come from Tcas.Config.
--    R3  The preconditions are the caller's promise. Inside the body,
--        Own_SL >= 3 is what makes the table index and the separation
--        goal meaningful.
--    R4  Choose_Sense starts at Upward and only the Downward branches
--        assign. No path can return No_Sense, so the postcondition is true
--        by construction. Generate maps the sense to its advisory with a
--        two-way choice, so the sense and the advisory always match.
--    R5  Non-crossing sense preferred (decisions 1 and 2 of the
--        specification): when both senses reach the goal, the sense that
--        keeps the ownship on its current side wins.
--    R6  Constant cost: both separations are computed before any decision.
--        No loop.
--
--  ARITHMETIC (all in Integer, truncation toward zero, as Tcas.Units)
--    Own projected altitude =
--        Own_Alt
--      + (Own_Rate * Delay_Part) / 60          current rate, first seconds
--      + (Target   * After_Part) / 60          target rate, the rest
--    where Delay_Part = min (tau, RA_Response_Delay_S) and
--    After_Part = tau - Delay_Part. Each term is divided by 60 on its own,
--    so every term truncates separately. The largest intermediate value is
--    1 500 * 120 = 180 000, far below the 32-bit limit, and the sum is
--    saturated into Altitude_Ft by Tcas.Units.Clamp_Altitude.
--    The intruder uses Tcas.Units.Project_Altitude with its own rate.
--
--  STRUCTURE OF THE CODE
--    Local helper first (Project_Own), then the functions of the
--    specification in the order Effective_Tau, Sense_Separation,
--    Choose_Sense, Generate. Every function has ONE exit (a single return
--    at the end). Choose_Sense is a flat if / elsif chain, one branch per
--    decision, in the order of the specification, so each condition is a
--    separate, testable branch for MC/DC coverage (DO-178C Table A-7).
--
--  PROOF OBLIGATIONS (discharged by GNATprove, tcas2.gpr level 2)
--    * Absence of runtime errors: every integer expression above stays in
--      range, the table index Own_SL is in the index range, the negation
--      of the target rate cannot overflow (Vertical_Rate_Fpm is
--      symmetric).
--    * The postconditions of Effective_Tau, Choose_Sense and Generate, and
--      the precondition of Sense_Separation at each call.
--
--  SPARK NOTES
--    * SPARK_Mode is On.
--    * The with and use clauses of the specification apply to this body,
--      so Tcas.Config, Tcas.Tracker and Tcas.Types are visible without
--      repeating them here.
--    * No deviations are requested by this body.

pragma SPARK_Mode (On);

with Tcas.Units; use Tcas.Units;

package body Tcas.RA is

   --  Own projection (local to this body)

   --  Ownship altitude at closest approach, following the pilot response
   --  model: the current rate for the response delay, then the target rate.
   function Project_Own
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Target   : Vertical_Rate_Fpm;
      Seconds  : Tau_S) return Altitude_Ft
   with
     Global => null;

   function Project_Own
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Target   : Vertical_Rate_Fpm;
      Seconds  : Tau_S) return Altitude_Ft
   is
      Delay_Part : constant Integer :=
        Integer'Min (Integer (Seconds),
                     Integer (Tcas.Config.RA_Response_Delay_S));
      After_Part : constant Integer := Integer (Seconds) - Delay_Part;
   begin
      return Clamp_Altitude
        (Integer (Own_Alt)
         + (Integer (Own_Rate) * Delay_Part) / 60
         + (Integer (Target) * After_Part) / 60);
   end Project_Own;

   --  Time to closest approach

   function Effective_Tau (Item : Tr.Track) return Tau_S is
      Result : Tau_S := 0;
   begin
      if Item.Range_Rate >= 0 then
         null;   --  not closing: evaluate at tau 0 (current geometry)
      else
         Result := Compute_Tau (Item.Slant, Item.Range_Rate);
      end if;

      return Result;
   end Effective_Tau;

   --  Separation of one sense

   function Sense_Separation
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Sense    : RA_Sense;
      Item     : Tr.Track) return Altitude_Diff_Ft
   is
      Tau : constant Tau_S := Effective_Tau (Item);

      --  the rate the pilot is asked to reach for this sense
      Target : constant Vertical_Rate_Fpm :=
        (if Sense = Upward
         then Tcas.Config.RA_Target_Rate_Fpm
         else -Tcas.Config.RA_Target_Rate_Fpm);

      Own_Proj : constant Altitude_Ft :=
        Project_Own (Own_Alt, Own_Rate, Target, Tau);

      Int_Proj : constant Altitude_Ft :=
        Project_Altitude (Item.Altitude, Item.Vertical_Rate, Tau);
   begin
      --  positive is good: Upward ends above the intruder, Downward ends
      --  below it
      return (if Sense = Upward
              then Altitude_Difference (Own_Proj, Int_Proj)
              else Altitude_Difference (Int_Proj, Own_Proj));
   end Sense_Separation;

   --  Choice of the sense

   function Choose_Sense
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track) return RA_Sense
   is
      --  separation goal of this level; Own_SL >= 3, so it is a real goal
      ALIM : constant Altitude_Diff_Ft :=
        Tcas.Config.Thresholds (Own_SL).RA_ALIM;

      Up_Sep   : constant Altitude_Diff_Ft :=
        Sense_Separation (Own_Alt, Own_Rate, Upward, Item);
      Down_Sep : constant Altitude_Diff_Ft :=
        Sense_Separation (Own_Alt, Own_Rate, Downward, Item);

      Up_Ok   : constant Boolean := Up_Sep >= ALIM;
      Down_Ok : constant Boolean := Down_Sep >= ALIM;

      --  starts at Upward: no path can return No_Sense (rule R4)
      Result : RA_Sense := Upward;
   begin
      if Up_Ok and then Down_Ok and then Own_Alt >= Item.Altitude then
         null;   --  decision 1: both reach the goal, above or level, Upward
      elsif Up_Ok and then Down_Ok then
         Result := Downward;                                 --  decision 2
      elsif Up_Ok then
         null;   --  decision 3: only Upward reaches the goal
      elsif Down_Ok then
         Result := Downward;                                 --  decision 4
      elsif Up_Sep >= Down_Sep then
         null;   --  decision 5: neither reaches it, Upward gives more
      else
         Result := Downward;                                 --  decision 6
      end if;

      return Result;
   end Choose_Sense;

   --  Advisory

   function Generate
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track) return Advisory
   is
      Sense : constant RA_Sense :=
        Choose_Sense (Own_Alt, Own_Rate, Own_SL, Item);
   begin
      --  the sense and the advisory always match (rule R4)
      return (if Sense = Upward
              then (Kind        => Climb,
                    Sense       => Upward,
                    Target_Rate => Tcas.Config.RA_Target_Rate_Fpm)
              else (Kind        => Descend,
                    Sense       => Downward,
                    Target_Rate => -Tcas.Config.RA_Target_Rate_Fpm));
   end Generate;

end Tcas.RA;