--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/common/units/tcas-units.ads
--  Module      : Safe arithmetic between the physical types (spec only)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : shared by P2, P3 and P4 (pure, no state)
--  Requirements: REQ-002 (absence of runtime errors), placeholder IDs
--
--  PURPOSE
--    The types in Tcas.Types have tight ranges, so ordinary arithmetic on
--    them can leave the range (the difference of two altitudes is not an
--    altitude). This package gives the logic a small set of operations
--    that are proved to stay in range, so no other unit needs to do
--    arithmetic that could overflow. It is the Ada counterpart of the
--    Fixed_Math helpers in the FSAC project.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure package, expression functions only: no state, no body.
--        GNATprove sees the full definition of every function, so a
--        postcondition can be proved without a separate implementation.
--    R2  Intermediate arithmetic is done in Integer (32 bits on the
--        target). The largest intermediate value here is 1 200 000
--        (rate times time), far below the 32-bit limit, so no
--        intermediate step can overflow.
--    R3  Every conversion back into a physical type SATURATES to the
--        range of that type. A value outside the range becomes the
--        nearest bound. It never raises Constraint_Error.
--    R4  Division uses truncation toward zero (Ada's "/" on Integer), the
--        same rule on host and target, so results are bit-exact between
--        lanes. A divisor is never zero: each division sits behind a
--        condition that proves it.
--    R5  Every function declares Global => null: it reads nothing but its
--        parameters.
--
--  UNITS USED
--    feet (ft), feet per minute (ft/min), feet per second (ft/s),
--    seconds (s). One minute is 60 seconds.
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * No deviations are requested by this specification.
--  ===========================================================================

pragma SPARK_Mode (On);

with Tcas.Types; use Tcas.Types;

package Tcas.Units
  with Pure
is

   --  Saturating conversions into the physical types

   --  Each function returns the nearest value inside the target range
   --  (rule R3). They are the only way to convert from Integer.

   function Clamp_Altitude (Value : Integer) return Altitude_Ft is
     (if Value < Integer (Altitude_Ft'First) then Altitude_Ft'First
      elsif Value > Integer (Altitude_Ft'Last) then Altitude_Ft'Last
      else Altitude_Ft (Value))
   with Global => null;

   function Clamp_Vertical_Rate (Value : Integer) return Vertical_Rate_Fpm is
     (if Value < Integer (Vertical_Rate_Fpm'First)
      then Vertical_Rate_Fpm'First
      elsif Value > Integer (Vertical_Rate_Fpm'Last)
      then Vertical_Rate_Fpm'Last
      else Vertical_Rate_Fpm (Value))
   with Global => null;

   function Clamp_Range (Value : Integer) return Range_Ft is
     (if Value < Integer (Range_Ft'First) then Range_Ft'First
      elsif Value > Integer (Range_Ft'Last) then Range_Ft'Last
      else Range_Ft (Value))
   with Global => null;

   function Clamp_Tau (Value : Integer) return Tau_S is
     (if Value < Integer (Tau_S'First) then Tau_S'First
      elsif Value > Integer (Tau_S'Last) then Tau_S'Last
      else Tau_S (Value))
   with Global => null;

   --  Vertical geometry

   --  Signed difference A - B. Altitude_Diff_Ft is wide enough for any
   --  pair of altitudes, so this can never overflow (design of
   --  Tcas.Types, rule R2 there).
   function Altitude_Difference (A, B : Altitude_Ft)
     return Altitude_Diff_Ft is
     (Altitude_Diff_Ft (A) - Altitude_Diff_Ft (B))
   with Global => null;

   --  Altitude A moved by a signed difference, saturated.
   function Add_Altitude (A : Altitude_Ft; Delta_Ft : Altitude_Diff_Ft)
     return Altitude_Ft is
     (Clamp_Altitude (Integer (A) + Integer (Delta_Ft)))
   with Global => null;

   --  Altitude after Seconds seconds at a constant vertical rate:
   --    Alt + Rate * Seconds / 60, truncated toward zero, saturated.
   --  The product is at most 10 000 * 120 = 1 200 000 (rule R2).
   function Project_Altitude
     (Alt     : Altitude_Ft;
      Rate    : Vertical_Rate_Fpm;
      Seconds : Tau_S) return Altitude_Ft is
     (Clamp_Altitude
        (Integer (Alt) + (Integer (Rate) * Integer (Seconds)) / 60))
   with Global => null;

   --  Horizontal geometry

   --  Time to closest approach, in seconds: range divided by closing
   --  speed. If the aircraft are not closing (rate zero or positive) the
   --  result is the sentinel No_Tau. A result above the largest tau
   --  saturates to No_Tau as well. The divisor is -Rate, which is
   --  positive in the branch where the division happens (rule R4).
   function Compute_Tau (Slant : Range_Ft; Rate : Range_Rate_Fps)
     return Tau_S is
     (if Rate >= 0 then No_Tau
      else Clamp_Tau (Integer (Slant) / Integer (-Rate)))
   with
     Global => null,
     Post   => (if Rate >= 0 then Compute_Tau'Result = No_Tau);

end Tcas.Units;