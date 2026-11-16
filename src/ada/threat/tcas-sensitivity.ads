--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/threat/tcas-sensitivity.ads
--  Module      : Sensitivity level selection (specification)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    The thresholds of the threat logic depend on the sensitivity level
--    (SL). This package chooses the level from the two ownship altitude
--    sources: the pressure altitude (above sea level) and the radio
--    altitude (above the ground, valid only near the ground). It returns
--    a level from 2 to 7. The detector (Tcas.Threat) takes it as input.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure function, no state, Global => null.
--    R2  No number in this unit. Every altitude boundary comes from
--        Tcas.Config (SL2_Upper_Ft .. SL6_Upper_Ft).
--    R3  FAIL SAFE. A pressure altitude that is not valid gives SL2
--        (SL_Fail_Safe): traffic advisories only, no resolution
--        advisory from a source we cannot trust.
--    R4  A missing radio altitude only matters near the ground. A radio
--        altimeter does not work at cruise altitude, so "no radio
--        altitude" is normal there. It gives SL2 only when the pressure
--        altitude is in the low band (at or below SL3_Upper_Ft), where
--        the radio altitude would have decided the level.
--    R5  Standby (SL 1) is never returned. Standby is commanded by other
--        logic (for example, aircraft on the ground), not by altitude.
--    R6  Constant cost: a flat chain of comparisons, no loop.
--
--  ORDER OF DECISIONS IN Select_Level (the first match wins)
--    1 pressure altitude not valid                          SL 2
--    2 radio valid and <= SL2_Upper_Ft                      SL 2
--    3 radio valid and <= SL3_Upper_Ft                      SL 3
--    4 radio not valid and pressure <= SL3_Upper_Ft         SL 2
--    5 pressure <= SL4_Upper_Ft                             SL 4
--    6 pressure <= SL5_Upper_Ft                             SL 5
--    7 pressure <= SL6_Upper_Ft                             SL 6
--    8 anything higher                                      SL 7
--
--  KNOWN SIMPLIFICATION (open item for v2)
--    No hysteresis. An aircraft hovering at a boundary can change level
--    on every cycle. DO-185B defines different boundaries for climbing
--    and descending. The boundaries themselves are [VERIFY] in
--    Tcas.Config.
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * The body is in tcas-sensitivity.adb; GNATprove proves the
--      postconditions and the absence of runtime errors there.
--    * No deviations are requested by this specification.
--  ===========================================================================

pragma SPARK_Mode (On);

with Tcas.Config;
with Tcas.Types; use Tcas.Types;

package Tcas.Sensitivity is

   --  One ownship altitude reading together with its validity. The
   --  decoder (Tcas.A429_Decoder) provides both: Valid is True when the
   --  status is Decoded. When Valid is False, Value is not used.
   type Altitude_Source is record
      Valid : Boolean      := False;
      Value : Altitude_Ft  := 0;
   end record;

   --  Level for the given sources. Never SL 1 (rule R5).
   function Select_Level
     (Pressure : Altitude_Source;
      Radio    : Altitude_Source) return Sensitivity_Level
   with
     Global => null,
     Post   =>
       --  never standby (rule R5)
       Select_Level'Result >= SL_Fail_Safe
       --  an untrusted pressure altitude gives the fail-safe level (R3)
       and then
         (if not Pressure.Valid then Select_Level'Result = SL_Fail_Safe)
       --  a radio altitude at or below 1 000 ft never gives more than SL 2
       and then
         (if Pressure.Valid
             and then Radio.Valid
             and then Radio.Value <= Tcas.Config.SL2_Upper_Ft
          then Select_Level'Result = SL_Fail_Safe);

end Tcas.Sensitivity;