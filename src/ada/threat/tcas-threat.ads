--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/threat/tcas-threat.ads
--  Module      : Threat detector, classification of one track (spec)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Decides how dangerous one intruder is. It reads one track from the
--    track table (Tcas.Tracker), the ownship altitude and vertical rate,
--    and the sensitivity level, and returns a Threat_Class: other traffic,
--    proximate traffic, traffic advisory or resolution advisory. It does
--    NOT choose a maneuver (that is the RA generator) and it does NOT
--    change anything: every function has Global => null and reads only
--    its parameters.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure functions, no state. The detector can be tested and proved
--        alone, and it cannot change the track table.
--    R2  No number in this unit. Every threshold comes from Tcas.Config,
--        indexed by the sensitivity level, so the mapping from
--        requirement to number is one lookup.
--    R3  A threshold of 0 never triggers (rule R5 of Tcas.Config). This is
--        stated in the postconditions of the two tests, so the RA-free
--        levels (SL 1 and 2) are a proved property, not a convention.
--    R4  Corroboration. A track with fewer than Min_Hits_For_Advisory
--        updates can be at most Proximate_Traffic. It never becomes a TA
--        or an RA from one reply (security analysis: spoofing).
--    R5  Fail towards the weaker class only for the reasons above. A
--        free slot is Other_Traffic. Nothing else lowers a class.
--    R6  Arithmetic is the proved arithmetic of Tcas.Units. The absolute
--        value of a vertical separation cannot overflow because
--        Altitude_Diff_Ft is symmetric (-127 900 .. 127 900).
--
--  THE TWO TESTS (both must pass for an advisory)
--    Range test
--      passes when Tau_Threshold > 0 and (Slant <= DMOD or Tau <= the
--      threshold). Tau = Slant / closing speed (Tcas.Units.Compute_Tau);
--      an intruder that is not closing has Tau = No_Tau and can pass only
--      by the DMOD distance.
--    Vertical test
--      passes when ZTHR > 0 and the vertical separation is <= ZTHR now,
--      or will be <= ZTHR at Tau seconds, with both aircraft projected at
--      their vertical rates (Tcas.Units.Project_Altitude). No projection
--      is made when Tau = No_Tau.
--
--  ORDER OF DECISIONS IN Classify (the first match wins)
--    1 the slot is Free                          Other_Traffic
--    2 sensitivity level is 1 (standby)          Other_Traffic
--    3 corroborated, SL >= 3, both tests with the RA thresholds
--                                                Resolution_Advisory
--    4 corroborated, both tests with the TA thresholds
--                                                Traffic_Advisory
--    5 within the proximate range and altitude   Proximate_Traffic
--    6 anything else                             Other_Traffic
--
--  UNITS USED
--    feet (ft), feet per minute (ft/min), seconds (s).
--
--  KNOWN SIMPLIFICATION (open item for v2)
--    DO-185B uses a modified tau with DMOD built into the formula. Here
--    the plain tau and a separate DMOD distance test are used. Both are
--    in the specification of Tcas.Config as [VERIFY] values.
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * The body is in tcas-threat.adb; GNATprove proves the
--      postconditions and the absence of runtime errors there.
--    * No deviations are requested by this specification.

pragma SPARK_Mode (On);

with Tcas.Config;
with Tcas.Tracker;
with Tcas.Types; use Tcas.Types;

package Tcas.Threat is

   package Tr renames Tcas.Tracker;

   use type Tr.Track_State;
   use type Tr.Hit_Count;

   --  Building blocks (public so that the tests can check them alone)

   --  Absolute vertical distance between two altitudes. Never negative.
   function Vertical_Separation (A, B : Altitude_Ft)
     return Altitude_Diff_Ft
   with
     Global => null,
     Post   => Vertical_Separation'Result >= 0;

   --  Range test of one intruder against one pair of thresholds.
   --  A threshold of 0 never passes (rule R3).
   function Range_Test
     (Item          : Tr.Track;
      Tau_Threshold : Tau_S;
      DMOD          : Range_Ft) return Boolean
   with
     Global => null,
     Post   => (if Tau_Threshold = 0 then not Range_Test'Result);

   --  Vertical test of one intruder against one vertical threshold.
   --  A threshold of 0 never passes (rule R3).
   function Vertical_Test
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Item     : Tr.Track;
      ZTHR     : Altitude_Diff_Ft) return Boolean
   with
     Global => null,
     Post   => (if ZTHR = 0 then not Vertical_Test'Result);

   --  Classification

   --  Class of one track. Own_SL is the current sensitivity level.
   function Classify
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track) return Threat_Class
   with
     Global => null,
     Post   =>
       --  a free slot is never a threat
       (if Item.State = Tr.Free then Classify'Result = Other_Traffic)
       --  standby gives no alert and no advisory
       and then (if Own_SL = 1 then Classify'Result = Other_Traffic)
       --  an advisory needs an active, corroborated track (rule R4)
       and then
         (if Classify'Result in Traffic_Advisory | Resolution_Advisory
          then Item.State = Tr.Active
               and then Item.Updates >= Tcas.Config.Min_Hits_For_Advisory)
       --  an RA exists only from SL 3 up (rule R3, SL 1 and 2 have none)
       and then
         (if Classify'Result = Resolution_Advisory then Own_SL >= 3);

end Tcas.Threat;