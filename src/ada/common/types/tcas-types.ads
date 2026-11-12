--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/common/types/tcas-types.ads
--  Module      : Common physical and protocol types (specification only)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : shared by P2, P3 and P4 (pure, no state, no code)
--  Requirements: REQ-002 (absence of runtime errors), placeholder IDs
--
--  PURPOSE
--    This package is the vocabulary of the whole system. Every number that
--    represents something physical (a height, a speed, a distance, a time)
--    gets its own type with a fixed unit and a fixed range. The C layer
--    delivers raw words; the decoder (partition 2) converts them into these
--    types; the core (partition 3) works only with these types.
--
--    The ranges are the contract. When the decoder rejects a value outside
--    them, the core may assume every value it receives is inside them, and
--    GNATprove can prove that no computation in the core overflows.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure package: no state, no elaboration code, no body. Nothing here
--        can fail at run time (SPARK_Mode, pragma Pure).
--    R2  One type per physical meaning. Altitude_Ft and Range_Ft are both
--        feet but are different types, so they cannot be mixed by accident.
--    R3  Integer types with explicit units, no floating point. Integers are
--        exact, have a bounded range, and behave identically on host and
--        target (bit-exact comparison between lanes).
--    R4  Every range is the PHYSICAL envelope plus a small margin, never
--        "as large as the machine allows". A tight range lets proofs of
--        overflow freedom succeed with simple arithmetic.
--    R5  Types that cross the Ada/C boundary have a fixed size and
--        representation (Size and enumeration representation clauses), so
--        the layout matches the C headers.
--    R6  Sentinel values are named constants (No_Tau), never magic numbers
--        at the point of use.
--
--  UNITS USED
--    feet (ft), feet per minute (ft/min), feet per second (ft/s),
--    hundredths of a degree (cdeg), seconds (s), milliseconds (ms).
--    The units follow the aviation conventions of RTCA DO-185B, where
--    altitude is in feet and vertical rate is in feet per minute.
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * Pure: the package may be withed from any other unit without adding
--      dependencies on state.
--    * No deviations are requested by this specification.
--  ===========================================================================

pragma SPARK_Mode (On);

package Tcas.Types
  with Pure
is


   --  capacity constants

   --  largest number of intruder tracks held at the same time; the tracker uses a statically sized array of this length 
   --  (ASM-004: no heap)
   Max_Tracks : constant := 60;

   --  length of the major surveillance and threat-evaluation cycle (ASM-001)
   Cycle_Ms : constant := 1_000;

   --  sltitude and vertical motion

   --  pressure altitude in feet, the lower bound allows operation below sea level; the upper bound is the highest value a Mode S altitude
   --  field can carry (126 700 ft)
   type Altitude_Ft is range -1_200 .. 126_700;

   --  vertical difference between two aircraft, in feet; symmetric, and wide enough for any difference of two Altitude_Ft values
   type Altitude_Diff_Ft is range -127_900 .. 127_900;

   --  vertical rate in feet per minute, positive up. +-10 000 ft/min is beyond any transport-category aircraft in normal flight
   type Vertical_Rate_Fpm is range -10_000 .. 10_000;


   --  range and bearing

   --  slant range to an intruder, in feet; 150 000 ft (about 24.7 NM) covers the protected volume of TCAS II with margin
   type Range_Ft is range 0 .. 150_000;

   --  rate of change of range in feet per second; negative = closing; +-3 000 ft/s (about 1 775 kt) covers the sum of two fast aircraft
   type Range_Rate_Fps is range -3_000 .. 3_000;

   --  bearing relative to the own aircraft nose, in hundredths of a degree, clockwise, 0 .. 35 999
   type Bearing_Cdeg is range 0 .. 35_999;

   --  time

   --  elapsed time in milliseconds, used for track age and staleness; one hour is far beyond any age the tracker keeps
   type Time_Ms is range 0 .. 3_600_000;

   --  Tau: estimated time to closest approach, in seconds; 120 s is the largest value the logic cares about; the upper value also serves
   --  as the "not closing" sentinel (rule R6)
   type Tau_S is range 0 .. 120;

   No_Tau : constant Tau_S := Tau_S'Last;

   --  sensitivity level

   --  sensitivity level selects the tau and vertical-separation thresholds used by the threat logic; it derives from ownship altitude
   --    SL 1 : standby / no advisories
   --    SL 2 : traffic advisories only
   --    SL 3 .. 7 : traffic advisories and resolution advisories
   --  a failed altitude source falls back to SL 2 (fail-safe, see the schedule in the architecture document, section 4C)
   type Sensitivity_Level is range 1 .. 7;

   SL_Fail_Safe : constant Sensitivity_Level := 2;

   --  track identification

   --  index into the static track table
   subtype Track_Index is Positive range 1 .. Max_Tracks;

   --  Mode S aircraft address, 24 bits; matches the ICAO address field of Mode S downlink formats;
   --  size fixed for the C boundary (rule R5)
   type Mode_S_Address is mod 2 ** 24
     with Size => 24;

   --  threat classification and advisory sense--

   --  how dangerous an intruder currently is; stored as one byte with fixed codes so the display layer and the C code read the same 
   --  values
   type Threat_Class is
     (Other_Traffic,
      Proximate_Traffic,
      Traffic_Advisory,
      Resolution_Advisory);

   for Threat_Class use
     (Other_Traffic       => 0,
      Proximate_Traffic   => 1,
      Traffic_Advisory    => 2,
      Resolution_Advisory => 3);

   for Threat_Class'Size use 8;

   --  direction of a vertical resolution advisory
   type RA_Sense is
     (No_Sense,
      Upward,
      Downward);

   for RA_Sense use
     (No_Sense => 0,
      Upward   => 1,
      Downward => 2);

   for RA_Sense'Size use 8;

end Tcas.Types;