--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/common/config/tcas-config.ads
--  Module      : Configuration constants and threshold tables (spec only)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : shared by P2, P3 and P4 (pure, no state, no code)
--  Requirements: REQ-003 (RA logic), REQ-002 (no runtime errors),
--                placeholder IDs
--
--  PURPOSE
--    This package is the single home of every number that tunes the system:
--    how long a cycle is, how many tracks and replies fit, how old data may
--    be, and the threshold table that decides when an intruder becomes a
--    traffic advisory (TA) or a resolution advisory (RA).
--
--    No other unit may contain a threshold literal. If a value is needed
--    by the logic and is not here, it is added here first. This makes the
--    mapping from requirement to number a single lookup.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure package: no state, no body, no elaboration code. All values
--        are static, known at compile time.
--    R2  Every value has a name, a unit in the name or the comment, and a
--        source. Values taken from a standard cite the table.
--    R3  Thresholds are stored in the same types the logic uses
--        (Tcas.Types), so no conversion is done at the point of use.
--    R4  The threshold table has one row per sensitivity level, indexed by
--        Sensitivity_Level. There is no row that can be missing: the array
--        bounds are the type's range, and the compiler rejects an aggregate
--        with a gap.
--    R5  Levels without a function carry the value 0 (no RA at SL 1 and 2).
--        "Zero" always means "this threshold never triggers".
--    R6  The compile-time check ties the schedule windows to the cycle
--        length, so an edit that breaks the budget does not compile.
--
--  [VERIFY] MARKER
--    Values marked [VERIFY] are transcribed from the DO-185B (TCAS II
--    v7.1) sensitivity level tables and must be checked line by line
--    against the controlled copy of the standard before they are used for
--    anything but development. They are placeholders for certification
--    purposes (open item for v2).
--
--  UNITS USED
--    seconds (s), milliseconds (ms), feet (ft). DMOD is the distance
--    modification in feet (1 NM = 6 076 ft, rounded to whole feet).
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * No deviations are requested by this specification.
--  ===========================================================================

pragma SPARK_Mode (On);

with Tcas.Types; use Tcas.Types;

package Tcas.Config
  with Pure
is

   --  Schedule (architecture document, section 4C)

   --  Each window is the length of the slot in the 1 000 ms cycle, not the
   --  worst-case execution time. The budgets are allocated by the
   --  scheduler (src/ada/scheduler).

   Window_Ownship_Ms     : constant := 150;  --  000 .. 150  ownship ingestion
   Window_Interrogate_Ms : constant := 300;  --  150 .. 450  Mode S traffic
   Window_Threat_Ms      : constant := 300;  --  450 .. 750  tracks and threats
   Window_Advisory_Ms    : constant := 150; --  750 .. 900  RA and coordination
   Window_Output_Ms      : constant := 100; --  900 .. 1000 display, audio, BIT

   --  The five windows must fill the cycle exactly (rule R6). If the sum
   --  differs from Cycle_Ms, compilation stops with the message below.
   pragma Compile_Time_Error
     (Window_Ownship_Ms + Window_Interrogate_Ms + Window_Threat_Ms
        + Window_Advisory_Ms + Window_Output_Ms /= Cycle_Ms,
      "Schedule windows do not add up to Cycle_Ms");

   --  Capacity and bounds

   --  Largest number of Mode S replies processed in one cycle. The rest
   --  are dropped by priority (nearest range first): denial-of-service
   --  limit of the security analysis.
   Max_Replies_Per_Cycle : constant := 256;

   --  Largest number of coordination attempts in the advisory window.
   Max_Coord_Retries : constant := 3;

   --  Data freshness

   --  Ownship sensor data older than this is stale (ASM / section 4C).
   Max_Sensor_Age_Ms : constant Time_Ms := 200;

   --  A track not updated for this long is dropped by the tracker.
   --  [VERIFY] DO-185B drops a track after a number of missed
   --  interrogations rather than a time; this is a development value.
   Max_Track_Age_Ms : constant Time_Ms := 5_000;

   --  Threat thresholds per sensitivity level

   --  One row of the table. A zero in an RA field means no RA is issued at
   --  that level (rule R5).
   type Level_Thresholds is record
      TA_Tau  : Tau_S;             --  traffic advisory tau, seconds
      RA_Tau  : Tau_S;             --  resolution advisory tau, seconds
      TA_DMOD : Range_Ft;          --  TA distance modification, feet
      RA_DMOD : Range_Ft;          --  RA distance modification, feet
      TA_ZTHR : Altitude_Diff_Ft;  --  TA vertical threshold, feet
      RA_ZTHR : Altitude_Diff_Ft;  --  RA vertical threshold, feet
      RA_ALIM : Altitude_Diff_Ft;  --  RA vertical separation goal, feet
   end record;

   --  [VERIFY] every row, against DO-185B Tables (sensitivity level
   --  definition and the TA/RA threshold tables for TCAS II v7.1).
   --  Conversions: 0.30 NM = 1 823 ft, 0.33 NM = 2 005 ft,
   --  0.48 NM = 2 917 ft, 0.75 NM = 4 557 ft, 1.00 NM = 6 076 ft,
   --  0.20 NM = 1 215 ft, 0.35 NM = 2 127 ft, 0.55 NM = 3 342 ft,
   --  0.80 NM = 4 861 ft, 1.10 NM = 6 684 ft.
   Thresholds : constant array (Sensitivity_Level) of Level_Thresholds :=
     (1 => (TA_Tau => 0,  RA_Tau => 0,
            TA_DMOD => 0, RA_DMOD => 0,
            TA_ZTHR => 0, RA_ZTHR => 0, RA_ALIM => 0),
      2 => (TA_Tau => 20, RA_Tau => 0,
            TA_DMOD => 1_823, RA_DMOD => 0,
            TA_ZTHR => 850, RA_ZTHR => 0, RA_ALIM => 0),
      3 => (TA_Tau => 25, RA_Tau => 15,
            TA_DMOD => 2_005, RA_DMOD => 1_215,
            TA_ZTHR => 850, RA_ZTHR => 600, RA_ALIM => 300),
      4 => (TA_Tau => 30, RA_Tau => 20,
            TA_DMOD => 2_917, RA_DMOD => 2_127,
            TA_ZTHR => 850, RA_ZTHR => 600, RA_ALIM => 300),
      5 => (TA_Tau => 40, RA_Tau => 25,
            TA_DMOD => 4_557, RA_DMOD => 3_342,
            TA_ZTHR => 850, RA_ZTHR => 600, RA_ALIM => 350),
      6 => (TA_Tau => 45, RA_Tau => 30,
            TA_DMOD => 6_076, RA_DMOD => 4_861,
            TA_ZTHR => 850, RA_ZTHR => 600, RA_ALIM => 400),
      7 => (TA_Tau => 48, RA_Tau => 35,
            TA_DMOD => 6_076, RA_DMOD => 6_684,
            TA_ZTHR => 1_200, RA_ZTHR => 700, RA_ALIM => 600));

   --  Sensitivity level selection

   --  The ownship altitude selects the level. Below the first boundary the
   --  radio altitude (height above ground) is used; above it the pressure
   --  altitude. Each value is the UPPER limit of the level named, in feet.
   --  [VERIFY] boundaries and hysteresis against DO-185B.

   SL2_Upper_Ft : constant Altitude_Ft := 1_000;   --  radio altitude
   SL3_Upper_Ft : constant Altitude_Ft := 2_350;   --  radio altitude
   SL4_Upper_Ft : constant Altitude_Ft := 5_000;   --  pressure altitude
   SL5_Upper_Ft : constant Altitude_Ft := 10_000;  --  pressure altitude
   SL6_Upper_Ft : constant Altitude_Ft := 20_000;  --  pressure altitude
   --  Above SL6_Upper_Ft the level is 7.

end Tcas.Config;