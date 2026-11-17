--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/tracker/tcas-tracker.ads
--  Module      : Intruder track table (specification)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Keeps the list of intruders the system is following: up to Max_Tracks
--    aircraft, each with its last known position and its estimated rates.
--    The Mode S parser (partition 2) feeds it one Measurement per valid
--    reply; the threat detector reads the table and decides what is a
--    threat. This unit decides only WHO is being tracked and HOW FRESH the
--    data is. It does not judge danger.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  No package state. The table is a value the CALLER owns, so every
--        subprogram has Global => null and the unit can be tested and
--        proved on its own (same pattern as the C drivers, rules R1/R2).
--    R2  Fixed capacity. The table is an array of Max_Tracks entries
--        (Tcas.Types): no heap, no growth, no unbounded loop (ASM-004).
--    R3  Every loop is bounded by Max_Tracks, so the cost of every
--        operation has a known upper limit.
--    R4  Rates are derived here, from two successive updates of the same
--        track. The first update has no history, so both rates are 0 and
--        Updates = 1; the threat detector must not act on such a track
--        alone (rule R6 of the security analysis: no RA from an
--        uncorroborated track).
--    R5  Rates are computed in Long_Long_Integer (64 bits, exact), because
--        a vertical-rate product can reach 127 900 x 60 000, above the
--        32-bit limit. The result is saturated to a fixed bound before it
--        returns to Integer, and every conversion into a physical type
--        saturates (Tcas.Units), so no operation can overflow.
--    R6  A full table is not an error: Update reports Table_Full and the
--        table is unchanged. The caller decides what to drop (nearest
--        range first is the policy of the schedule, section 4C).
--    R7  Stale data goes away. Age_Tracks drops every track older than
--        Tcas.Config.Max_Track_Age_Ms, so the logic never acts on old
--        positions.
--
--  ORDER OF EVENTS IN ONE CYCLE (the caller is the scheduler, P3)
--    1 Age_Tracks (Table, elapsed since the previous cycle)
--    2 Update (Table, M, Result) once per valid Mode S measurement
--    3 the threat detector reads the table
--
--  UNITS USED
--    feet (ft), feet per second (ft/s), feet per minute (ft/min),
--    hundredths of a degree (cdeg), milliseconds (ms).
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * The body is in tcas-tracker.adb; GNATprove proves the
--      postconditions and the absence of runtime errors there.
--    * No deviations are requested by this specification.
--  ===========================================================================

pragma SPARK_Mode (On);

with Tcas.Types; use Tcas.Types;

package Tcas.Tracker is

   --  Track and table

   type Track_State is
     (Free,     --  the slot is empty
      Active);  --  the slot holds a track being followed

   --  Number of updates a track has received, saturating at 255. A new
   --  track has 1. The threat detector uses it to require corroboration.
   type Hit_Count is range 0 .. 255;

   type Track is record
      State         : Track_State        := Free;
      Address       : Mode_S_Address     := 0;
      Slant         : Range_Ft           := 0;
      Range_Rate    : Range_Rate_Fps     := 0;
      Bearing       : Bearing_Cdeg       := 0;
      Altitude      : Altitude_Ft        := 0;
      Vertical_Rate : Vertical_Rate_Fpm  := 0;
      Age           : Time_Ms            := 0;  --  since the last update
      Updates       : Hit_Count          := 0;
   end record;

   type Track_Table is array (Track_Index) of Track;

   --  What the parser delivers for one valid reply from one aircraft.
   type Measurement is record
      Address  : Mode_S_Address;
      Slant    : Range_Ft;
      Bearing  : Bearing_Cdeg;
      Altitude : Altitude_Ft;
   end record;

   --  Result of an Update.
   type Update_Result is
     (Inserted,     --  a new track was created
      Updated,      --  an existing track was refreshed
      Table_Full);  --  no free slot: the table is unchanged (rule R6)

   --  Value of Find when the address is not in the table.
   No_Track : constant := 0;

   --  Queries

   --  Number of active tracks.
   function Count (Table : Track_Table) return Natural
   with
     Global => null,
     Post   => Count'Result <= Max_Tracks;

   --  Index of the active track with this address, or No_Track.
   --
   --  The postcondition is two-sided:
   --    * soundness   -- a non-zero result points at a matching Active track;
   --    * completeness -- if no non-zero result is returned, no matching
   --                      Active track exists anywhere in the table.
   --  Completeness is what lets callers (Update in particular) prove that
   --  writing a fresh Active record with a given Address makes a later
   --  Find return non-zero. Without it the prover has to unroll Find's
   --  loop across the package boundary and times out.
   function Find (Table : Track_Table; Address : Mode_S_Address)
     return Natural
   with
     Global => null,
     Post   =>
       Find'Result <= Max_Tracks
       and then
         (if Find'Result /= No_Track then
            Table (Find'Result).State = Active
            and then Table (Find'Result).Address = Address
          else
            (for all I in Track_Index =>
               not (Table (I).State = Active
                    and then Table (I).Address = Address)));

   --  Operations

   --  Empties the table. A zero-filled or uninitialized table is NOT
   --  assumed valid: call this before the first use.
   procedure Init (Table : out Track_Table)
   with
     Global            => null,
     Always_Terminates => True,
     Post              => (for all I in Track_Index => Table (I).State = Free);

   --  Feeds one measurement.
   --    * Address already tracked: the track is refreshed, its rates are
   --      recomputed from the previous position and the time since the
   --      last update, its Age goes to 0, Updates grows (saturating).
   --    * Address not tracked and a slot is free: a track is created with
   --      rates 0, Age 0 and Updates 1.
   --    * Address not tracked and no free slot: Table_Full, no change.
   procedure Update
     (Table  : in out Track_Table;
      M      : Measurement;
      Result : out Update_Result)
   with
     Global            => null,
     Always_Terminates => True,
     Post              =>
       (case Result is
          when Inserted   =>
            Find (Table'Old, M.Address) = No_Track
            and then Find (Table, M.Address) /= No_Track,
          when Updated    =>
            Find (Table'Old, M.Address) /= No_Track
            and then Find (Table, M.Address) /= No_Track,
          when Table_Full =>
            Table = Table'Old
            and then Find (Table, M.Address) = No_Track);

   --  Advances the clock of every active track by Elapsed (saturating at
   --  Time_Ms'Last) and drops each track whose Age then exceeds
   --  Tcas.Config.Max_Track_Age_Ms. Dropped receives how many were
   --  removed. Never creates a track.
   procedure Age_Tracks
     (Table   : in out Track_Table;
      Elapsed : Time_Ms;
      Dropped : out Natural)
   with
     Global            => null,
     Always_Terminates => True,
     Post              =>
       Dropped <= Max_Tracks
       and then
         (for all I in Track_Index =>
            (if Table (I).State = Active
             then Table'Old (I).State = Active));

end Tcas.Tracker;