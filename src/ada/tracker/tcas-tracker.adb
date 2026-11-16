--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/tracker/tcas-tracker.adb
--  Module      : Intruder track table (body)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P3 TCAS_CORE
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Implements the operations declared in tcas-tracker.ads. The
--    specification holds the contract (rules, order of events, meaning of
--    each result); this body holds the code that makes it true. Read the
--    specification first.
--
--  DESIGN RULES (as in the specification, with the way each is met here)
--    R1  No state: every subprogram works only on its parameters.
--    R2  Fixed capacity: the only loops run over Track_Index, 1 .. 60.
--    R3  Every loop has a loop invariant that pins down what the loop has
--        done so far, so the postcondition follows after the loop.
--    R4  The first update of a track gives rates of 0. A later update
--        derives the rates from the position change and from Age, the time
--        since the previous update. If Age is 0 (two updates with no time
--        between them) the previous rates are kept: dividing by zero time
--        would mean nothing.
--    R5  Rate computes in 64 bits (Long_Long_Integer). Its inputs are
--        restricted by subtypes (Difference, Factor) so that the product
--        is below 1.2E10, and its output is saturated to +-1 000 000
--        before it comes back to Integer. The result then goes through
--        Tcas.Units.Clamp_* into the physical type.
--    R6  A full table is not an error: Update reports Table_Full and does
--        not touch the table (the Table_Full branch has no assignment).
--    R7  Age_Tracks adds Elapsed with saturation and empties every slot
--        whose Age then exceeds Tcas.Config.Max_Track_Age_Ms.
--
--  RATE FORMULAS (truncation toward zero, as in Tcas.Units)
--    Range_Rate    (ft/s)    = (New_Slant - Old_Slant) * 1 000   / Age_Ms
--    Vertical_Rate (ft/min)  = (New_Alt   - Old_Alt)   * 60 000  / Age_Ms
--  Largest products: 150 000 * 1 000 = 1.5E8 and 127 900 * 60 000 = 7.7E9.
--  The second one is above the 32-bit limit, which is why Rate uses 64 bits.
--  [VERIFY] WCET of the 64-bit division on the target (the Cortex-R52 has
--  a hardware 32-bit divider; the 64-bit one runs in the runtime library).
--
--  STRUCTURE OF THE CODE
--    Local helpers come first (Rate, Find_Free, Refresh), then the
--    operations of the specification in the order Count, Find, Init,
--    Update, Age_Tracks. Every function has ONE exit (a single return at
--    the end). Loops have no early exit: the first match is kept by a
--    guard in the loop body, not by leaving the loop, so each loop always
--    runs 60 times and its cost is constant.
--
--  PROOF OBLIGATIONS (discharged by GNATprove, tcas2.gpr level 2)
--    * Absence of runtime errors: index checks (Found is in 1 .. 60 where
--      it is used as an index), range checks of every conversion, division
--      checks in Rate, overflow checks of the 64-bit product.
--    * The postconditions of Count, Find (including completeness), Init,
--      Update and Age_Tracks.
--    * The loop invariants, at entry and after each iteration.
--    * Termination of Refresh (Always_Terminates aspect, see local spec).
--
--  SPARK NOTES
--    * SPARK_Mode is On.
--    * No deviations are requested by this body.
--  ===========================================================================

pragma SPARK_Mode (On);

with Tcas.Config;
with Tcas.Units; use Tcas.Units;

package body Tcas.Tracker is
   --  A slot with nothing in it. Used by Init and by Age_Tracks.
   Empty_Track : constant Track :=
     (State         => Free,
      Address       => 0,
      Slant         => 0,
      Range_Rate    => 0,
      Bearing       => 0,
      Altitude      => 0,
      Vertical_Rate => 0,
      Age           => 0,
      Updates       => 0);

   --  ------------------------------------------------------------------
   --  Rate helper (local to this body)
   --  ------------------------------------------------------------------

   --  Largest absolute change between two updates: Slant gives +-150 000
   --  and Altitude gives +-127 900; the subtype leaves some margin.
   subtype Difference is Integer range -200_000 .. 200_000;

   --  The two scale factors in use are 1 000 and 60 000.
   subtype Factor is Positive range 1 .. 60_000;

   --  Saturation bound of the 64-bit result, well inside Integer.
   Rate_Bound : constant := 1_000_000;

   function Rate
     (Diff   : Difference;
      Scale  : Factor;
      Age_Ms : Positive) return Integer
   with
     Global => null,
     Post   => Rate'Result in -Rate_Bound .. Rate_Bound;

   function Rate
     (Diff   : Difference;
      Scale  : Factor;
      Age_Ms : Positive) return Integer
   is
      Q : constant Long_Long_Integer :=
        (Long_Long_Integer (Diff) * Long_Long_Integer (Scale))
        / Long_Long_Integer (Age_Ms);
   begin
      return (if Q < -Long_Long_Integer (Rate_Bound) then -Rate_Bound
              elsif Q > Long_Long_Integer (Rate_Bound) then Rate_Bound
              else Integer (Q));
   end Rate;

   --  ------------------------------------------------------------------
   --  Free-slot search (local to this body)
   --  ------------------------------------------------------------------

   --  Index of the first Free slot, or No_Track when the table is full.
   function Find_Free (Table : Track_Table) return Natural
   with
     Global => null,
     Post   =>
       Find_Free'Result <= Max_Tracks
       and then
         (if Find_Free'Result /= No_Track
          then Table (Find_Free'Result).State = Free);

   function Find_Free (Table : Track_Table) return Natural is
      Found : Natural := No_Track;
   begin
      for I in Track_Index loop
         pragma Loop_Invariant (Found <= Max_Tracks);
         pragma Loop_Invariant
           (if Found /= No_Track then Table (Found).State = Free);

         if Found = No_Track and then Table (I).State = Free then
            Found := I;
         end if;
      end loop;

      return Found;
   end Find_Free;

   --  ------------------------------------------------------------------
   --  Refresh of an existing track (local to this body)
   --  ------------------------------------------------------------------

   --  Updates one active track with a new measurement of the same
   --  aircraft: new position, new rates, Age back to 0, one more hit.
   procedure Refresh (Item : in out Track; M : Measurement)
   with
     Global           => null,
     Always_Terminates,
     Pre              => Item.State = Active and then Item.Address = M.Address,
     Post             => Item.State = Active and then Item.Address = M.Address;

   procedure Refresh (Item : in out Track; M : Measurement) is
      New_Range_Rate    : Range_Rate_Fps    := Item.Range_Rate;
      New_Vertical_Rate : Vertical_Rate_Fpm := Item.Vertical_Rate;
      New_Updates       : Hit_Count         := Item.Updates;
   begin
      --  rule R4: rates need a time interval; with none, keep the old ones
      if Item.Age > 0 then
         New_Range_Rate :=
           Clamp_Range_Rate
             (Rate (Integer (M.Slant) - Integer (Item.Slant),
                    1_000,
                    Positive (Item.Age)));
         New_Vertical_Rate :=
           Clamp_Vertical_Rate
             (Rate (Integer (M.Altitude) - Integer (Item.Altitude),
                    60_000,
                    Positive (Item.Age)));
      end if;

      --  the hit count saturates and never wraps
      if Item.Updates < Hit_Count'Last then
         New_Updates := Item.Updates + 1;
      end if;

      Item :=
        (State         => Active,
         Address       => Item.Address,
         Slant         => M.Slant,
         Range_Rate    => New_Range_Rate,
         Bearing       => M.Bearing,
         Altitude      => M.Altitude,
         Vertical_Rate => New_Vertical_Rate,
         Age           => 0,
         Updates       => New_Updates);
   end Refresh;

   --  ------------------------------------------------------------------
   --  Queries
   --  ------------------------------------------------------------------

   function Count (Table : Track_Table) return Natural is
      N : Natural := 0;
   begin
      for I in Track_Index loop
         pragma Loop_Invariant (N <= I - 1);

         if Table (I).State = Active then
            N := N + 1;
         end if;
      end loop;

      return N;
   end Count;

   function Find (Table : Track_Table; Address : Mode_S_Address)
     return Natural
   is
      Found : Natural := No_Track;
   begin
      for I in Track_Index loop
         pragma Loop_Invariant (Found <= Max_Tracks);
         pragma Loop_Invariant
           (if Found /= No_Track
            then Table (Found).State = Active
                 and then Table (Found).Address = Address
            else
              (for all J in Track_Index range 1 .. I - 1 =>
                 not (Table (J).State = Active
                      and then Table (J).Address = Address)));

         --  the first match is kept; later ones cannot replace it
         if Found = No_Track
           and then Table (I).State = Active
           and then Table (I).Address = Address
         then
            Found := I;
         end if;
      end loop;

      return Found;
   end Find;

   --  ------------------------------------------------------------------
   --  Operations
   --  ------------------------------------------------------------------

   procedure Init (Table : out Track_Table) is
   begin
      Table := (others => Empty_Track);
   end Init;

   procedure Update
     (Table  : in out Track_Table;
      M      : Measurement;
      Result : out Update_Result)
   is
      Idx : constant Natural := Find (Table, M.Address);
   begin
      if Idx /= No_Track then
         --  the aircraft is already tracked: refresh its track
         Refresh (Table (Idx), M);
         Result := Updated;
      else
         declare
            Slot : constant Natural := Find_Free (Table);
         begin
            if Slot /= No_Track then
               --  a new aircraft and a free slot: start a track
               Table (Slot) :=
                 (State         => Active,
                  Address       => M.Address,
                  Slant         => M.Slant,
                  Range_Rate    => 0,
                  Bearing       => M.Bearing,
                  Altitude      => M.Altitude,
                  Vertical_Rate => 0,
                  Age           => 0,
                  Updates       => 1);
               Result := Inserted;
            else
               --  a new aircraft and no free slot: nothing changes (R6)
               Result := Table_Full;
            end if;
         end;
      end if;
   end Update;

   procedure Age_Tracks
     (Table   : in out Track_Table;
      Elapsed : Time_Ms;
      Dropped : out Natural)
   is
   begin
      Dropped := 0;

      for I in Track_Index loop
         pragma Loop_Invariant (Dropped <= I - 1);
         --  no slot becomes Active in this loop (it never creates a track)
         pragma Loop_Invariant
           (for all J in Track_Index =>
              (if Table (J).State = Active
               then Table'Loop_Entry (J).State = Active));

         if Table (I).State = Active then
            Table (I).Age :=
              Clamp_Time (Integer (Table (I).Age) + Integer (Elapsed));

            if Table (I).Age > Tcas.Config.Max_Track_Age_Ms then
               Table (I) := Empty_Track;
               Dropped := Dropped + 1;
            end if;
         end if;
      end loop;
   end Age_Tracks;

end Tcas.Tracker;