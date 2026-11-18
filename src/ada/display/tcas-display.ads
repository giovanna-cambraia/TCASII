--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/display/tcas-display.ads
--  Module      : Display and audio formatter (specification)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : P4 DISPLAY_AUDIO
--  Requirements: REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    Turns what one scheduler cycle produced (Tcas.Scheduler.Cycle_Output,
--    the track table and the ownship data) into one
--    Tcas.Display_Port.Display_Frame: the mode, the advisory, the audio
--    phrase and the selected intruder. The C driver (the future
--    tcas_display_drv.h) puts the frame on the display and audio lines.
--
--    This unit decides WHAT is shown and said. It does not decide whether
--    there is a threat or which way to maneuver: that was already decided
--    by the threat logic and the RA life cycle, and arrives in the cycle
--    output.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  One small state (Formatter_State), owned by the CALLER, as in
--        Tcas.Tracker and Tcas.RA_State. It holds the class shown in the
--        previous cycle, so that "Traffic, traffic" plays at the onset of
--        a traffic advisory and not every cycle. Every subprogram has
--        Global => null.
--    R2  The frame always passes Display_Port.Is_Well_Formed. This is a
--        postcondition of Format, so a formatter bug cannot put an
--        inconsistent advisory on the cockpit display: the proof fails
--        first.
--    R3  The mode follows the ADVISORY first and the level second. While
--        an RA is held the level can already have fallen to SL 2 (the
--        pressure altitude was lost, see test_scheduler.adb). The port
--        requires the TA/RA mode whenever an advisory is present, so the
--        advisory wins.
--    R4  The audio phrase agrees with the advisory (rule R7 of the port),
--        and it is chosen by event, not by state, so a held advisory is
--        not announced again every cycle.
--    R5  Fail closed. A cycle without a selected intruder publishes no
--        intruder data at all: range 0, relative altitude 0, bearing 0,
--        class "other".
--    R6  Constant cost: no loop. Every value is computed from the inputs
--        of this cycle and the one remembered class.
--    R7  No number in this unit apart from the codes of the port. Every
--        threshold lives in Tcas.Config, and the advisory rate is read
--        from the advisory itself.
--
--  ORDER OF DECISIONS IN Audio_Of (the first match wins)
--    1 event is Cleared                                "Clear of conflict"
--    2 event is Reversed, Climb                        "Climb, climb now"
--    3 event is Reversed, Descend                      "Descend, descend now"
--    4 event is Issued, Climb                          "Climb, climb"
--    5 event is Issued, Descend                        "Descend, descend"
--    6 event is No_Event, Maintained or Clearing, the
--      selected intruder is a traffic advisory now and
--      the previous class was below a traffic advisory "Traffic, traffic"
--    7 anything else                                   silence
--
--  MODE (Mode_Of)
--    an advisory is present (rule R3)        TA and RA
--    level 3 or more                         TA and RA
--    level 2                                 TA only
--    level 1                                 standby
--  Select_Level never returns 1 (Tcas.Sensitivity), so standby does not
--  occur in a real cycle; Mode_Of still covers it.
--
--  CALLING ORDER (the caller is the cyclic executive, after Run_Cycle)
--    Init (S)                                   once, at start-up
--    Format (S, Own, Table, Output, Frame)      once per cycle
--  Own is the same Ownship_Data that was given to Run_Cycle, so the
--  relative altitude uses the same ownship altitude as the threat logic.
--
--  KNOWN SIMPLIFICATIONS (open items for v2)
--    Only the selected intruder is shown; the full traffic list for the
--    display is not built. The frame layout is a placeholder until the
--    display and audio interface control document exists. An RA phrase is
--    spoken once, at the event; real systems repeat it while the advisory
--    lasts. If the selected aircraft changes between two traffic
--    advisories, "Traffic, traffic" is not repeated. If the pressure
--    altitude is not valid, the relative altitude is computed against the
--    0 ft that the decoder publishes (the FINDING of
--    tests/integration/test_integration.adb).
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * The body is in tcas-display.adb; GNATprove proves the
--      postconditions and the absence of runtime errors there.
--    * No deviations are requested by this specification.

pragma SPARK_Mode (On);

with Tcas.Config;
with Tcas.Display_Port;
with Tcas.RA;
with Tcas.RA_State;
with Tcas.Scheduler;
with Tcas.Tracker;
with Tcas.Types; use Tcas.Types;

package Tcas.Display is

   package Dp renames Tcas.Display_Port;
   package Rg renames Tcas.RA;
   package Rs renames Tcas.RA_State;
   package Sc renames Tcas.Scheduler;
   package Tr renames Tcas.Tracker;

   use type Dp.U8;
   use type Rg.RA_Kind;
   use type Rs.RA_Event;
   use type Tr.Track_State;

   --  State

   --  What the formatter remembers: the class shown in the previous cycle
   --  (Other_Traffic when no intruder was shown).
   type Formatter_State is record
      Prev_Class : Threat_Class := Other_Traffic;
   end record;

   --  Checks on the inputs (public so that the tests can use them)

   --  True when the advisory is one the RA life cycle can produce: none,
   --  or a Climb at +target rate, or a Descend at -target rate.
   function Advisory_Ok (A : Rg.Advisory) return Boolean is
     ((A.Kind = Rg.No_Advisory
       and then A.Sense = No_Sense
       and then A.Target_Rate = 0)
      or else
        (A.Kind = Rg.Climb
         and then A.Sense = Upward
         and then A.Target_Rate = Tcas.Config.RA_Target_Rate_Fpm)
      or else
        (A.Kind = Rg.Descend
         and then A.Sense = Downward
         and then A.Target_Rate = -Tcas.Config.RA_Target_Rate_Fpm))
   with Global => null;

   --  True when a cycle output is internally consistent: a real level, a
   --  valid advisory, a selected index in range, and an event that agrees
   --  with the advisory (Issued, Maintained, Reversed and Clearing have
   --  one; Cleared and No_Event do not). Tcas.Scheduler.Run_Cycle
   --  produces such outputs (its postcondition and that of
   --  Tcas.RA_State.Step).
   function Output_Ok (Output : Sc.Cycle_Output) return Boolean is
     (Output.Level >= SL_Fail_Safe
      and then Advisory_Ok (Output.Adv)
      and then Output.Selected <= Max_Tracks
      and then
        (if Output.Event in
              Rs.Issued | Rs.Maintained | Rs.Reversed | Rs.Clearing
         then Output.Adv.Kind /= Rg.No_Advisory
         else Output.Adv.Kind = Rg.No_Advisory))
   with Global => null;

   --  Building blocks (public so that the tests can check them alone)

   --  The display mode (see MODE).
   function Mode_Of
     (Level : Sensitivity_Level;
      Adv   : Rg.Advisory) return Dp.U8
   with
     Global => null,
     Post   =>
       Mode_Of'Result <= Dp.Mode_TA_RA
       and then
         (if Adv.Kind /= Rg.No_Advisory
          then Mode_Of'Result = Dp.Mode_TA_RA);

   --  The audio phrase (see ORDER OF DECISIONS). Class and Shown say what
   --  is displayed this cycle: Shown is False when no intruder is
   --  selected, and then Class is not used. Prev_Class is the class of the
   --  previous cycle.
   function Audio_Of
     (Event      : Rs.RA_Event;
      Adv        : Rg.Advisory;
      Shown      : Boolean;
      Class      : Threat_Class;
      Prev_Class : Threat_Class) return Dp.U8
   with
     Global => null,
     Pre    =>
       Advisory_Ok (Adv)
       and then (if Event in Rs.Issued | Rs.Reversed
                 then Adv.Kind /= Rg.No_Advisory)
       and then (if Event = Rs.Cleared
                 then Adv.Kind = Rg.No_Advisory),
     Post   =>
       Audio_Of'Result <= Dp.Audio_Clear_Of_Conflict
       --  a phrase never contradicts the advisory (rule R4)
       and then
         (if Audio_Of'Result = Dp.Audio_Climb
             or else Audio_Of'Result = Dp.Audio_Climb_Now
          then Adv.Kind = Rg.Climb)
       and then
         (if Audio_Of'Result = Dp.Audio_Descend
             or else Audio_Of'Result = Dp.Audio_Descend_Now
          then Adv.Kind = Rg.Descend)
       and then
         (if Audio_Of'Result = Dp.Audio_Clear_Of_Conflict
          then Event = Rs.Cleared and then Adv.Kind = Rg.No_Advisory)
       --  "Traffic, traffic" needs a shown traffic advisory
       and then
         (if Audio_Of'Result = Dp.Audio_Traffic
          then Shown and then Class = Traffic_Advisory)
       --  the event decides
       and then
         (if Event = Rs.Cleared
          then Audio_Of'Result = Dp.Audio_Clear_Of_Conflict)
       and then
         (if Event = Rs.Issued and then Adv.Kind = Rg.Climb
          then Audio_Of'Result = Dp.Audio_Climb)
       and then
         (if Event = Rs.Issued and then Adv.Kind = Rg.Descend
          then Audio_Of'Result = Dp.Audio_Descend)
       and then
         (if Event = Rs.Reversed and then Adv.Kind = Rg.Climb
          then Audio_Of'Result = Dp.Audio_Climb_Now)
       and then
         (if Event = Rs.Reversed and then Adv.Kind = Rg.Descend
          then Audio_Of'Result = Dp.Audio_Descend_Now)
       --  a held advisory is not announced again (rule R4)
       and then
         (if Event = Rs.Maintained or else Event = Rs.Clearing
          then Audio_Of'Result = Dp.Audio_None
               or else Audio_Of'Result = Dp.Audio_Traffic);

   --  Operations

   --  Nothing was shown before. Call before the first Format.
   procedure Init (S : out Formatter_State)
   with
     Global => null,
     Post   => S.Prev_Class = Other_Traffic;

   --  Fills the frame for one cycle and remembers the class shown.
   --    Own    : the ownship data that was given to Run_Cycle
   --    Table  : the track table after Run_Cycle
   --    Output : what Run_Cycle reported
   procedure Format
     (S      : in out Formatter_State;
      Own    : Sc.Ownship_Data;
      Table  : Tr.Track_Table;
      Output : Sc.Cycle_Output;
      Frame  : out Dp.Display_Frame)
   with
     Global => null,
     Pre    =>
       Output_Ok (Output)
       and then
         (if Output.Selected /= Tr.No_Track
          then Table (Output.Selected).State = Tr.Active),
     Post   =>
       --  rule R2: the frame can always be transmitted
       Dp.Is_Well_Formed (Frame)
       --  the mode follows the advisory and the level (rule R3)
       and then Frame.Mode = Mode_Of (Output.Level, Output.Adv)
       --  the advisory on the display is the one from the cycle
       and then
         (Frame.Advisory = Dp.Advisory_Climb)
           = (Output.Adv.Kind = Rg.Climb)
       and then
         (Frame.Advisory = Dp.Advisory_Descend)
           = (Output.Adv.Kind = Rg.Descend)
       --  an intruder is shown exactly when one is selected (rule R5)
       and then
         (Frame.Intruder_Valid = 1) = (Output.Selected /= Tr.No_Track)
       --  what is remembered is what was shown
       and then
         S.Prev_Class =
           (if Output.Selected /= Tr.No_Track
            then Output.Selected_Class
            else Other_Traffic);

end Tcas.Display;