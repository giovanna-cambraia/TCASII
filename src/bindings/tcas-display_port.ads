--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/bindings/tcas-display_port.ads
--  Module      : Ada view of the C display and audio frame (spec only)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : boundary between P4 (SPARK formatter) and P1 (C driver)
--  Requirements: REQ-001, REQ-002, REQ-004 (placeholder IDs)
--
--  PURPOSE
--    This is the OUTPUT twin of Tcas.A429_Port and Tcas.Modes_Port. The
--    SPARK formatter (partition 4) fills one Display_Frame per cycle with
--    the state of the system: the mode, the advisory, the audio phrase and
--    the selected intruder. The C driver (the future tcas_display_drv.h)
--    reads the frame in place, without a copy, and puts it on the display
--    and audio lines.
--
--    The input ports check what arrives from outside. This one checks what
--    leaves: Is_Well_Formed states every promise a frame must keep before
--    the C driver may transmit it, so a formatter bug cannot put an
--    inconsistent advisory on the cockpit display.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure package: no state, no body. Constants, one record type and
--        one expression function.
--    R2  The layout is the C layout, byte for byte: Convention C, an
--        explicit representation clause with byte offsets, and a size
--        check that stops the compilation if the record is not 20 bytes.
--    R3  Bytes stay bytes. Mode, Advisory, Audio, Intruder_Class and
--        Intruder_Valid are Unsigned_8, not enumeration types, so the
--        layout shared with C is fixed and Is_Well_Formed can check every
--        value (the same rule as the two input ports).
--    R4  Every code has the value of its C counterpart, and the comment
--        names it. A cross-check test (to be written, like
--        check_modes_layout.c) compares both sides.
--    R5  Is_Well_Formed has Global => null and no side effects, so it can
--        be used in the postcondition of the formatter and in a
--        precondition of the C-facing code.
--    R6  FAIL CLOSED. A frame without an intruder carries NOTHING about
--        one: range 0, relative altitude 0, bearing 0, class 0. A garbage
--        value is never published, even by accident.
--    R7  The audio phrase must agree with the advisory. "Climb" audio
--        needs a Climb advisory, "Descend" audio needs a Descend
--        advisory, "Clear of conflict" needs no advisory, and "Traffic"
--        needs a selected traffic advisory. A silent mismatch between
--        what the pilot sees and what the pilot hears is the failure this
--        rule exists to prevent.
--
--  LAYOUT (must equal tcas_display_frame_t in tcas_display_drv.h, 20 bytes)
--    offset  size  field
--       0      4   Slant_Ft         selected intruder, feet, 0 when none
--       4      4   Rel_Alt_Ft       intruder minus ownship, feet (signed)
--       8      2   Bearing_Cdeg     hundredths of a degree, 0 .. 35 999
--      10      2   Target_Rate_Fpm  RA target rate (signed), 0 when none
--      12      1   Mode             0 standby, 1 TA only, 2 TA and RA
--      13      1   Advisory         0 none, 1 climb, 2 descend
--      14      1   Audio            0 .. 6, see AUDIO CODES
--      15      1   Intruder_Class   0 other .. 3 resolution advisory
--      16      1   Intruder_Valid   1 an intruder is selected, 0 none
--      17      1   Reserved8        always 0
--      18      2   Reserved16       always 0
--    [VERIFY] The layout is a PLACEHOLDER until the display and audio
--    interface control document exists. The offsets assume 4-byte
--    alignment of the 32-bit fields and no padding, true for the
--    Cortex-R52 and x86_64 hosts.
--
--  AUDIO CODES
--    0 none                      4 "Climb, climb now"   (reversal to climb)
--    1 "Traffic, traffic"        5 "Descend, descend now" (reversal)
--    2 "Climb, climb"            6 "Clear of conflict"
--    3 "Descend, descend"
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * No deviations are requested by this specification.

pragma SPARK_Mode (On);

with Interfaces;

package Tcas.Display_Port
  with Pure
is

   subtype U8  is Interfaces.Unsigned_8;
   subtype U16 is Interfaces.Unsigned_16;
   subtype U32 is Interfaces.Unsigned_32;
   subtype I16 is Interfaces.Integer_16;
   subtype I32 is Interfaces.Integer_32;

   use type U8;
   use type U16;
   use type U32;
   use type I16;
   use type I32;

   --  Mode (TCAS_DISPLAY_MODE_* in the C header)

   Mode_Standby : constant U8 := 0;   --  no advisories
   Mode_TA_Only : constant U8 := 1;   --  traffic advisories only
   Mode_TA_RA   : constant U8 := 2;   --  traffic and resolution advisories

   --  Advisory (TCAS_DISPLAY_ADV_* in the C header)

   Advisory_None    : constant U8 := 0;
   Advisory_Climb   : constant U8 := 1;
   Advisory_Descend : constant U8 := 2;

   --  Audio (TCAS_DISPLAY_AUDIO_* in the C header, see AUDIO CODES)

   Audio_None             : constant U8 := 0;
   Audio_Traffic          : constant U8 := 1;
   Audio_Climb            : constant U8 := 2;
   Audio_Descend          : constant U8 := 3;
   Audio_Climb_Now        : constant U8 := 4;
   Audio_Descend_Now      : constant U8 := 5;
   Audio_Clear_Of_Conflict : constant U8 := 6;

   --  Intruder class (TCAS_DISPLAY_CLASS_* in the C header; the values
   --  of Tcas.Types.Threat_Class)

   Class_Other      : constant U8 := 0;
   Class_Proximate  : constant U8 := 1;
   Class_Traffic    : constant U8 := 2;   --  traffic advisory
   Class_Resolution : constant U8 := 3;   --  resolution advisory

   --  Limits

   --  Largest slant range: Tcas.Types.Range_Ft'Last.
   Slant_Max : constant U32 := 150_000;

   --  Largest bearing: 359.99 degrees (Tcas.Types.Bearing_Cdeg'Last).
   Bearing_Max : constant U16 := 35_999;

   --  Largest vertical difference: Tcas.Types.Altitude_Diff_Ft'Last.
   Alt_Diff_Max : constant I32 := 127_900;

   --  Largest RA target rate: Tcas.Types.Vertical_Rate_Fpm'Last.
   Rate_Max : constant I16 := 10_000;

   --  The frame (tcas_display_frame_t in the C header)

   type Display_Frame is record
      Slant_Ft        : U32;
      Rel_Alt_Ft      : I32;
      Bearing_Cdeg    : U16;
      Target_Rate_Fpm : I16;
      Mode            : U8;
      Advisory        : U8;
      Audio           : U8;
      Intruder_Class  : U8;
      Intruder_Valid  : U8;
      Reserved8       : U8;
      Reserved16      : U16;
   end record
     with Convention => C;

   for Display_Frame use record
      Slant_Ft        at  0 range 0 .. 31;
      Rel_Alt_Ft      at  4 range 0 .. 31;
      Bearing_Cdeg    at  8 range 0 .. 15;
      Target_Rate_Fpm at 10 range 0 .. 15;
      Mode            at 12 range 0 .. 7;
      Advisory        at 13 range 0 .. 7;
      Audio           at 14 range 0 .. 7;
      Intruder_Class  at 15 range 0 .. 7;
      Intruder_Valid  at 16 range 0 .. 7;
      Reserved8       at 17 range 0 .. 7;
      Reserved16      at 18 range 0 .. 15;
   end record;

   for Display_Frame'Size use 160;

   --  The record must be exactly 20 bytes (rule R2). If it is not,
   --  compilation stops with the message below.
   pragma Compile_Time_Error
     (Display_Frame'Size /= 160,
      "Display_Frame must be 20 bytes, as in the C header");

   --  Output check (rule R5)

   --  True when the frame keeps every promise made before transmission:
   --    * the reserved fields are 0 and every code is in its range
   --    * with an intruder: range, bearing and relative altitude are
   --      inside their limits
   --    * without an intruder (rule R6): range, relative altitude,
   --      bearing and class are all 0
   --    * the target rate agrees with the advisory: 0 for none, positive
   --      for a Climb, negative for a Descend, never beyond Rate_Max
   --    * an advisory needs the TA/RA mode
   --    * the audio phrase agrees with the advisory (rule R7)
   function Is_Well_Formed (F : Display_Frame) return Boolean is
     (F.Reserved8 = 0
      and then F.Reserved16 = 0
      and then F.Mode <= Mode_TA_RA
      and then F.Advisory <= Advisory_Descend
      and then F.Audio <= Audio_Clear_Of_Conflict
      and then F.Intruder_Class <= Class_Resolution
      and then F.Intruder_Valid <= 1
      and then
        (if F.Intruder_Valid = 1
         then F.Slant_Ft <= Slant_Max
              and then F.Bearing_Cdeg <= Bearing_Max
              and then F.Rel_Alt_Ft in -Alt_Diff_Max .. Alt_Diff_Max
         else F.Slant_Ft = 0
              and then F.Rel_Alt_Ft = 0
              and then F.Bearing_Cdeg = 0
              and then F.Intruder_Class = Class_Other)
      and then F.Target_Rate_Fpm in -Rate_Max .. Rate_Max
      and then
        (if F.Advisory = Advisory_None then F.Target_Rate_Fpm = 0
         elsif F.Advisory = Advisory_Climb then F.Target_Rate_Fpm > 0
         else F.Target_Rate_Fpm < 0)
      and then (if F.Advisory /= Advisory_None then F.Mode = Mode_TA_RA)
      and then
        (if F.Audio = Audio_Traffic
         then F.Intruder_Valid = 1
              and then F.Intruder_Class = Class_Traffic)
      and then
        (if F.Audio = Audio_Climb or else F.Audio = Audio_Climb_Now
         then F.Advisory = Advisory_Climb)
      and then
        (if F.Audio = Audio_Descend or else F.Audio = Audio_Descend_Now
         then F.Advisory = Advisory_Descend)
      and then
        (if F.Audio = Audio_Clear_Of_Conflict
         then F.Advisory = Advisory_None))
   with Global => null;

end Tcas.Display_Port;