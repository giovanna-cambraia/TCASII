--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : tests/integration/test_integration.adb
--  Module      : Integration test of the Ada core, port records to advisory
--  Language    : Ada 2012 (the tests are not in the SPARK subset)
--  Assurance   : DO-178C Software Level B (target), verification artifact
--  Tests       : src/ada/decoder (A429 decoder, Mode S parser),
--                src/ada/scheduler, src/ada/tracker, src/ada/threat,
--                src/ada/ra, wired together exactly as the cyclic
--                executive will wire them
--  Requirements: REQ-001, REQ-002, REQ-003 (placeholder IDs)
--
--  PURPOSE
--    The unit tests check each package alone, with inputs written by hand.
--    This test checks that the packages FIT: that what one produces is
--    what the next one needs. It feeds the chain with the records that the
--    C drivers leave in memory (Tcas.A429_Port.Word and
--    Tcas.Modes_Port.Frame), runs the real decoders and the real parser,
--    and drives the real scheduler through whole encounters.
--
--  WHAT IS AND IS NOT COVERED
--    The C drivers themselves are NOT called here (that needs a mixed
--    Ada/C build and a mock of the volatile registers). They are covered
--    by their own Unity tests, and the layout of the records that cross
--    into Ada is covered by check_a429_layout.c and check_modes_layout.c.
--    A test that calls the C drivers from Ada is a separate step.
--
--  DESIGN RULES OF THE TESTS
--    T1  Every scenario starts with Sc.Init, so scenarios do not depend on
--        each other or on their order.
--    T2  Frames are built by this test: the format and the altitude code
--        by hand (AC_Of, checked against hand-worked codes first), the AP
--        field as CRC XOR address. The CRC comes from Mp.CRC24, whose own
--        correctness is the job of test_modes_parser.adb (published
--        vector, hand values, independent reference); here it only has to
--        agree with itself, so that a frame built for an address parses
--        back to that address.
--    T3  Expected values are worked out by hand and written next to each
--        check. The sweep uses an independent formula (see the header
--        table) for the slant, not a copy of the parser.
--    T4  After EVERY cycle of the encounter the whole output is checked
--        (level, event, advisory, selected track, its class) and the
--        invariant of the RA state.
--    T5  Values are literals from Tcas.Config (all [VERIFY]): reply delay
--        128 000 ns, 49 178 553 / 100 000 000 ft per ns, the SL 4
--        thresholds, RA_Clear_Cycles 2, response delay 5 s, target rate
--        1 500 ft/min, Max_Track_Age_Ms 5 000. When the table is corrected
--        in v2, these tests must change with it, and that is intended.
--    T6  Output: one line per failed check, then a summary. The exit
--        status is 1 if anything failed, else 0, so a script can use it.
--
--  GEOMETRY OF THE ENCOUNTER (ownship at 4 000 ft, rate 0, radio altitude
--  not available: Select_Level gives SL 4; one cycle is 1 000 ms)
--    Aircraft ABCDEF at 4 000 ft, replies at these round trips:
--      cycle 1   134 711 ns  ->  3 300 ft   (one update: Proximate)
--      cycle 2   134 101 ns  ->  3 000 ft   rate -300 ft/s, tau 10: RA
--      cycle 3   133 491 ns  ->  2 700 ft   RA kept
--      cycle 4   132 881 ns  ->  2 400 ft   RA kept
--      cycle 5   433 013 ns  -> 150 000 ft  moving away: first quiet cycle
--      cycle 6   433 013 ns  -> 150 000 ft  second quiet cycle: cleared
--      cycle 7   433 013 ns  -> 150 000 ft  nothing to report
--    How each round trip gives its slant (feet = net * 49 178 553 / 1E8,
--    truncated toward zero, net = round trip - 128 000):
--      net 6 711   330 037 269 183 / 1E8 = 3 300.37 -> 3 300
--      net 6 101   300 038 351 853 / 1E8 = 3 000.38 -> 3 000
--      net 5 491   270 039 434 523 / 1E8 = 2 700.39 -> 2 700
--      net 4 881   240 040 517 193 / 1E8 = 2 400.40 -> 2 400
--      net 305 013                        -> 150 000 (parser test)
--    The tracker derives the range rate from two successive slants one
--    cycle apart: (3 000 - 3 300) * 1 000 / 1 000 = -300 ft/s, so tau is
--    10 s. At tau 10 each sense gains 125 ft from the pilot response and
--    the intruder is level, so Up = Down = 125, neither reaches ALIM
--    (300), the tie goes to Upward: a Climb at +1 500 ft/min.
--
--  ALTITUDE CODES USED (13 bits, M = 0, Q = 1, N = (altitude + 1 000)/25)
--    4 000 ft: N = 200 -> 16#0318#     10 000 ft: N = 440 -> 16#06B8#
--    -975 ft:  N = 1   -> 16#0011#     38 000 ft: N = 1 560 -> 16#1838#
--    0 ft:     N = 40  -> 16#0098#     50 175 ft: N = 2 047 -> 16#1FBF#
--
--  TESTS BY GROUP
--    encoder     : the test's own altitude encoder against hand codes
--    ownship     : A429 words through the decoders into Ownship_Data
--    encounter   : seven cycles, port records to advisory, and the data
--                  that reaches the tracker
--    hostile     : ghost, corrupted, DF 17, bad timing, rejected and
--                  malformed frames never reach the tracker
--    fail safe   : DOCUMENTS the behavior of an invalid pressure altitude
--                  (see FINDING below)
--    sweep       : 3 015 frames through parser, scheduler and tracker
--
--  FINDING (open item for v2, documented by the fail safe group)
--    When the pressure altitude is not valid the decoder returns 0 ft
--    (rule R5 of Tcas.A429_Decoder) and Tcas.Scheduler passes that 0 to
--    the threat logic, because it reads Own.Pressure.Value whether it is
--    valid or not. The level correctly falls to SL 2, but the intruders
--    are classified against an ownship at 0 ft, so a real traffic
--    advisory is lost. This is NOT fail-safe. The tests check the current
--    behavior so that the fix (hold the last good altitude with an age
--    limit, or inhibit advisories while it is invalid) is a visible change.
--
--  USAGE (from the project root; no project file is needed)
--    mkdir -p build/obj_test build/bin
--    gnatmake tests/integration/test_integration.adb -gnat2012 -gnatwa \
--        -D build/obj_test -o build/bin/test_integration.exe \
--        -Isrc/ada/common -Isrc/ada/common/types \
--        -Isrc/ada/common/config -Isrc/ada/common/units \
--        -Isrc/ada/tracker -Isrc/ada/threat -Isrc/ada/ra \
--        -Isrc/ada/scheduler -Isrc/ada/decoder -Isrc/bindings
--    build/bin/test_integration.exe
--
--  NOTES
--    * tcas2.gpr does not list tests/, so the project build never sees
--      this program (it has a main procedure and is not part of the
--      product).
--    * If a check fails, recheck the hand arithmetic in the header before
--      blaming the chain.

with Ada.Command_Line;
with Ada.Text_IO;

with Tcas.A429_Decoder;
with Tcas.A429_Port;
with Tcas.Modes_Parser;
with Tcas.Modes_Port;
with Tcas.RA;
with Tcas.RA_State;
with Tcas.Scheduler;
with Tcas.Tracker;
with Tcas.Types;

procedure Test_Integration is

   package A4  renames Tcas.A429_Port;
   package Ms  renames Tcas.Modes_Port;
   package Dec renames Tcas.A429_Decoder;
   package Mp  renames Tcas.Modes_Parser;
   package Sc  renames Tcas.Scheduler;
   package Tr  renames Tcas.Tracker;
   package Rg  renames Tcas.RA;
   package Rs  renames Tcas.RA_State;
   package Ty  renames Tcas.Types;

   use type Dec.Decode_Status;
   use type Mp.Parse_Status;
   use type Rs.RA_Event;
   use type Rg.Advisory;
   use type Ty.Threat_Class;
   use type Ms.U32;

   --  Test bookkeeping
   Total  : Natural := 0;
   Failed : Natural := 0;

   procedure Check (Name : String; Condition : Boolean) is
   begin
      Total := Total + 1;
      if not Condition then
         Failed := Failed + 1;
         Ada.Text_IO.Put_Line ("FAIL: " & Name);
      end if;
   end Check;

   --  Constants
   Addr1      : constant Natural := 16#ABCDEF#;   --  the aircraft we ask
   Addr_Ghost : constant Natural := 16#ABCDE0#;   --  another aircraft

   RTT_3300 : constant Long_Long_Integer := 134_711;
   RTT_3000 : constant Long_Long_Integer := 134_101;
   RTT_2700 : constant Long_Long_Integer := 133_491;
   RTT_2400 : constant Long_Long_Integer := 132_881;
   RTT_Far  : constant Long_Long_Integer := 433_013;

   No_Adv : constant Rg.Advisory :=
     (Kind => Rg.No_Advisory, Sense => Ty.No_Sense, Target_Rate => 0);
   Climb_Adv : constant Rg.Advisory :=
     (Kind => Rg.Climb, Sense => Ty.Upward, Target_Rate => 1_500);

   --  A429 words (what the C driver leaves, rule T2 of the unit tests)
   function Valid_Word (Label : A4.U8; Data : Natural) return A4.Word is
     (Data       => A4.U32 (Data),
      Frame      => 1,
      Label      => Label,
      Sdi        => 0,
      Ssm        => A4.Ssm_Normal,
      Status     => A4.Status_OK,
      Valid      => 1,
      Reserved8  => 0,
      Reserved16 => 0);

   function Rejected_Word (Label : A4.U8; Status : A4.U8) return A4.Word is
     (Data       => 0,
      Frame      => 1,
      Label      => Label,
      Sdi        => 0,
      Ssm        => 0,
      Status     => Status,
      Valid      => 0,
      Reserved8  => 0,
      Reserved16 => 0);

   --  The ownship data the way the cyclic executive builds it: through
   --  the real decoders. The radio altitude is not available.
   function Own_Data
     (Pressure_Word : A4.Word;
      Rate_Word     : A4.Word) return Sc.Ownship_Data
   is
      PR : constant Dec.Altitude_Result :=
        Dec.Decode_Pressure_Altitude (Pressure_Word);
      RR : constant Dec.Vertical_Rate_Result :=
        Dec.Decode_Altitude_Rate (Rate_Word);
   begin
      return (Pressure   => (Valid => PR.Status = Dec.Decoded,
                             Value => PR.Value),
              Radio      => (Valid => False, Value => 0),
              Rate_Valid => RR.Status = Dec.Decoded,
              Rate       => RR.Value);
   end Own_Data;

   --  4 000 ft is 8 000 counts (divisor 2); rate 0 is 0 counts.
   Pressure_4000 : constant A4.Word :=
     Valid_Word (A4.Label_Pressure_Alt, 8_000);
   Rate_Zero     : constant A4.Word := Valid_Word (A4.Label_Alt_Rate, 0);

   Own_Chain : constant Sc.Ownship_Data :=
     Own_Data (Pressure_4000, Rate_Zero);

   Own_Bad : constant Sc.Ownship_Data :=
     Own_Data (Rejected_Word (A4.Label_Pressure_Alt, A4.Status_Err_Ssm_Fw),
               Rate_Zero);

   --  Mode S frames (what the C driver leaves)

   --  The altitude code of an altitude in feet, M = 0, Q = 1:
   --  N = (altitude + 1 000) / 25, bits 12 .. 7 are N / 32, bit 5 is
   --  (N mod 32) / 16, bits 3 .. 0 are N mod 16, bit 4 (Q) is set.
   function AC_Of (Alt : Integer) return Natural is
      N : constant Natural := (Alt + 1_000) / 25;
   begin
      return (N / 32) * 128 + ((N mod 32) / 16) * 32 + 16 + N mod 16;
   end AC_Of;

   --  A valid reply of aircraft Addr: format DF, altitude code AC, the AP
   --  field built as CRC XOR address (rule T2).
   function Build
     (DF      : Natural;
      Long    : Boolean;
      AC      : Natural;
      Addr    : Natural;
      RTT     : Long_Long_Integer;
      Bearing : Natural) return Ms.Frame
   is
      Len : constant Natural := (if Long then 14 else 7);
      F   : Ms.Frame :=
        (Data          => (others => 0),
         Length        => Ms.U8 (Len),
         Status        => Ms.Status_OK,
         Round_Trip_Ns => Ms.U32 (RTT),
         Bearing       => Ms.U16 (Bearing),
         Valid         => 1,
         Reserved      => 0);
      AP : Ms.U32;
   begin
      F.Data (0) := Ms.U8 (DF * 8);
      F.Data (2) := Ms.U8 (AC / 256);
      F.Data (3) := Ms.U8 (AC mod 256);

      AP := Mp.CRC24 (F) xor Ms.U32 (Addr);

      F.Data (Len - 3) := Ms.U8 (AP / 65_536);
      F.Data (Len - 2) := Ms.U8 ((AP / 256) mod 256);
      F.Data (Len - 1) := Ms.U8 (AP mod 256);

      return F;
   end Build;

   --  A frame the driver rejected: invalid, carrying nothing.
   function Rejected_Frame (Status : Natural) return Ms.Frame is
     (Data          => (others => 0),
      Length        => 0,
      Status        => Ms.U8 (Status),
      Round_Trip_Ns => 0,
      Bearing       => 0,
      Valid         => 0,
      Reserved      => 0);

   --  The chain: frames to measurements, measurements to a cycle

   Buf : Sc.Measurement_Array (1 .. 16) :=
     (others => Mp.Zero_Measurement);

   C : Sc.Cycle_State;
   O : Sc.Cycle_Output;

   --  Parses one frame for the aircraft we interrogated; a parsed
   --  measurement is appended to Buf, exactly as the executive does.
   procedure Ingest
     (F        : Ms.Frame;
      Expected : Natural;
      N        : in out Natural;
      Status   : out Mp.Parse_Status)
   is
      R : constant Mp.Parse_Result :=
        Mp.Parse (F, Ty.Mode_S_Address (Expected));
   begin
      Status := R.Status;

      if R.Status = Mp.Parsed then
         N := N + 1;
         Buf (N) := R.Value;
      end if;
   end Ingest;

   --  Runs one cycle with the first N measurements of Buf.
   procedure Cyc
     (Own     : Sc.Ownship_Data;
      N       : Natural;
      Elapsed : Natural := 1_000) is
   begin
      Sc.Run_Cycle (C, Own, Buf (1 .. N), Ty.Time_Ms (Elapsed), O);
   end Cyc;

   --  One cycle of the standard aircraft: a reply of aircraft ABCDEF at
   --  4 000 ft and bearing 9 000, with the given round trip.
   procedure Fly
     (Tag : String;
      RTT : Long_Long_Integer;
      Own : Sc.Ownship_Data)
   is
      N  : Natural := 0;
      St : Mp.Parse_Status;
   begin
      Ingest (Build (0, False, AC_Of (4_000), Addr1, RTT, 9_000),
              Addr1, N, St);
      Check (Tag & ": the frame parses (got " & Mp.Parse_Status'Image (St)
             & ")", St = Mp.Parsed);
      Cyc (Own, N);
   end Fly;

   --  Checks the whole output of a cycle (rule T4).
   procedure Check_Out
     (Name     : String;
      Level    : Natural;
      Event    : Rs.RA_Event;
      Adv      : Rg.Advisory;
      Selected : Natural;
      Class    : Ty.Threat_Class) is
   begin
      Check (Name & ": level", Natural (O.Level) = Level);
      Check (Name & ": event (expected " & Rs.RA_Event'Image (Event)
             & ", got " & Rs.RA_Event'Image (O.Event) & ")",
             O.Event = Event);
      Check (Name & ": advisory", O.Adv = Adv);
      Check (Name & ": selected track", O.Selected = Selected);
      Check (Name & ": class (expected " & Ty.Threat_Class'Image (Class)
             & ", got " & Ty.Threat_Class'Image (O.Selected_Class) & ")",
             O.Selected_Class = Class);
      Check (Name & ": RA state consistent", Rs.Is_Consistent (C.RA));
   end Check_Out;

   --  Group: the test's own encoder (rule T2)
   procedure Test_Encoder is
   begin
      Check ("encoder: 4000 ft is 0318",   AC_Of (4_000)  = 16#0318#);
      Check ("encoder: 10000 ft is 06B8",  AC_Of (10_000) = 16#06B8#);
      Check ("encoder: -975 ft is 0011",   AC_Of (-975)   = 16#0011#);
      Check ("encoder: 0 ft is 0098",      AC_Of (0)      = 16#0098#);
      Check ("encoder: 38000 ft is 1838",  AC_Of (38_000) = 16#1838#);
      Check ("encoder: 50175 ft is 1FBF",  AC_Of (50_175) = 16#1FBF#);
   end Test_Encoder;

   --  Group: ownship words through the decoders
   procedure Test_Ownship is
      Wrong_Label  : constant A4.Word :=
        Valid_Word (A4.Label_Radio_Alt, 8_000);
      Out_Of_Range : constant A4.Word :=
        Valid_Word (A4.Label_Pressure_Alt, 16#3_FFFF#);
      Bad_Rate     : constant Sc.Ownship_Data :=
        Own_Data (Pressure_4000,
                  Rejected_Word (A4.Label_Alt_Rate, A4.Status_Err_Parity));
      Wrong        : constant Sc.Ownship_Data :=
        Own_Data (Wrong_Label, Rate_Zero);
      Range_Bad    : constant Sc.Ownship_Data :=
        Own_Data (Out_Of_Range, Rate_Zero);
   begin
      Check ("ownship: pressure valid", Own_Chain.Pressure.Valid);
      Check ("ownship: 8000 counts is 4000 ft",
             Integer (Own_Chain.Pressure.Value) = 4_000);
      Check ("ownship: radio not available", not Own_Chain.Radio.Valid);
      Check ("ownship: rate valid", Own_Chain.Rate_Valid);
      Check ("ownship: rate 0", Integer (Own_Chain.Rate) = 0);

      Check ("ownship: driver-rejected pressure is not valid",
             not Own_Bad.Pressure.Valid);
      Check ("ownship: a failed pressure altitude reads 0 ft",
             Integer (Own_Bad.Pressure.Value) = 0);
      Check ("ownship: the rate is still valid", Own_Bad.Rate_Valid);

      Check ("ownship: driver-rejected rate is not valid",
             not Bad_Rate.Rate_Valid);
      Check ("ownship: a failed rate reads 0", Integer (Bad_Rate.Rate) = 0);
      Check ("ownship: the pressure altitude is untouched",
             Bad_Rate.Pressure.Valid
             and then Integer (Bad_Rate.Pressure.Value) = 4_000);

      Check ("ownship: wrong label is not valid", not Wrong.Pressure.Valid);
      Check ("ownship: out of range is not valid, not clamped",
             not Range_Bad.Pressure.Valid
             and then Integer (Range_Bad.Pressure.Value) = 0);
   end Test_Ownship;

   --  Group: a full encounter, port records to advisory
   procedure Test_Encounter is
      Id : Natural;
   begin
      Sc.Init (C);

      --  cycle 1: the first reply creates a track, one update only
      Fly ("enc 1", RTT_3300, Own_Chain);
      Check_Out ("enc 1", 4, Rs.No_Event, No_Adv, 1, Ty.Proximate_Traffic);

      --  what reached the tracker is what was encoded (rule T3)
      Id := Tr.Find (C.Table, Ty.Mode_S_Address (Addr1));
      Check ("enc 1: the track exists", Id = 1);
      Check ("enc 1: tracker address",
             Natural (C.Table (1).Address) = Addr1);
      Check ("enc 1: tracker slant is 3300 ft",
             Natural (C.Table (1).Slant) = 3_300);
      Check ("enc 1: tracker bearing is 9000",
             Natural (C.Table (1).Bearing) = 9_000);
      Check ("enc 1: tracker altitude is 4000 ft",
             Integer (C.Table (1).Altitude) = 4_000);
      Check ("enc 1: one update", Natural (C.Table (1).Updates) = 1);
      Check ("enc 1: no rate yet",
             Integer (C.Table (1).Range_Rate) = 0
             and then Integer (C.Table (1).Vertical_Rate) = 0);

      --  cycle 2: the rate is -300 ft/s, tau 10: an RA, a Climb (tie)
      Fly ("enc 2", RTT_3000, Own_Chain);
      Check_Out ("enc 2", 4, Rs.Issued, Climb_Adv, 1,
                 Ty.Resolution_Advisory);
      Check ("enc 2: tracker slant is 3000 ft",
             Natural (C.Table (1).Slant) = 3_000);
      Check ("enc 2: derived range rate is -300 ft/s",
             Integer (C.Table (1).Range_Rate) = -300);
      Check ("enc 2: two updates", Natural (C.Table (1).Updates) = 2);

      --  cycles 3 and 4: the same advisory
      Fly ("enc 3", RTT_2700, Own_Chain);
      Check_Out ("enc 3", 4, Rs.Maintained, Climb_Adv, 1,
                 Ty.Resolution_Advisory);
      Check ("enc 3: tracker slant is 2700 ft",
             Natural (C.Table (1).Slant) = 2_700);

      Fly ("enc 4", RTT_2400, Own_Chain);
      Check_Out ("enc 4", 4, Rs.Maintained, Climb_Adv, 1,
                 Ty.Resolution_Advisory);
      Check ("enc 4: tracker slant is 2400 ft",
             Natural (C.Table (1).Slant) = 2_400);

      --  cycle 5: the intruder moves away; the range rate clamps to
      --  +3000 ft/s; first quiet cycle, the advisory is held
      Fly ("enc 5", RTT_Far, Own_Chain);
      Check_Out ("enc 5", 4, Rs.Clearing, Climb_Adv, 1, Ty.Other_Traffic);
      Check ("enc 5: tracker slant is 150000 ft",
             Natural (C.Table (1).Slant) = 150_000);
      Check ("enc 5: range rate clamps to +3000 ft/s",
             Integer (C.Table (1).Range_Rate) = 3_000);

      --  cycle 6: second quiet cycle, the RA ends
      Fly ("enc 6", RTT_Far, Own_Chain);
      Check_Out ("enc 6", 4, Rs.Cleared, No_Adv, 1, Ty.Other_Traffic);
      Check ("enc 6: no RA active", not C.RA.Active);

      --  cycle 7: nothing more to report
      Fly ("enc 7", RTT_Far, Own_Chain);
      Check_Out ("enc 7", 4, Rs.No_Event, No_Adv, 1, Ty.Other_Traffic);
      Check ("enc 7: still one track", Tr.Count (C.Table) = 1);
   end Test_Encounter;

   --  Group: hostile and garbage frames never reach the tracker

   procedure Test_Hostile is
      Good : constant Ms.Frame :=
        Build (0, False, AC_Of (4_000), Addr1, RTT_3300, 9_000);

      Ghost : constant Ms.Frame :=
        Build (0, False, AC_Of (4_000), Addr_Ghost, RTT_3300, 9_000);

      Corrupt : Ms.Frame := Good;
      Malformed : Ms.Frame := Good;

      DF17 : constant Ms.Frame :=
        Build (17, True, AC_Of (4_000), Addr1, RTT_3300, 9_000);

      Bad_Time : constant Ms.Frame :=
        Build (0, False, AC_Of (4_000), Addr1, 4_294_967_295, 9_000);

      N  : Natural := 0;
      St : Mp.Parse_Status;
   begin
      Corrupt.Data (1) := Ms.U8 ((Natural (Corrupt.Data (1)) + 1) mod 256);
      Malformed.Reserved := 1;

      --  the good frame itself parses (so the failures below are real)
      Ingest (Good, Addr1, N, St);
      Check ("hostile: the good frame parses", St = Mp.Parsed and then N = 1);
      N := 0;

      --  each bad frame is rejected for its own reason
      Ingest (Ghost, Addr1, N, St);
      Check ("hostile: ghost aircraft (wrong address, valid CRC)",
             St = Mp.Wrong_Address);

      Ingest (Corrupt, Addr1, N, St);
      Check ("hostile: corrupted message",
             St = Mp.Wrong_Address);

      Ingest (DF17, Addr1, N, St);
      Check ("hostile: DF 17 squitter",
             St = Mp.Unsupported_Format);

      Ingest (Bad_Time, Addr1, N, St);
      Check ("hostile: impossible round trip",
             St = Mp.Bad_Timing);

      Ingest (Rejected_Frame (3), Addr1, N, St);
      Check ("hostile: driver-rejected frame",
             St = Mp.Driver_Rejected);

      Ingest (Malformed, Addr1, N, St);
      Check ("hostile: malformed frame",
             St = Mp.Malformed_Frame);

      Check ("hostile: none of them produced a measurement", N = 0);

      --  a cycle with nothing valid leaves the table empty
      Sc.Init (C);
      Cyc (Own_Chain, N);
      Check_Out ("hostile cycle", 4, Rs.No_Event, No_Adv, 0,
                 Ty.Other_Traffic);
      Check ("hostile cycle: no tracks", Tr.Count (C.Table) = 0);
      Check ("hostile cycle: nothing rejected by the table",
             O.Rejected = 0 and then O.Dropped = 0);

      --  a valid frame together with a ghost: only the valid one is
      --  tracked
      Sc.Init (C);
      N := 0;
      Ingest (Ghost, Addr1, N, St);
      Ingest (Good, Addr1, N, St);
      Check ("hostile mixed: exactly one measurement", N = 1);
      Cyc (Own_Chain, N);
      Check ("hostile mixed: one track", Tr.Count (C.Table) = 1);
      Check ("hostile mixed: the real aircraft is tracked",
             Tr.Find (C.Table, Ty.Mode_S_Address (Addr1)) /= Tr.No_Track);
      Check ("hostile mixed: the ghost is not",
             Tr.Find (C.Table, Ty.Mode_S_Address (Addr_Ghost))
               = Tr.No_Track);
   end Test_Hostile;

   --  Group: fail safe (documents the FINDING of the header)

   procedure Test_Fail_Safe is
   begin
      Check ("failsafe: pressure not valid", not Own_Bad.Pressure.Valid);
      Check ("failsafe: its value is 0 ft (the cause of the finding)",
             Integer (Own_Bad.Pressure.Value) = 0);

      Sc.Init (C);

      --  the intruder is level with the REAL ownship (4 000 ft) and on a
      --  collision course, but the ownship is read as 0 ft: the level
      --  falls to SL 2 (correct) and the intruder is Other_Traffic, 4 000
      --  ft away vertically (not a TA: the finding)
      Fly ("failsafe 1", RTT_3300, Own_Bad);
      Check_Out ("failsafe 1", 2, Rs.No_Event, No_Adv, 1,
                 Ty.Other_Traffic);

      Fly ("failsafe 2", RTT_3000, Own_Bad);
      Check_Out ("failsafe 2", 2, Rs.No_Event, No_Adv, 1,
                 Ty.Other_Traffic);

      Check ("failsafe: the tracker still has the real geometry",
             Natural (C.Table (1).Slant) = 3_000
             and then Integer (C.Table (1).Range_Rate) = -300
             and then Integer (C.Table (1).Altitude) = 4_000);
      Check ("failsafe: no RA, as the fail-safe level requires",
             not C.RA.Active);

      --  the same two frames with a valid altitude give an RA (group
      --  encounter): only the ownship altitude differs
      Sc.Init (C);
      Fly ("failsafe control 1", RTT_3300, Own_Chain);
      Fly ("failsafe control 2", RTT_3000, Own_Chain);
      Check ("failsafe control: with a valid altitude the same frames give an RA",
             O.Event = Rs.Issued);
   end Test_Fail_Safe;

   --  Group: sweep of frames through parser, scheduler and tracker

   --  For every combination of round trip, intruder altitude and bearing:
   --  build a frame, parse it, run one cycle on a fresh table, and compare
   --  what is in the tracker with what was encoded. The expected slant
   --  comes from the formula of the header (rule T3), written here with
   --  its own arithmetic. The round trips are 128 000 + K * 1 525 ns for K
   --  in 0 .. 200 (net 0 .. 305 000 ns, slants 0 .. 149 994 ft).
   procedure Sweep is
      Alts     : constant array (1 .. 5) of Integer :=
        (-975, 0, 4_000, 38_000, 50_175);
      Bearings : constant array (1 .. 3) of Natural :=
        (0, 18_000, 35_999);

      Combos : Natural := 0;
      Bad    : Natural := 0;
   begin
      for K in 0 .. 200 loop
         for Ai in Alts'Range loop
            for Bi in Bearings'Range loop
               declare
                  Net : constant Long_Long_Integer :=
                    Long_Long_Integer (K) * 1_525;
                  RTT : constant Long_Long_Integer := 128_000 + Net;
                  Exp_Slant : constant Long_Long_Integer :=
                    (Net * 49_178_553) / 100_000_000;

                  N  : Natural := 0;
                  St : Mp.Parse_Status;
                  Ok : Boolean;
               begin
                  Sc.Init (C);
                  Ingest (Build (0, False, AC_Of (Alts (Ai)), Addr1, RTT,
                                 Bearings (Bi)),
                          Addr1, N, St);
                  Cyc (Own_Chain, N);

                  Ok := St = Mp.Parsed
                    and then N = 1
                    and then Tr.Count (C.Table) = 1
                    and then Tr.Find (C.Table, Ty.Mode_S_Address (Addr1)) = 1
                    and then Long_Long_Integer (C.Table (1).Slant)
                               = Exp_Slant
                    and then Integer (C.Table (1).Altitude) = Alts (Ai)
                    and then Natural (C.Table (1).Bearing) = Bearings (Bi)
                    and then Natural (C.Table (1).Updates) = 1
                    and then Natural (O.Level) = 4
                    and then O.Event = Rs.No_Event
                    and then O.Selected = 1;

                  Combos := Combos + 1;

                  if not Ok then
                     if Bad = 0 then
                        Ada.Text_IO.Put_Line
                          ("sweep first mismatch: K" & Integer'Image (K)
                           & " altitude" & Integer'Image (Alts (Ai))
                           & " bearing" & Natural'Image (Bearings (Bi))
                           & ", parse status " & Mp.Parse_Status'Image (St)
                           & ", expected slant"
                           & Long_Long_Integer'Image (Exp_Slant));
                     end if;
                     Bad := Bad + 1;
                  end if;
               end;
            end loop;
         end loop;
      end loop;

      Check ("sweep: all" & Natural'Image (Combos)
             & " frames (mismatches" & Natural'Image (Bad) & ")",
             Combos = 3_015 and then Bad = 0);
   end Sweep;

begin
   Test_Encoder;
   Test_Ownship;
   Test_Encounter;
   Test_Hostile;
   Test_Fail_Safe;
   Sweep;

   Ada.Text_IO.Put_Line
     ("test_integration:" & Natural'Image (Total) & " checks,"
      & Natural'Image (Failed) & " failed");

   if Failed > 0 then
      Ada.Text_IO.Put_Line ("FAIL");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   else
      Ada.Text_IO.Put_Line ("OK");
   end if;
end Test_Integration;