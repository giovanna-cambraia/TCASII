--  ===========================================================================
--  TCAS II - Traffic Alert and Collision Avoidance System
--  ===========================================================================
--  File        : src/ada/common/tcas.ads
--  Module      : Root of the Tcas package hierarchy (specification only)
--  Language    : Ada 2012 / SPARK 2014
--  Assurance   : DO-178C Software Level B (target)
--  Partition   : shared by P2, P3 and P4 (pure, no state, no code)
--  Requirements: REQ-002 (absence of runtime errors), placeholder IDs
--
--  PURPOSE
--    This package is the parent of every other unit in the project:
--    Tcas.Types, Tcas.Config, Tcas.Units, Tcas.Tracker and so on. It has
--    no declarations that the logic depends on. Its job is to give the
--    hierarchy one name, so that every unit is written Tcas.Something and
--    no unit name can collide with a library or a vendor package.
--
--    It carries one item of information: the version of the specification
--    this source tree implements, so that logs and tests can report it.
--
--  DESIGN RULES (each one maps to a SPARK or DO-178C argument)
--    R1  Pure package: no state, no elaboration code, no body. Every child
--        that is Pure requires a Pure parent, so this must be Pure.
--    R2  No logic. Nothing is computed here, so nothing here can fail at
--        run time and nothing here needs to be proved.
--    R3  Constants only, and only static ones. They are known at compile
--        time, so they add no elaboration order dependency.
--    R4  Do not add types here. Types belong in Tcas.Types. Keeping this
--        package empty of types means a change of type never forces a
--        recompilation of the whole hierarchy through the parent.
--
--  SPARK NOTES
--    * SPARK_Mode is On: the specification is inside the SPARK subset.
--    * No deviations are requested by this specification.
--  ===========================================================================

pragma SPARK_Mode (On);

package Tcas
  with Pure
is

   --  Version of the TCAS logic this tree implements; Version 7.1 is the one
   --  defined by RTCA DO-185B / EUROCAE ED-143 (ASM-001).
   Logic_Version_Major : constant := 7;
   Logic_Version_Minor : constant := 1;

end Tcas;