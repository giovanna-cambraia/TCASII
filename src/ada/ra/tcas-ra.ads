pragma SPARK_Mode (On);

with Tcas.Config;
with Tcas.Tracker;
with Tcas.Types; use Tcas.Types;

package Tcas.RA is

   package Tr renames Tcas.Tracker;

   use type Tr.Track_State;

   --  What the pilot is told to do.
   type RA_Kind is
     (No_Advisory,
      Climb,
      Descend);

   --  One advisory. Target_Rate is the vertical rate the pilot is asked
   --  to reach; it is 0 for No_Advisory.
   type Advisory is record
      Kind        : RA_Kind           := No_Advisory;
      Sense       : RA_Sense          := No_Sense;
      Target_Rate : Vertical_Rate_Fpm := 0;
   end record;

   --  Building blocks (public so that the tests can check them alone)

   function Effective_Tau (Item : Tr.Track) return Tau_S
   with
     Global => null,
     Post   => (if Item.Range_Rate >= 0 then Effective_Tau'Result = 0);

   function Sense_Separation
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Sense    : RA_Sense;
      Item     : Tr.Track) return Altitude_Diff_Ft
   with
     Global => null,
     Pre    => Sense /= No_Sense;

   function Choose_Sense
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track) return RA_Sense
   with
     Global => null,
     Pre    => Item.State = Tr.Active and then Own_SL >= 3,
     Post   => Choose_Sense'Result /= No_Sense;

   function Generate
     (Own_Alt  : Altitude_Ft;
      Own_Rate : Vertical_Rate_Fpm;
      Own_SL   : Sensitivity_Level;
      Item     : Tr.Track) return Advisory
   with
     Global => null,
     Pre    => Item.State = Tr.Active and then Own_SL >= 3,
     Post   =>
       Generate'Result.Sense /= No_Sense
       and then
         (if Generate'Result.Sense = Upward
          then Generate'Result.Kind = Climb
               and then Generate'Result.Target_Rate
                          = Tcas.Config.RA_Target_Rate_Fpm)
       and then
         (if Generate'Result.Sense = Downward
          then Generate'Result.Kind = Descend
               and then Generate'Result.Target_Rate
                          = -Tcas.Config.RA_Target_Rate_Fpm);

end Tcas.RA;