A PID Controller for CraftBeerPi4 Kettles that uses Time Proportioning for output.

This is useful for gas burners whose output is controlled as either ON or OFF.  The PID output is converted into a proportion of the specified time interval during which the burner is turned either on or off.

This controller also helps eliminate jitter (rapid quick firing of the burner), by providing a configurable minimum ON time for each interval, to preserve burner equipment.
