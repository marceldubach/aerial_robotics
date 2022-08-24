## Notes from 14.5.2020

**logging variables:**
- define a logging variable (e.g. stabilizer)
- add the measurements into the logging variable (same names as in cfclient)
- a logging variable can take up to 6 measurements!
*done*
if we need more, maybe we can create a second variable, but we need to change the logfile creation then (include both variables....)

**zrange sensor**
The zrange sensor is included in the state estimation. This means the drone goes up when it sees something below itself.
We need to adapt the setpoint lower if we detect a box below the drone.
(work in progress)


**waypoint navigation** 
- p-controller is implemented
- initial offset is causing problems (offset can be obtained only after take-off... :(:( 
this is not very precise
--> is there a way to set the estimator to 0 at the beginning?

- what about callbacks and position commander??

