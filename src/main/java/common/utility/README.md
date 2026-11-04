# Utilities

Tools for seeing what the robot is doing and checking that it works.

| Package or class | Purpose |
|---|---|
| [`Log`](Log.java) | Consistent console messages, sorted by severity. |
| [`shuffleboard`](shuffleboard) | Shuffleboard widgets in one line each, for showing values and tuning. |
| [`dashboard`](dashboard) | Values, autos and buttons on the dashboard in one line each. They go to both Elastic and NarwhalDashboard. |
| [`narwhaldashboard`](narwhaldashboard) | The server for the team's own web dashboard, which is being replaced by Elastic. Use `dashboard` rather than calling it directly. |
| [`tester`](tester) | System checks to run in the pits before a match. |
| [`sysid`](sysid) | Tests that measure a mechanism so SysId can calculate its feedforward gains. |

To review values after a match, no extra logging is needed: mechanisms and state machines already
publish everything to NetworkTables, which the robot records to a log file for AdvantageScope.

## Replacing NarwhalDashboard with Elastic

The team is moving to Elastic, the dashboard included with WPILib, so there is no web page or server
of our own to maintain. Robot code uses [`Dashboard`](dashboard/Dashboard.java), which sends
everything to both Elastic and NarwhalDashboard until the switch is finished. To finish it:

1. In the robot project, lay out Elastic with the drivers, export the layout to
   `src/main/deploy/elastic-layout.json`, and add
   `WebServer.start(5800, Filesystem.getDeployDirectory().getPath())` to the `Robot` constructor, so
   any laptop can load it with File, Download from Robot.
2. Once the drivers use only Elastic, delete the `narwhaldashboard` package and the NarwhalDashboard
   lines in `Dashboard`. In `build.gradle`, remove the Java-WebSocket and json-simple dependencies
   (nothing else uses them) and the coverage exclusion for this package.
3. In the robot project, rename the dashboard values from NarwhalDashboard's keys (`robotX`,
   `voltage`, ...) to names that read well in Elastic.
