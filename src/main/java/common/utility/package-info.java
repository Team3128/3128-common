/**
 * Tools for seeing what the robot is doing and checking it works.
 *
 * <ul>
 * <li>{@link common.utility.Log}: consistent console messages, sorted by how serious they are.</li>
 * <li>{@code shuffleboard}: Shuffleboard widgets in one line each, for showing values and tuning.</li>
 * <li>{@code dashboard}: values, autos and buttons on the dashboard, in one line each. They go to
 *     both Elastic and NarwhalDashboard.</li>
 * <li>{@code narwhaldashboard}: the server for the team's own web dashboard. Use {@code dashboard}
 *     rather than calling it directly.</li>
 * <li>{@code tester}: system checks to run in the pits before a match.</li>
 * <li>{@code sysid}: tests that measure a mechanism so SysId can work out its feedforward gains.</li>
 * </ul>
 *
 * <p>For logging values to review after a match, mechanisms and state machines already publish
 * everything to NetworkTables, which the robot records to a log file for AdvantageScope.
 */
package common.utility;
