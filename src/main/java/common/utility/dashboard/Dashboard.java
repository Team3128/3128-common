package common.utility.dashboard;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import common.core.misc.NAR_Robot;
import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Publishes robot values, buttons and the auto chooser to the dashboard in a single call each.
 *
 * <pre>{@code
 * Dashboard.add("voltage", RobotController::getBatteryVoltage);
 * Supplier<Command> auto = Dashboard.addAutoChooser("Auto", "Default Auto", autos);
 * }</pre>
 *
 * <p>Values with a supplier are updated every loop, on the robot's main thread. A {@code /} in
 * a name groups values, for example, {@code "Tests/Intake"}. Adding an existing name replaces the old entry.
 *
 * <p>All values currently go to two dashboards:
 * <ul>
 * <li>Elastic, through SmartDashboard's NetworkTables entries, under the full name.</li>
 * <li>NarwhalDashboard, under the part of the name after the last {@code /}, which is the key its
 *     web page expects.</li>
 * </ul>
 * This is the only class that knows how values reach a dashboard: retiring NarwhalDashboard (see
 * {@code README.md} in the {@code utility} package for the plan), or moving off SmartDashboard
 * when WPILib removes it in 2027, should affect only this class.
 */
@SuppressWarnings("deprecation")
public class Dashboard {

    static {
        NAR_Robot.addPeriodic(Dashboard::update, 0.02);
    }

    /** The action to run each loop to update each value, by name. */
    private static final HashMap<String, Runnable> updaters = new HashMap<String, Runnable>();

    private Dashboard() {}

    /**
     * Displays a number that updates every loop.
     * @param name The name of the value on the dashboard
     * @param value The source of the value
     */
    public static void add(String name, DoubleSupplier value) {
        updaters.put(name, () -> SmartDashboard.putNumber(name, value.getAsDouble()));
        NarwhalDashboard.getInstance().addUpdate(narwhalDashboardKey(name), value::getAsDouble);
    }

    /**
     * Displays a Boolean value that updates every loop.
     * @param name The name of the value on the dashboard
     * @param value The source of the value
     */
    public static void add(String name, BooleanSupplier value) {
        updaters.put(name, () -> SmartDashboard.putBoolean(name, value.getAsBoolean()));
        NarwhalDashboard.getInstance().addUpdate(narwhalDashboardKey(name), value::getAsBoolean);
    }

    /**
     * Displays text that updates every loop.
     * @param name The name of the value on the dashboard
     * @param value The source of the value
     */
    public static void add(String name, Supplier<String> value) {
        updaters.put(name, () -> SmartDashboard.putString(name, value.get()));
        NarwhalDashboard.getInstance().addUpdate(narwhalDashboardKey(name), value::get);
    }

    /**
     * Displays a button that runs a command.
     * @param name The name of the button on the dashboard
     * @param command The command to run when the button is pressed
     */
    public static void add(String name, Command command) {
        add(name, (Sendable) command);
        NarwhalDashboard.getInstance().addButton(narwhalDashboardKey(name), (boolean pressed) -> {
            if (pressed) CommandScheduler.getInstance().schedule(command);
        });
    }

    /**
     * Displays an object that WPILib can already render on Elastic, for example, a {@code Field2d} as a
     * field. NarwhalDashboard does not display these.
     * @param name The name of the widget on the dashboard
     * @param sendable The object to display
     */
    public static void add(String name, Sendable sendable) {
        updaters.remove(name);
        SmartDashboard.putData(name, sendable);
    }

    /**
     * Displays a dropdown for selecting the auto. On NarwhalDashboard it is the auto chooser; an auto
     * selected there takes priority over one selected on Elastic.
     * @param name The name of the dropdown on the dashboard
     * @param defaultAuto The name of the auto that runs if none is selected
     * @param autos Each auto, keyed by the name shown in the dropdown, in display order
     * @return The selected auto
     */
    public static Supplier<Command> addAutoChooser(String name, String defaultAuto, Map<String, Command> autos) {
        final SendableChooser<Command> chooser = new SendableChooser<>();
        for (final Map.Entry<String, Command> auto : autos.entrySet()) {
            if (auto.getKey().equals(defaultAuto)) chooser.setDefaultOption(auto.getKey(), auto.getValue());
            else chooser.addOption(auto.getKey(), auto.getValue());
        }
        add(name, chooser);

        final NarwhalDashboard narwhalDashboard = NarwhalDashboard.getInstance();
        narwhalDashboard.addAutos(autos.keySet().toArray(new String[0]));
        return () -> {
            final String narwhalDashboardAuto = narwhalDashboard.getSelectedAuto();
            return narwhalDashboardAuto != null ? autos.get(narwhalDashboardAuto) : chooser.getSelected();
        };
    }

    /** Returns the key NarwhalDashboard's web page uses for a value: the name after its last {@code /}. */
    private static String narwhalDashboardKey(String name) {
        return name.substring(name.lastIndexOf('/') + 1);
    }

    /**
     * Sends every value's latest reading to Elastic. Elastic's buttons and dropdowns are read by
     * {@code SmartDashboard.updateValues()}, which the robot loop already calls, and
     * NarwhalDashboard sends its own values every loop.
     */
    static void update() {
        for (final Runnable updater : updaters.values()) {
            updater.run();
        }
    }
}
