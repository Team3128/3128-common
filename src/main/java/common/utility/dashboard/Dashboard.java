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
 * Puts robot values, buttons and the auto picker on the dashboard, in one line each.
 *
 * <pre>{@code
 * Dashboard.add("voltage", RobotController::getBatteryVoltage);
 * Supplier<Command> auto = Dashboard.addAutoChooser("Auto", "Default Auto", autos);
 * }</pre>
 *
 * <p>Values with a supplier are updated every loop, on the robot's own thread. Use a {@code /} in
 * a name to group values, e.g. {@code "Tests/Intake"}. Adding a name again replaces the old one.
 *
 * <p>Everything goes to two dashboards for now:
 * <ul>
 * <li>Elastic, through SmartDashboard's NetworkTables entries, under the full name.</li>
 * <li>NarwhalDashboard, under the part of the name after the last {@code /}, which is the key its
 *     web page looks for.</li>
 * </ul>
 * This is the only class that knows how values reach a dashboard: retiring NarwhalDashboard (see
 * its class comment for the plan), or moving off SmartDashboard when WPILib removes it in 2027,
 * should only change this class.
 */
@SuppressWarnings("deprecation") // NarwhalDashboard, until it's retired
public class Dashboard {

    // Every value is updated every loop
    static {
        NAR_Robot.addPeriodic(Dashboard::update, 0.02);
    }

    // What to run each loop to update each value, by name
    private static final HashMap<String, Runnable> updaters = new HashMap<String, Runnable>();

    private Dashboard() {}

    /**
     * Shows a number that updates every loop.
     * @param name Name of the value on the dashboard
     * @param value Where the value comes from
     */
    public static void add(String name, DoubleSupplier value) {
        updaters.put(name, () -> SmartDashboard.putNumber(name, value.getAsDouble()));
        NarwhalDashboard.getInstance().addUpdate(narwhalDashboardKey(name), value::getAsDouble);
    }

    /**
     * Shows a true/false value that updates every loop.
     * @param name Name of the value on the dashboard
     * @param value Where the value comes from
     */
    public static void add(String name, BooleanSupplier value) {
        updaters.put(name, () -> SmartDashboard.putBoolean(name, value.getAsBoolean()));
        NarwhalDashboard.getInstance().addUpdate(narwhalDashboardKey(name), value::getAsBoolean);
    }

    /**
     * Shows text that updates every loop.
     * @param name Name of the value on the dashboard
     * @param value Where the value comes from
     */
    public static void add(String name, Supplier<String> value) {
        updaters.put(name, () -> SmartDashboard.putString(name, value.get()));
        NarwhalDashboard.getInstance().addUpdate(narwhalDashboardKey(name), value::get);
    }

    /**
     * Shows a button that runs a command.
     * @param name Name of the button on the dashboard
     * @param command Command to run when it's pressed
     */
    public static void add(String name, Command command) {
        add(name, (Sendable) command);
        NarwhalDashboard.getInstance().addButton(narwhalDashboardKey(name), (boolean pressed) -> {
            if (pressed) CommandScheduler.getInstance().schedule(command);
        });
    }

    /**
     * Shows something WPILib already knows how to draw on Elastic, e.g. a {@code Field2d} as a
     * field. NarwhalDashboard doesn't show these.
     * @param name Name of the widget on the dashboard
     * @param sendable What to show
     */
    public static void add(String name, Sendable sendable) {
        updaters.remove(name);
        SmartDashboard.putData(name, sendable);
    }

    /**
     * Shows a dropdown for picking the auto. On NarwhalDashboard it's the auto picker; an auto
     * picked there wins over the one picked on Elastic.
     * @param name Name of the dropdown on the dashboard
     * @param defaultAuto Name of the auto that runs if no one picks one
     * @param autos Each auto, by the name shown in the dropdown, in the order to show them
     * @return The auto that's picked
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

    /** The key NarwhalDashboard's web page knows a value by: the name after its last {@code /}. */
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
