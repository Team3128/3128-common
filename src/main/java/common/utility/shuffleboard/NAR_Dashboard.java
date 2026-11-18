package common.utility.shuffleboard;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import common.core.misc.NAR_Robot;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Team 3128's NetworkTables wrapper for dashboards such as Elastic.
 * Replaces NAR_Shuffleboard.
 *
 * <p>Everything is published under {@code /SmartDashboard/<tabName>/<name>}.
 * Elastic has no concept of tabs or widget positions on the robot side, so
 * layout (tabs, widget types, positions, sizes) is configured in the Elastic
 * app itself. The x/y/width/height overloads are kept only so existing call
 * sites keep compiling; those arguments are ignored.
 *
 * @since 2026 
 */
public class NAR_Dashboard {

    /**
     * Root NetworkTables table. Must stay "SmartDashboard" so that Sendables
     * published through {@link SmartDashboard} land in the same place as plain values.
     */
    public static final String ROOT = "SmartDashboard";

    static {
        NAR_Robot.addPeriodic(NAR_Dashboard::update, 0.02);
    }

    private NAR_Dashboard() {}

    /**
     * Storage class for NAR_Dashboard
     */
    private static class WidgetInfo {

        private final GenericEntry m_entry;
        private Supplier<?> m_supply;

        /**
         * Creates a new WidgetInfo and the NetworkTables entry behind it
         *d
         * @param tabName the group the entry lives in
         * @param name the name of the entry
         */
        public WidgetInfo(String tabName, String name) {
            m_entry = NetworkTableInstance.getDefault()
                .getTopic("/" + ROOT + "/" + tabName + "/" + name)
                .getGenericEntry();
        }

        public void update() {
            if (m_supply == null) return;
            publish(m_entry, m_supply.get());
        }
    }

    private static final HashMap<String, HashMap<String, WidgetInfo>> tabs = new HashMap<>();

    private static SendableChooser<String> autoChooser;

    /**
     * Gets (or lazily creates) the WidgetInfo for a tab/name pair
     */
    private static WidgetInfo getInfo(String tabName, String name) {
        return tabs
            .computeIfAbsent(tabName, t -> new HashMap<>())
            .computeIfAbsent(name, n -> new WidgetInfo(tabName, n));
    }

    /**
     * Writes a value to an entry. Types NetworkTables can't hold (Pose2d, enums, etc.)
     * are published as their toString().
     */
    private static void publish(GenericEntry entry, Object value) {
        if (value == null) {
            entry.setValue("null");
            return;
        }
        try {
            entry.setValue(value);
        } catch (IllegalArgumentException e) {
            entry.setValue(String.valueOf(value));
        }
    }

    // ------------------------------------------------------------------
    // Plain values
    // ------------------------------------------------------------------

    /**
     * Publishes a value to the dashboard
     *
     * @param tabName the group to publish under
     * @param name the name of the entry
     * @param data the value to publish
     * @return the underlying entry
     */
    public static GenericEntry addData(String tabName, String name, Object data) {
        WidgetInfo info = getInfo(tabName, name);
        publish(info.m_entry, data);
        return info.m_entry;
    }

    /**
     * Publishes a continuously updating value to the dashboard
     *
     * @param tabName the group to publish under
     * @param name the name of the entry
     * @param supply supplier polled every 20ms to update the value
     * @return the underlying entry
     */
    public static GenericEntry addData(String tabName, String name, Supplier<?> supply) {
        WidgetInfo info = getInfo(tabName, name);
        info.m_supply = supply;
        info.update();
        return info.m_entry;
    }

    /** Position arguments are ignored; layout is set in Elastic. */
    public static GenericEntry addData(String tabName, String name, Object data, int x, int y) {
        return addData(tabName, name, data);
    }

    /** Position arguments are ignored; layout is set in Elastic. */
    public static GenericEntry addData(String tabName, String name, Supplier<?> supply, int x, int y) {
        return addData(tabName, name, supply);
    }

    /** Position and size arguments are ignored; layout is set in Elastic. */
    public static GenericEntry addData(String tabName, String name, Object data, int x, int y, int width, int height) {
        return addData(tabName, name, data);
    }

    /** Position and size arguments are ignored; layout is set in Elastic. */
    public static GenericEntry addData(String tabName, String name, Supplier<?> supply, int x, int y, int width, int height) {
        return addData(tabName, name, supply);
    }

    // ------------------------------------------------------------------
    // Sendables / commands
    // ------------------------------------------------------------------

    /**
     * Publishes a Sendable (subsystems, commands, PID controllers, choosers, etc.)
     * under {@code /SmartDashboard/<tabName>/<name>}. Elastic picks the matching widget
     * from the Sendable's type.
     *
     * @param tabName the group to publish under
     * @param name the name of the entry
     * @param data the sendable to publish
     */
    public static void addSendable(String tabName, String name, Sendable data) {
        SmartDashboard.putData(tabName + "/" + name, data);
    }

    /** Position arguments are ignored; layout is set in Elastic. */
    public static void addSendable(String tabName, String name, Sendable data, int x, int y) {
        addSendable(tabName, name, data);
    }

    /** Position and size arguments are ignored; layout is set in Elastic. */
    public static void addSendable(String tabName, String name, Sendable data, int x, int y, int width, int height) {
        addSendable(tabName, name, data);
    }

    /**
     * Publishes a command so it can be run from a Command widget in Elastic
     *
     * @param tabName the group to publish under
     * @param name the name of the entry
     * @param command command to publish
     */
    public static void addCommand(String tabName, String name, Command command) {
        addSendable(tabName, name, command);
    }

    /** Position arguments are ignored; layout is set in Elastic. */
    public static void addCommand(String tabName, String name, Command command, int x, int y) {
        addCommand(tabName, name, command);
    }

    // ------------------------------------------------------------------
    // Camera
    // ------------------------------------------------------------------

    /**
     * Publishes an MJPEG stream URL to the standard CameraPublisher table.
     * In Elastic, add a Camera Stream widget and point it at the same URL
     * (or the stream listed under CameraPublisher).
     *
     * @param tabName unused, kept for compatibility
     * @param name unused, kept for compatibility
     * @param cameraName the camera's name in CameraPublisher
     * @param URL the MJPEG stream URL
     */
    public static void addVideoStream(String tabName, String name, String cameraName, String URL) {
        NetworkTableInstance.getDefault()
            .getTable("CameraPublisher")
            .getSubTable(cameraName)
            .getEntry("streams")
            .setStringArray(new String[] {"mjpg:" + URL});
    }

    /** Position and size arguments are ignored; layout is set in Elastic. */
    public static void addVideoStream(String tabName, String name, String cameraName, String URL, int x, int y, int width, int height) {
        addVideoStream(tabName, name, cameraName, URL);
    }

    // ------------------------------------------------------------------
    // Autos
    // ------------------------------------------------------------------

    /**
     * Publishes an auto chooser. In Elastic, add a ComboBox Chooser widget
     * pointed at {@code /SmartDashboard/Autos/Auto Chooser}.
     *
     * @param autos a String array with all auto names
     */
    public static void addAutos(String[] autos) {
        autoChooser = new SendableChooser<>();
        for (String auto : autos) {
            autoChooser.addOption(auto, auto);
        }
        addSendable("Autos", "Auto Chooser", autoChooser);
    }

    /**
     * Gets the selected auto name
     *
     * @return the selected auto, or null if nothing is selected
     */
    public static String getSelectedAutoName() {
        return autoChooser == null ? null : autoChooser.getSelected();
    }

    // ------------------------------------------------------------------
    // Editable (debug) values
    // ------------------------------------------------------------------

    /**
     * Creates an editable number entry, allows the user to edit a variable from the dashboard
     *
     * @param tabName the group to publish under
     * @param name the name of the entry
     * @param Default starting value
     * @return DoubleSupplier containing the current value
     */
    public static DoubleSupplier debug(String tabName, String name, double Default) {
        final GenericEntry entry = getInfo(tabName, name).m_entry;
        entry.setDefaultValue(Default);
        return () -> entry.getDouble(Default);
    }

    /** Position arguments are ignored; layout is set in Elastic. */
    public static DoubleSupplier debug(String tabName, String name, double Default, int x, int y) {
        return debug(tabName, name, Default);
    }

    /**
     * Creates an editable boolean entry (use a Toggle Switch widget in Elastic)
     *
     * @param tabName the group to publish under
     * @param name the name of the entry
     * @param Default starting value
     * @return BooleanSupplier containing the current value
     */
    public static BooleanSupplier debugSwitch(String tabName, String name, boolean Default) {
        final GenericEntry entry = getInfo(tabName, name).m_entry;
        entry.setDefaultValue(Default);
        return () -> entry.getBoolean(Default);
    }

    /** Position arguments are ignored; layout is set in Elastic. */
    public static BooleanSupplier debugSwitch(String tabName, String name, boolean Default, int x, int y) {
        return debugSwitch(tabName, name, Default);
    }

    /**
     * Creates an editable boolean entry (use a Toggle Button widget in Elastic)
     *
     * @param tabName the group to publish under
     * @param name the name of the entry
     * @param Default starting value
     * @return BooleanSupplier containing the current value
     */
    public static BooleanSupplier debugButton(String tabName, String name, boolean Default) {
        return debugSwitch(tabName, name, Default);
    }

    /** Position arguments are ignored; layout is set in Elastic. */
    public static BooleanSupplier debugButton(String tabName, String name, boolean Default, int x, int y) {
        return debugSwitch(tabName, name, Default);
    }

    // ------------------------------------------------------------------
    // Reading values back
    // ------------------------------------------------------------------

    /**
     * Get the value reference of an entry storing longs.
     *
     * @param tabName the group the entry lives in
     * @param name the name of the entry
     * @return LongSupplier referencing the entry
     */
    public static LongSupplier getLong(String tabName, String name) {
        final GenericEntry entry = getInfo(tabName, name).m_entry;
        return () -> entry.getInteger(0);
    }

    /**
     * Get the value reference of an entry storing doubles.
     *
     * @param tabName the group the entry lives in
     * @param name the name of the entry
     * @return DoubleSupplier referencing the entry
     */
    public static DoubleSupplier getDouble(String tabName, String name) {
        final GenericEntry entry = getInfo(tabName, name).m_entry;
        return () -> entry.getDouble(0);
    }

    /**
     * Get the value reference of an entry storing booleans.
     *
     * @param tabName the group the entry lives in
     * @param name the name of the entry
     * @return BooleanSupplier referencing the entry
     */
    public static BooleanSupplier getBoolean(String tabName, String name) {
        final GenericEntry entry = getInfo(tabName, name).m_entry;
        return () -> entry.getBoolean(false);
    }

    /**
     * Get the value reference of an entry storing Strings.
     *
     * @param tabName the group the entry lives in
     * @param name the name of the entry
     * @return Supplier referencing the entry
     */
    public static Supplier<String> getString(String tabName, String name) {
        final GenericEntry entry = getInfo(tabName, name).m_entry;
        return () -> entry.getString("");
    }

    /**
     * Get the underlying NetworkTables entry
     *
     * @param tabName the group the entry lives in
     * @param name the name of the entry
     * @return the GenericEntry
     */
    public static GenericEntry getEntry(String tabName, String name) {
        return getInfo(tabName, name).m_entry;
    }

    // ------------------------------------------------------------------
    // Periodic
    // ------------------------------------------------------------------

    /**
     * Updates every supplier-backed entry
     */
    public static void update() {
        for (final HashMap<String, WidgetInfo> entries : tabs.values()) {
            for (final WidgetInfo entry : entries.values()) {
                entry.update();
            }
        }
    }
}