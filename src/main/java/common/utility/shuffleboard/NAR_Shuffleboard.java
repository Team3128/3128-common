package common.utility.shuffleboard;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;


import common.core.misc.NAR_Robot;
import common.utility.Log;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Creates Shuffleboard widgets in a single call each and keeps them up to date.
 *
 * <ul>
 * <li>{@code addData(tab, name, () -> value, x, y)} shows a value that updates every loop.</li>
 * <li>{@code debug(...)} and {@code debugSwitch(...)} create boxes and switches that can be changed from
 *     Shuffleboard; they return a supplier that reads the latest value.</li>
 * <li>{@code addCommand(...)} creates a button that runs a command.</li>
 * </ul>
 *
 * <p>Positions are on a grid of {@link #WINDOW_WIDTH} by {@link #WINDOW_HEIGHT} per tab, counting
 * from 0 at the top left. A widget placed off the edge or on top of another logs a warning.
 * Adding a widget with a name already on the tab updates that widget rather than creating a
 * second one, so these methods are safe to call every loop.
 *
 * @since 2022 Rapid React
 * @author Mason Lam, Arav Chadha, Peter Ma
 */
@SuppressWarnings("unused")
public class NAR_Shuffleboard {

    static {
        NAR_Robot.addPeriodic(NAR_Shuffleboard::update, 0.02);
    }

    private NAR_Shuffleboard() {}

    /**
     * A widget and the source of its value.
     */
    private static class WidgetInfo {
        
        private SimpleWidget m_widget;
        private Supplier<Object> m_supply;
        private GenericEntry m_entry;
        
        /**
         * Creates a WidgetInfo.
         *
         * @param widget The widget containing the entry
         * @param supply The supplier updating the entry
         */
        public WidgetInfo(SimpleWidget widget, Supplier<Object> supply){
            m_widget = widget;
            m_supply = supply;
            m_entry = widget.getEntry();
        }

        /** Displays the supplier's latest value; has no effect for a widget with a fixed value. */
        public void update() {
            if(m_supply == null) return;
            m_entry.setValue(m_supply.get());
        }
    }

    /** The number of widgets that fit across a tab. */
    public static int WINDOW_WIDTH = 9;
    /** The number of widgets that fit down a tab. */
    public static int WINDOW_HEIGHT = 4;

    /** Every widget, by tab and then by name. */
    private static final HashMap<String, HashMap<String, WidgetInfo>> tabs = new HashMap<String, HashMap<String,WidgetInfo>>();
    /** Which grid spots on each tab have a widget, for autofill and the overlap warning. */
    private static final HashMap<String, boolean[][]> widgetPositions = new HashMap<String, boolean[][]>();

    private static SimpleWidget[] autoWidgets;
    private static String[] autoNames;
    private static String selectedAutoName;
    private static int prevAutoIndex;

    private static boolean shouldUpdate = true;
    
    /**
     * Creates a tab.
     *
     * @param tabName The title of the new tab
     */
    private static void create_tab(String tabName) {
        tabs.put(tabName, new HashMap<String,WidgetInfo>());
        widgetPositions.put(tabName, new boolean[WINDOW_WIDTH][WINDOW_HEIGHT]);
    }

    /**
     * Displays a value in Shuffleboard, at the next open position.
     *
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param data The value to display
     * @return The SimpleWidget, which can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Object data) {
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        for (int i = 0; i < WINDOW_HEIGHT; i++) {
            for (int j = 0; j < WINDOW_WIDTH; j++) {
                if (!widgetPositions.get(tabName)[j][i]) return addData(tabName, name, data, j, i, 1, 1);
            }
        }
        return addData(tabName, name, data, 0, 0, 1, 1);
    }

    /**
     * Displays a value in Shuffleboard, at the next open position.
     *
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param supply The value to display
     * @return The SimpleWidget, which can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply) {
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        for (int i = 0; i < WINDOW_HEIGHT; i++) {
            for (int j = 0; j < WINDOW_WIDTH; j++) {
                if (!widgetPositions.get(tabName)[j][i]) return addData(tabName, name, supply, j, i, 1, 1);
            }
        }
        return addData(tabName, name, supply, 0, 0, 1, 1);
    }

    /**
     * Displays a value in Shuffleboard.
     *
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param data The value to display
     * @param x X coordinate of the widget, starting from 0
     * @param y Y coordinate of the widget, starting from 0
     * @return The SimpleWidget, which can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Object data, int x, int y) {
        return addData(tabName, name, data, x, y, 1, 1);
    }

    /**
     * Displays an updating value in Shuffleboard.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param supply The supplier that updates the value
     * @param x X coordinate of the widget, starting from 0
     * @param y Y coordinate of the widget, starting from 0
     * @return The SimpleWidget, which can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply, int x, int y) {
        return addData(tabName, name, supply, x, y, 1, 1);
    }

    /**
     * Displays an updating value in Shuffleboard.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param supply The supplier that updates the value
     * @param x X coordinate of the widget, starting from 0
     * @param y Y coordinate of the widget, starting from 0
     * @param width Width of the widget
     * @param height Height of the widget
     * @return The SimpleWidget, which can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply, int x, int y, int width, int height){
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        if(tabs.get(tabName).containsKey(name)) {
            tabs.get(tabName).get(name).m_supply = supply;
            return tabs.get(tabName).get(name).m_widget;
        }
        placeWidget(tabName, name, x, y, width, height);
        SimpleWidget widget = Shuffleboard.getTab(tabName).add(name,supply.get()).withPosition(x, y).withSize(width, height);
        if(supply.get() instanceof Boolean) widget.withWidget(BuiltInWidgets.kBooleanBox);
        else widget.withWidget(BuiltInWidgets.kTextView);
        tabs.get(tabName).put(name, new WidgetInfo(widget,supply));
        return widget;
    }

    /**
    * Displays a value in Shuffleboard.
    *
    * @param tabName The title of the tab
    * @param name The name of the widget
    * @param data The value to display
    * @param x X coordinate of the widget, starting from 0
    * @param y Y coordinate of the widget, starting from 0
    * @param width Width of the widget
    * @param height Height of the widget
    * @return The SimpleWidget, which can be modified
    */
    public static SimpleWidget addData(String tabName, String name, Object data, int x, int y, int width, int height) {
        if(!tabs.containsKey(tabName)) create_tab(tabName);
        if (tabs.get(tabName).containsKey(name)) {
            tabs.get(tabName).get(name).m_entry.setValue(data);
            return tabs.get(tabName).get(name).m_widget;
        }
        placeWidget(tabName, name, x, y, width, height);
        SimpleWidget widget = Shuffleboard.getTab(tabName).add(name,data).withPosition(x, y).withSize(width,height).withWidget(BuiltInWidgets.kTextView);
        tabs.get(tabName).put(name, new WidgetInfo(widget,null));
        return widget;
    }

    /**
     * Displays a sendable value, such as a subsystem or command. Works on all classes that extend Sendable.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param data The sendable value to display
     * @param x X coordinate of the widget
     * @param y Y coordinate of the widget
     * @return The ComplexWidget, which can be modified
     */
    public static ComplexWidget addSendable(String tabName, String name, Sendable data, int x, int y) {
        if (data instanceof SubsystemBase) return addSendable(tabName, name, data, x, y, 2, 1);
        if (data instanceof PIDController) return addSendable(tabName, name, data, x, y, 1, 2);
        return addSendable(tabName, name, data, x, y, 1, 1);
    }

    /**
     * Displays a sendable value, such as a subsystem or command. Works on all classes that extend Sendable.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param data The sendable value to display
     * @param x X coordinate of the widget
     * @param y Y coordinate of the widget
     * @return The ComplexWidget, which can be modified
     */
    public static ComplexWidget addSendable(String tabName, String name, Sendable data, int x, int y, int width, int height) {
        try {
            if(!tabs.containsKey(tabName)) create_tab(tabName);
            final ComplexWidget widget = Shuffleboard.getTab(tabName).add(name, data).withPosition(x,y).withSize(width, height);
            placeWidget(tabName, name, x, y, width, height);
            return widget;
        }
        catch(Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Displays a command as a button.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param command The command to display
     * @param x X coordinate of the widget
     * @param y Y coordinate of the widget
     * @return The ComplexWidget, which can be modified
     */
    public static ComplexWidget addCommand(String tabName, String name, Command command, int x, int y) {
        return addSendable(tabName, name, command, x, y, 1, 1).withWidget(BuiltInWidgets.kCommand);
    }

    /**
     * Adds a video stream to Shuffleboard.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param cameraName The name of the camera
     * @param URL The URL of the stream
     * @param x X coordinate of the widget
     * @param y Y coordinate of the widget
     * @param width Width of the widget
     * @param height Height of the widget
     * @return The ComplexWidget, which can be modified
     */
    public static ComplexWidget addVideoStream(String tabName, String name, String cameraName, String URL, int x, int y, int width, int height) {
        try {
            if (!tabs.containsKey(tabName)) create_tab(tabName);

            ComplexWidget videoStream = Shuffleboard.getTab(tabName)
            .addCamera(name, cameraName, URL)
            .withProperties(Map.of("showControls", false))
            .withPosition(x, y)
            .withSize(width, height);
            placeWidget(tabName, name, x, y, width, height);
            return videoStream;
        }
        catch(Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Displays auto paths on Shuffleboard and updates the selected auto.
     * 
     * @param autos The names of all autos
     */
    public static void addAutos(String[] autos) {
        autoNames = autos;
        if (!tabs.containsKey("Autos")) create_tab("Autos");

        autoWidgets = new SimpleWidget[autoNames.length];
        ShuffleboardLayout autoLayout = Shuffleboard.getTab("Autos")
        .getLayout("Auto Names", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withProperties(Map.of("Label position", "HIDDEN"));

        for (int i = 0; i < autoNames.length; i++) { 
            autoWidgets[i] = autoLayout.add(autoNames[i], false).withWidget("Toggle Button");
        }
        addData("Autos", "Auto", ()-> (updateAutoSelection() == null) ? "null" : updateAutoSelection(), 2, 0);
    }

    /**
     * Updates selectedAutoName and the Shuffleboard auto selection layout.
     * 
     * @param autos The names of all autos
     */
    private static String updateAutoSelection() {
        boolean isSelected = false;
        for (int i = 0; i < autoNames.length; i++) {
            if (autoWidgets[i].getEntry().getBoolean(false)) {
                isSelected = true;
                if (i == prevAutoIndex) { continue; }
                if (prevAutoIndex != -1) { autoWidgets[prevAutoIndex].getEntry().setBoolean(false); }
                prevAutoIndex = i;
                selectedAutoName = autoNames[i];
            }
        }
        if (!isSelected) {
            selectedAutoName = null;
            prevAutoIndex = -1;
        }
        return selectedAutoName;
    }

    /**
     * Returns selectedAutoName.
     * 
     */
    public static String getSelectedAutoName() {
        return selectedAutoName;
    }

    /**
     * Creates a debug text widget, allowing a variable to be edited from Shuffleboard.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param Default The starting value for the widget
     * @param x X coordinate of the widget
     * @param y Y coordinate of the widget
     * @return DoubleSupplier containing the value in the widget
     */
    public static DoubleSupplier debug(String tabName, String name, double Default, int x, int y) {
        final GenericEntry tab = addData(tabName, name, Default, x, y).withWidget(BuiltInWidgets.kTextView).getEntry();
        return ()-> tab.getDouble(Default);
    }

    /**
     * Creates a debug switch widget, allowing a variable to be edited from Shuffleboard.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param Default The starting value for the widget
     * @param x X coordinate of the widget
     * @param y Y coordinate of the widget
     * @return BooleanSupplier containing the value in the widget
     */
    public static BooleanSupplier debugSwitch(String tabName, String name, boolean Default, int x, int y) {
        final GenericEntry tab = addData(tabName, name, Default, x, y).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        return ()-> tab.getBoolean(Default);
    }

    /**
     * Creates a debug button widget, allowing a variable to be edited from Shuffleboard.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @param Default The starting value for the widget
     * @param x X coordinate of the widget
     * @param y Y coordinate of the widget
     * @return BooleanSupplier containing the value in the widget
     */
    public static BooleanSupplier debugButton(String tabName, String name, boolean Default, int x, int y) {
        final GenericEntry tab = addData(tabName, name, Default, x, y).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        return ()-> tab.getBoolean(Default);
    }

    /**
     * Returns a reference to the value of a widget storing longs.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @return LongSupplier referencing the widget
     */
    public static LongSupplier getLong(String tabName, String name){
        final GenericEntry entry = tabs.get(tabName).get(name).m_entry;
        return ()-> entry.getInteger(0);
    }

    /**
     * Returns a reference to the value of a widget storing doubles.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @return DoubleSupplier referencing the widget
     */
    public static DoubleSupplier getDouble(String tabName, String name){
        final GenericEntry entry = tabs.get(tabName).get(name).m_entry;
        return ()-> entry.getDouble(0);
    }

    /**
     * Returns a reference to the value of a widget storing Booleans.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @return BooleanSupplier referencing the widget
     */
    public static BooleanSupplier getBoolean(String tabName, String name){
        final GenericEntry entry = tabs.get(tabName).get(name).m_entry;
        return ()-> entry.getBoolean(false);
    }

    /**
     * Returns a reference to the value of a widget storing Strings.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @return StringSupplier referencing the widget
     */
    public static Supplier<String> getString(String tabName, String name){
        final GenericEntry entry = tabs.get(tabName).get(name).m_entry;
        return ()-> entry.getString("");
    }

    /**
     * Returns the SimpleWidget object for a widget.
     * 
     * @param tabName The title of the tab
     * @param name The name of the widget
     * @return SimpleWidget stored in the widget
     */
    public static SimpleWidget getEntry(String tabName,String name) {
        return tabs.get(tabName).get(name).m_widget;
    }

    /**
     * Updates every widget.
     */
    public static void update() {
        if (!shouldUpdate) return;
        for(final String tab : tabs.keySet()){
            final HashMap<String, WidgetInfo> entries = tabs.get(tab);
            for(final WidgetInfo entry : entries.values()){
                entry.update();
            }
        }
    }

    /**
     * Records the position of a new widget on its tab so that autofill skips it, and logs a warning if it is off the
     * edge of the tab or on top of another widget.
     *
     * @param tabName The title of the tab
     * @param name The name of the widget, for the warning
     * @param x X coordinate of the widget
     * @param y Y coordinate of the widget
     * @param width Width of the widget
     * @param height Height of the widget
     */
    private static void placeWidget(String tabName, String name, int x, int y, int width, int height) {
        if (x < 0 || y < 0 || x + width > WINDOW_WIDTH || y + height > WINDOW_HEIGHT) {
            Log.unusual("Shuffleboard", name + " at (" + x + "," + y + ") is off the edge of tab " + tabName);
            return;
        }
        final boolean[][] taken = widgetPositions.get(tabName);
        boolean overlaps = false;
        for (int i = x; i < x + width; i++) {
            for (int j = y; j < y + height; j++) {
                overlaps |= taken[i][j];
                taken[i][j] = true;
            }
        }
        // A warning rather than an error, so that a crowded tab does not prevent the robot from starting
        if (overlaps) Log.unusual("Shuffleboard", name + " at (" + x + "," + y + ") overlaps another widget on tab " + tabName);
    }
}