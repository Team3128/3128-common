package common.utility.shuffleboard;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import common.core.misc.NAR_Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Team 3128's Wrapper class for {@link Shuffleboard}
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
     * Storage class for NAR_Shuffleboard
     */
    private static class WidgetInfo {
        
        private SimpleWidget m_widget;
        private Supplier<Object> m_supply;
        private GenericEntry m_entry;
        
        /**
         * Creates a new WidgetInfo
         *
         * @param widget widget containing the entry 
         * @param supply supplier updating the entry
         */
        public WidgetInfo(SimpleWidget widget, Supplier<Object> supply){
            m_widget = widget;
            m_supply = supply;
            m_entry = widget.getEntry();
        }

        public void update() {
            if(m_supply == null) return;
            m_entry.setValue(m_supply.get());
        }
    }

    public static int WINDOW_WIDTH = 9;
    public static int WINDOW_HEIGHT = 4;

    private static final HashMap<String, HashMap<String, WidgetInfo>> tabs = new HashMap<String, HashMap<String,WidgetInfo>>();
    private static final HashMap<String, boolean[][]> widgetPositions = new HashMap<String, boolean[][]>();

    private static SimpleWidget[] autoWidgets;
    private static String[] autoNames;
    private static String selectedAutoName;
    private static int prevAutoIndex;

    private static boolean shouldUpdate = true;
    
    /**
     * Creates a new tab
     *
     * @param tabName the title of the new tab
     */
    private static void create_tab(String tabName) {
        tabs.put(tabName, new HashMap<String,WidgetInfo>());
        widgetPositions.put(tabName, new boolean[WINDOW_WIDTH][WINDOW_HEIGHT]);
    }

    /**
     * Displays a value in Shuffleboard (Only used for autofill)
     *
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @param data the value to display
     * @return SimpleWidget that can be modified
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
     * Displays a value in Shuffleboard (Only used for autofill)
     *
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @param supply the value to display
     * @return SimpleWidget that can be modified
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
     * Displays a value in Shuffleboard
     *
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @param data the value to display
     * @param x coord of the widget starting from 0
     * @param y coord of the widget starting from 0
     * @return SimpleWidget that can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Object data, int x, int y) {
        return addData(tabName, name, data, x, y, 1, 1);
    }

    /**
     * Displays an updating value in Shuffleboard
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @param supply object supplier to constantly update value
     * @param x coord of the widget starting from 0
     * @param y coord of the widget starting from 0
     * @return SimpleWidget that can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply, int x, int y) {
        return addData(tabName, name, supply, x, y, 1, 1);
    }

    /**
     * Displays an updating value in Shuffleboard
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @param supply object supplier to constantly update value
     * @param x -coord of the widget starting from 0
     * @param y -coord of the widget starting from 0
     * @param width -of the widget
     * @param height -of the widget
     * @return simple widget that can be modified
     */
    public static SimpleWidget addData(String tabName, String name, Supplier<Object> supply, int x, int y, int width, int height){
        if(!tabs.containsKey(tabName)) { 
            create_tab(tabName);
            fillEntryPositions(x,y,width,height, tabName);
        }
        if(tabs.get(tabName).containsKey(name)) {
            tabs.get(tabName).get(name).m_supply = supply;
            return tabs.get(tabName).get(name).m_widget;
        }
        SimpleWidget widget = Shuffleboard.getTab(tabName).add(name,supply.get()).withPosition(x, y).withSize(width, height);
        if(supply.get() instanceof Boolean) widget.withWidget(BuiltInWidgets.kBooleanBox);
        else widget.withWidget(BuiltInWidgets.kTextView);
        tabs.get(tabName).put(name, new WidgetInfo(widget,supply));
        return widget;
    }

    /**
    * Displays a value in Shuffleboard
    *
    * @param tabName the title of the tab to select
    * @param name the name of the widget
    * @param data value to display
    * @param x -coord of the widget starting from 0
    * @param y -coord of the widget starting from 0
    * @param width -of the widget
    * @param height -of the widget
    * @return simple widget that can be modified
    */
    public static SimpleWidget addData(String tabName, String name, Object data, int x, int y, int width, int height) {
        if(!tabs.containsKey(tabName)) {
            create_tab(tabName);
            fillEntryPositions(x,y,width,height,tabName);
        }
        if (tabs.get(tabName).containsKey(name)) {
            tabs.get(tabName).get(name).m_entry.setValue(data);
            return tabs.get(tabName).get(name).m_widget;
        }
        SimpleWidget widget = Shuffleboard.getTab(tabName).add(name,data).withPosition(x, y).withSize(width,height);
        tabs.get(tabName).put(name, new WidgetInfo(widget,null));
        return widget;
    }

    /**
     * Displays sendable values, like subsystems and command, works on all classes that extend sendable
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @param data sendable value to display
     * @param x x-coord of the widget
     * @param y y-coord of the widget
     * @return sendable widget that can be modified
     */
    public static ComplexWidget addSendable(String tabName, String name, Sendable data, int x, int y) {
        if (data instanceof SubsystemBase) return addSendable(tabName, name, data, x, y, 2, 1);
        if (data instanceof PIDController) return addSendable(tabName, name, data, x, y, 1, 2);
        if (data instanceof WPI_PigeonIMU) return addSendable(tabName, name, data, x, y, 2, 2);
        return addSendable(tabName, name, data, x, y, 1, 1); // Default width and height
    }

    /**
     * Displays sendable values, like subsystems and command, works on all classes that extend sendable
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @param data sendable value to display
     * @param x x-coord of the widget
     * @param y y-coord of the widget
     * @return sendable widget that can be modified
     */
    public static ComplexWidget addSendable(String tabName, String name, Sendable data, int x, int y, int width, int height) {
        try {
            if(!tabs.containsKey(tabName)) {
                create_tab(tabName);
                fillEntryPositions(x, y, width, height, tabName);
            }
            return Shuffleboard.getTab(tabName).add(name, data).withPosition(x,y).withSize(width, height);
        }
        catch(Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Adds video stream to shuffleboard
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @param cameraName sendable value to display
     * @param URL x-coord of the widget
     * @param x coord of the widget
     * @param y coord of the widget
     * @param width y-coord of the widget
     * @param height y-coord of the widget
     * @return sendable widget that can be modified
     */
    public static ComplexWidget addVideoStream(String tabName, String name, String cameraName, String URL, int x, int y, int width, int height) {
        try {
            if (!tabs.containsKey(tabName)) {
                create_tab(tabName);
                fillEntryPositions(x, y, width, height, tabName);
            }

            ComplexWidget videoStream = Shuffleboard.getTab(tabName)
            .addCamera(name, cameraName, URL)
            .withProperties(Map.of("showControls", false))
            .withPosition(x, y)
            .withSize(width, height);            
            return videoStream;
        }
        catch(Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Displays auto paths on Shuffleboard and updates selected auto
     * 
     * @param autos a String array with all auto names
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
        addData("Autos", "Auto", ()-> (updateAutoSelection() == null) ? "null" : updateAutoSelection(), 2, 0); // Continuously runs updateAutoSelection()
    }

    /**
     * Updates selectedAutoName and shuffleboard auto selection layout
     * 
     * @param autos a String array with all auto names
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
     * Gets selectedAutoName
     * 
     */
    public static String getSelectedAutoName() {
        return selectedAutoName;
    }

    /**
     * Creates a debug widget, allows user to edit variable from Shuffleboard
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @param Default starting value for the widget
     * @param x x-coord of the widget
     * @param y y-coord of the widget
     * @return DoubleSupplier containing the value in the widget
     */
    public static DoubleSupplier debug(String tabName, String name, double Default, int x, int y) {
        final GenericEntry tab = addData(tabName, name, Default, x, y).withWidget(BuiltInWidgets.kTextView).getEntry();
        return ()-> tab.getDouble(Default);
    }

    /**
     * Get the value reference of a widget storing longs.
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @return LongSupplier referencing the widget
     */
    public static LongSupplier getLong(String tabName, String name){
        final GenericEntry entry = tabs.get(tabName).get(name).m_entry;
        return ()-> entry.getInteger(0);
    }

    /**
     * Get the value reference of a widget storing doubles.
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @return DoubleSupplier referencing the widget
     */
    public static DoubleSupplier getDouble(String tabName, String name){
        final GenericEntry entry = tabs.get(tabName).get(name).m_entry;
        return ()-> entry.getDouble(0);
    }

    /**
     * Get the value reference of a widget storing booleans.
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @return BooleanSupplier referencing the widget
     */
    public static BooleanSupplier getBoolean(String tabName, String name){
        final GenericEntry entry = tabs.get(tabName).get(name).m_entry;
        return ()-> entry.getBoolean(false);
    }

    /**
     * Get the value reference of a widget storing Strings.
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @return StringSupplier referencing the widget
     */
    public static Supplier<String> getString(String tabName, String name){
        final GenericEntry entry = tabs.get(tabName).get(name).m_entry;
        return ()-> entry.getString("");
    }

    /**
     * Get the Simple Widget object from an widget
     * 
     * @param tabName the title of the tab to select
     * @param name the name of the widget
     * @return SimpleWidget stored in the widget
     */
    public static SimpleWidget getEntry(String tabName,String name) {
        return tabs.get(tabName).get(name).m_widget;
    }

    /**
     * Updates every widget
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
     * Fills widget position array for a given tab
     * 
     * @param x x-coord of the widget
     * @param y y-coord of the widget
     * @param width width of the widget
     * @param height height of the widget
     * @param tabName the title of the tab to select
     */
    private static void fillEntryPositions(int x, int y, int width, int height, String tabName) {
        if (x + width > WINDOW_WIDTH || y + height > WINDOW_HEIGHT) { throw new IllegalArgumentException("Widget Position Out of Bounds (" + x + "," + y + ") at Tab: " + tabName); }
        for (int i = x; i < x + width; i++) {
            for (int j = y; j < y + height; j++) {
                if (widgetPositions.get(tabName)[i][j]) { throw new IllegalArgumentException("Widget Position Overlapping (" + i + "," + j + ") at Tab: " + tabName); }
                widgetPositions.get(tabName)[i][j] = true;
            }
        }
    }
}