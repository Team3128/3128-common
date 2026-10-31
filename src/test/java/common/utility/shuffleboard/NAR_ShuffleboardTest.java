package common.utility.shuffleboard;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.DoubleSupplier;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

class NAR_ShuffleboardTest {

    private static final double TOLERANCE = 1e-9;
    private static int nextTab = 0;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    /** Each test gets its own tab, since widget positions are tracked per tab. */
    private static String newTab() {
        return "NAR_ShuffleboardTest" + nextTab++;
    }

    @Test
    void updatingWidgetShowsItsSuppliersLatestValue() {
        final String tab = newTab();
        final double[] value = {1};
        NAR_Shuffleboard.addData(tab, "Value", () -> value[0], 0, 0);

        value[0] = 7;
        NAR_Shuffleboard.update();

        assertEquals(7, NAR_Shuffleboard.getDouble(tab, "Value").getAsDouble(), TOLERANCE);
    }

    @Test
    void addingTheSameNameAgainReusesTheWidget() {
        final String tab = newTab();
        // Doubles, since NetworkTables won't read an integer entry as a double
        final SimpleWidget first = NAR_Shuffleboard.addData(tab, "Value", 1.0, 0, 0);
        final SimpleWidget second = NAR_Shuffleboard.addData(tab, "Value", 2.0, 0, 0);

        assertSame(first, second);
        assertEquals(2, NAR_Shuffleboard.getDouble(tab, "Value").getAsDouble(), TOLERANCE);
    }

    @Test
    void debugWidgetStartsAtItsDefault() {
        final DoubleSupplier kP = NAR_Shuffleboard.debug(newTab(), "kP", 0.3, 0, 0);
        assertEquals(0.3, kP.getAsDouble(), TOLERANCE);
    }

    @Test
    void misplacedWidgetsAreStillAdded() {
        final String tab = newTab();
        // Off the edge, and on top of the first widget: both only warn, so the robot still starts
        assertNotNull(NAR_Shuffleboard.addData(tab, "Off the edge", 1.0, NAR_Shuffleboard.WINDOW_WIDTH, 0));
        NAR_Shuffleboard.addData(tab, "First", 1.0, 0, 0);
        assertNotNull(NAR_Shuffleboard.addData(tab, "On top", 2.0, 0, 0));
    }

    @Test
    void autofillSkipsEveryTakenSpotNotJustTheFirst() {
        final String tab = newTab();
        NAR_Shuffleboard.addData(tab, "A", 1.0, 0, 0);
        NAR_Shuffleboard.addData(tab, "B", 1.0, 1, 0);

        NAR_Shuffleboard.addData(tab, "C", 1.0);
        Shuffleboard.update();

        // The next free spot along the top row
        final double[] position = NetworkTableInstance.getDefault()
            .getTable("Shuffleboard").getSubTable(".metadata").getSubTable(tab).getSubTable("C")
            .getEntry("Position").getDoubleArray(new double[0]);
        assertArrayEquals(new double[] {2, 0}, position, TOLERANCE);
    }

    @Test
    void updatingValuesCanBeAutofilledAndReplaced() {
        final String tab = newTab();
        final double[] value = {1};
        NAR_Shuffleboard.addData(tab, "Value", () -> value[0]);

        // Adding it again replaces where its value comes from
        NAR_Shuffleboard.addData(tab, "Value", () -> 9.0, 0, 0);
        NAR_Shuffleboard.update();
        assertEquals(9, NAR_Shuffleboard.getDouble(tab, "Value").getAsDouble(), TOLERANCE);

        // A true/false value gets a light
        NAR_Shuffleboard.addData(tab, "Ready", () -> true, 1, 0);
        NAR_Shuffleboard.update();
        assertTrue(NAR_Shuffleboard.getBoolean(tab, "Ready").getAsBoolean());
    }

    @Test
    void readersGetEachKindOfValue() {
        final String tab = newTab();
        NAR_Shuffleboard.addData(tab, "Count", 3L, 0, 0);
        NAR_Shuffleboard.addData(tab, "Name", "Narwhal", 1, 0);
        NAR_Shuffleboard.addData(tab, "On", true, 2, 0);

        assertEquals(3, NAR_Shuffleboard.getLong(tab, "Count").getAsLong());
        assertEquals("Narwhal", NAR_Shuffleboard.getString(tab, "Name").get());
        assertTrue(NAR_Shuffleboard.getBoolean(tab, "On").getAsBoolean());
        assertNotNull(NAR_Shuffleboard.getEntry(tab, "Count"));
    }

    @Test
    void switchesAndButtonsStartAtTheirDefault() {
        final String tab = newTab();
        assertTrue(NAR_Shuffleboard.debugSwitch(tab, "Switch", true, 0, 0).getAsBoolean());
        assertFalse(NAR_Shuffleboard.debugButton(tab, "Button", false, 1, 0).getAsBoolean());
    }

    @Test
    void sendablesCommandsAndVideoAreAdded() {
        final String tab = newTab();
        assertNotNull(NAR_Shuffleboard.addSendable(tab, "Subsystem", new SubsystemBase() {}, 0, 0));
        assertNotNull(NAR_Shuffleboard.addSendable(tab, "PID", new PIDController(1, 0, 0), 2, 0));
        assertNotNull(NAR_Shuffleboard.addSendable(tab, "Other", new Field2d(), 3, 0));
        assertNotNull(NAR_Shuffleboard.addCommand(tab, "Command", Commands.none(), 4, 0));
        assertNotNull(NAR_Shuffleboard.addVideoStream(tab, "Camera", "front", "http://camera:1181", 5, 0, 2, 2));

        // A second sendable with the same name fails, and gives null instead of crashing
        assertNull(NAR_Shuffleboard.addSendable(tab, "Subsystem", new SubsystemBase() {}, 0, 1));
        assertNull(NAR_Shuffleboard.addVideoStream(tab, "Camera", "front", "http://camera:1181", 5, 2, 2, 2));
    }

    @Test
    void autoChooserKeepsOneAutoSelected() {
        NAR_Shuffleboard.addAutos(new String[] {"Left", "Right"});
        final NetworkTable choices = NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getSubTable("Autos").getSubTable("Auto Names");

        NAR_Shuffleboard.update();
        assertNull(NAR_Shuffleboard.getSelectedAutoName());

        choices.getEntry("Left").setBoolean(true);
        NAR_Shuffleboard.update();
        assertEquals("Left", NAR_Shuffleboard.getSelectedAutoName());

        // Picking another turns the first off
        choices.getEntry("Right").setBoolean(true);
        NAR_Shuffleboard.update();
        assertEquals("Right", NAR_Shuffleboard.getSelectedAutoName());
        assertFalse(choices.getEntry("Left").getBoolean(true));

        choices.getEntry("Right").setBoolean(false);
        NAR_Shuffleboard.update();
        assertNull(NAR_Shuffleboard.getSelectedAutoName());
    }
}
