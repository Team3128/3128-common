package common.utility.shuffleboard;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.DoubleSupplier;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
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
    void widgetOutsideTheTabIsRejected() {
        assertThrows(IllegalArgumentException.class,
            () -> NAR_Shuffleboard.addData(newTab(), "Off the edge", 1, NAR_Shuffleboard.WINDOW_WIDTH, 0));
    }
}
