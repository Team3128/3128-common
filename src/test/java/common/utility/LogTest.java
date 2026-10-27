package common.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import common.utility.Log.Type;
import edu.wpi.first.hal.HAL;

class LogTest {

    private final ByteArrayOutputStream output = new ByteArrayOutputStream();
    private PrintStream originalOut;

    @BeforeAll
    static void startHal() {
        // Errors are also sent to the Driver Station
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void captureOutput() {
        originalOut = System.out;
        System.setOut(new PrintStream(output, true));
    }

    @AfterEach
    void restore() {
        System.setOut(originalOut);
        Log.logDebug = false;
        Type.disable(Type.values());
    }

    private String printed() {
        return output.toString().replace("\r", "");
    }

    @Test
    void infoPrintsSeverityCategoryAndMessage() {
        Log.info("Shooter", "Ready");
        assertEquals("[Info] [Shooter] Ready\n", printed());
    }

    @Test
    void infoCanPrintANumber() {
        Log.info("Speed", 2.5);
        assertEquals("[Info] [Speed] 2.5\n", printed());
    }

    @Test
    void errorsAreLabelled() {
        Log.unusual("A", "odd");
        Log.recoverable("B", "broke");
        assertTrue(printed().contains("[Unusual] [A] odd"));
        assertTrue(printed().contains("[Recoverable] [B] broke"));
    }

    @Test
    void debugIsHiddenUntilDebugLoggingAndItsTypeAreBothOn() {
        Log.debug(Type.MECHANISM, "Slider", "hidden");
        Log.logDebug = true;
        Log.debug(Type.MECHANISM, "Slider", "still hidden, type is off");
        Type.MECHANISM.enable();
        Log.debug(Type.MECHANISM, "Slider", "shown");

        assertEquals("\t\t\t\t[Debug] [Slider] shown\n", printed());
    }

    @Test
    void stateMachineDebugIsIndentedOneMoreLevelOutsideTheRobot() {
        Log.logDebug = true;
        Type.STATE_MACHINE_PRIMARY.enable();

        Log.debug(Type.STATE_MACHINE_PRIMARY, "Robot", "top");
        Log.debug(Type.STATE_MACHINE_PRIMARY, "Intake", "nested");

        assertEquals("\t\t[Debug] [Robot] top\n\t\t\t\t[Debug] [Intake] nested\n", printed());
    }

    @Test
    void profileReportsHowLongSomethingTook() {
        final boolean[] ran = {false};
        Log.profile("Startup", () -> ran[0] = true);
        assertTrue(ran[0]);
        assertTrue(printed().startsWith("[Info] [Startup] "));
    }
}
