package common.core.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.MockMotor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;

class IOLoggerTest {

    @BeforeAll
    static void startEverything() {
        assertTrue(HAL.initialize(500, 0));
        Logger.AdvancedHooks.disableRobotBaseCheck();
        Logger.disableConsoleCapture();
        Logger.addDataReceiver(new WPILOGWriter("build/iolog-test.wpilog"));
        Logger.start();
    }

    @Test
    void mechanismValuesReachTheLogFile() throws Exception {
        final MockMotor motor = MockMotor.talonLike(1);
        final MotorConfig config = new MotorConfig(0.01, 60, 40, false, Neutral.BRAKE);
        final LinearMechanism slider = new LinearMechanism("IOLoggerTestSlider", config,
            new PIDFFConfig(0.1), 0.01, 0, 1, 1, Math.PI / 2, 5, DCMotor.getKrakenX60(1), motor);

        motor.rawPosition = 0.3 / 0.01;
        for (int i = 0; i < 20; i++) {
            Logger.AdvancedHooks.invokePeriodicBeforeUser();
            slider.periodic();
            Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0);
            Thread.sleep(20);
        }

        // The writer saves on its own thread; wait for the file to fill
        final File logFile = new File("build/iolog-test.wpilog");
        for (int i = 0; i < 50 && logFile.length() == 0; i++) Thread.sleep(100);

        Logger.end();
        Thread.sleep(1000); // give the writer a moment to save the file
        assertTrue(logFile.length() > 0);

    }
}
