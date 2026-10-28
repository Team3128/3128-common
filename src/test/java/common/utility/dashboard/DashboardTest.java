package common.utility.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.LinkedHashMap;
import java.util.function.Supplier;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import common.utility.narwhaldashboard.NarwhalDashboard;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

class DashboardTest {

    private static final double TOLERANCE = 1e-9;

    @BeforeAll
    static void startHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void updatingShowsEachSuppliersLatestValue() {
        final double[] number = {1};
        final boolean[] flag = {false};
        final String[] text = {"a"};
        Dashboard.add("DashboardTest/Number", () -> number[0]);
        Dashboard.add("DashboardTest/Flag", () -> flag[0]);
        Dashboard.add("DashboardTest/Text", () -> text[0]);

        number[0] = 7;
        flag[0] = true;
        text[0] = "b";
        Dashboard.update();

        assertEquals(7, SmartDashboard.getNumber("DashboardTest/Number", 0), TOLERANCE);
        assertTrue(SmartDashboard.getBoolean("DashboardTest/Flag", false));
        assertEquals("b", SmartDashboard.getString("DashboardTest/Text", ""));
    }

    @Test
    void addingTheSameNameAgainReplacesTheOldValue() {
        Dashboard.add("DashboardTest/Replaced", () -> 1.0);
        Dashboard.add("DashboardTest/Replaced", () -> 2.0);
        Dashboard.update();

        assertEquals(2, SmartDashboard.getNumber("DashboardTest/Replaced", 0), TOLERANCE);
    }

    @Test
    void chooserShowsItsOptions() {
        final SendableChooser<String> chooser = new SendableChooser<>();
        chooser.setDefaultOption("First", "first");
        chooser.addOption("Second", "second");
        Dashboard.add("DashboardTest/Chooser", chooser);
        SmartDashboard.updateValues();

        final String[] options = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard").getSubTable("DashboardTest").getSubTable("Chooser")
            .getEntry("options").getStringArray(new String[0]);
        assertEquals(2, options.length);
        assertEquals("first", chooser.getSelected());
    }

    @Test
    void pressingACommandsButtonSchedulesIt() {
        // Commands only start while the robot is enabled
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        final Command command = Commands.idle();
        Dashboard.add("DashboardTest/Button", command);
        SmartDashboard.updateValues();
        assertFalse(command.isScheduled());

        // What the dashboard does when the button is pressed
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard").getSubTable("DashboardTest").getSubTable("Button")
            .getEntry("running").setBoolean(true);
        SmartDashboard.updateValues();

        assertTrue(command.isScheduled());
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void pressingANarwhalButtonSchedulesItsCommandByTheLastPartOfItsName() {
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        final Command command = Commands.idle();
        Dashboard.add("DashboardTest/NarwhalButton", command);

        // What NarwhalDashboard's page sends when the button is pressed
        NarwhalDashboard.getInstance().onMessage(null, "button:NarwhalButton:true");

        assertTrue(command.isScheduled());
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void autoChooserPicksTheDefaultUntilNarwhalPicksAnother() {
        final Command first = Commands.none().withName("First");
        final Command second = Commands.none().withName("Second");
        final LinkedHashMap<String, Command> autos = new LinkedHashMap<>();
        autos.put("First", first);
        autos.put("Second", second);
        final Supplier<Command> selected = Dashboard.addAutoChooser("DashboardTest/Auto", "First", autos);

        assertSame(first, selected.get());

        NarwhalDashboard.getInstance().onMessage(null, "selectAuto:Second");
        assertSame(second, selected.get());

        // Picking an auto that doesn't exist clears NarwhalDashboard's pick
        NarwhalDashboard.getInstance().onMessage(null, "selectAuto:Missing");
        assertSame(first, selected.get());
    }
}
