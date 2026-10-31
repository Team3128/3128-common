package common.core.misc;

import java.util.PriorityQueue;

import common.hardware.motorcontroller.NAR_CANSpark;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * Base class for Team 3128's {@code Robot} class. It behaves like WPILib's {@code TimedRobot}:
 * subclasses override {@code robotPeriodic()}, {@code teleopPeriodic()} and similar methods, which run every 20 ms.
 *
 * <p>It also:
 * <ul>
 * <li>Records a log file of NetworkTables, the console and the Driver Station for AdvantageScope,
 *     on a USB stick in the roboRIO if one is present.</li>
 * <li>Saves every Spark's settings to flash once, immediately after {@code robotInit()}.</li>
 * <li>Allows any class to schedule periodic work with {@link #addPeriodic(Runnable, double)}, for
 *     example, Shuffleboard updates, without the robot code having to call it.</li>
 * </ul>
 *
 * <p>Everything runs on one thread, in order, so code called from {@code addPeriodic} never runs
 * at the same time as robot code.
 *
 * @since 2023 Charged Up
 * @author Mason Lam, Wallim
 */
public class NAR_Robot extends IterativeRobotBase {

    @SuppressWarnings("MemberName")
    static class Callback implements Comparable<Callback> {
        public Runnable func;
        public double period;
        public double expirationTime;

        /**
         * Constructs a callback container.
         *
         * @param func             The callback to run.
         * @param startTimeSeconds The common starting point for all callback scheduling
         *                         in seconds.
         * @param periodSeconds    The period at which to run the callback in seconds.
         * @param offsetSeconds    The offset from the common starting time in seconds.
         */
        Callback(Runnable func, double startTimeSeconds, double periodSeconds, double offsetSeconds) {
            this.func = func;
            this.period = periodSeconds;
            this.expirationTime = startTimeSeconds
                    + offsetSeconds
                    + Math.floor((Timer.getFPGATimestamp() - startTimeSeconds) / this.period)
                            * this.period
                    + this.period;
        }

        @Override
        public boolean equals(Object rhs) {
            if (rhs instanceof Callback) {
                return Double.compare(expirationTime, ((Callback) rhs).expirationTime) == 0;
            }
            return false;
        }

        @Override
        public int hashCode() {
            return Double.hashCode(expirationTime);
        }

        @Override
        public int compareTo(Callback rhs) {
            // Elements with sooner expiration times are sorted as lesser. The head of
            // Java's PriorityQueue is the least element.
            return Double.compare(expirationTime, rhs.expirationTime);
        }
    }

    public static final double kDefaultPeriod = 0.02;

    /** Handle to the native notifier, passed to the JNI bindings. */
    private final int m_notifier = NotifierJNI.initializeNotifier();

    private static double m_startTime;

    private static final PriorityQueue<Callback> m_callbacks = new PriorityQueue<>();


    /** Creates a robot with the default 20 ms period. */
    protected NAR_Robot() {
        this(kDefaultPeriod);
    }

    /**
     * Creates a robot and starts recording NetworkTables, console output and Driver Station inputs
     * to a .wpilog file, on a USB stick in the roboRIO if one is present, or in {@code logs/} in
     * simulation.
     *
     * @param period Period in seconds.
     */
    protected NAR_Robot(double period) {
        super(period);
        m_startTime = Timer.getFPGATimestamp();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());


        addPeriodic(this::loopFunc, period);

        
        NotifierJNI.setNotifierName(m_notifier, "NAR_Robot");

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
    }

    @Override
    public void close() {
        NotifierJNI.stopNotifier(m_notifier);
        NotifierJNI.cleanNotifier(m_notifier);
    }

    /** Provides an alternate main loop through {@code startCompetition()}. */
    @Override
    public void startCompetition() {
        
        robotInit();
        // Every Spark is configured at this point, so each one's settings are saved once, as REV recommends
        NAR_CANSpark.burnFlashAll();

        if (isSimulation()) {
            simulationInit();
        }


        // Tell the Driver Station that the robot is ready to be enabled
        System.out.println("********** Robot program startup complete **********");
        DriverStationJNI.observeUserProgramStarting();

        while (true) {
            // The queue does not need an emptiness check because it always holds at least one element (the
            // constructor adds one), and each element is re-enqueued
            // at the end of the loop.
            var callback = m_callbacks.poll();

            NotifierJNI.updateNotifierAlarm(m_notifier, (long) (callback.expirationTime * 1e6));

            long curTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
            if (curTime == 0) {
                break;
            }

            try {
                callback.func.run();
            } catch (Exception e) {
                DriverStation.reportError(e.toString(), e.getStackTrace());
            }

            callback.expirationTime += callback.period;
            m_callbacks.add(callback);

            while ((long) (m_callbacks.peek().expirationTime * 1e6) <= curTime) {
                callback = m_callbacks.poll();

                try {
                    callback.func.run();
                } catch (Exception e) {
                    DriverStation.reportError(e.toString(), e.getStackTrace());
                }

                callback.expirationTime += callback.period;
                m_callbacks.add(callback);
            }
        }
    }

    /** Ends the main loop in startCompetition(). */
    @Override
    public void endCompetition() {
        NotifierJNI.stopNotifier(m_notifier);
    }

    /**
     * Adds a callback to run at a specific period.
     *
     * <p>
     * This is scheduled on the robot's Notifier, so the robot loop and the callback
     * run
     * synchronously. Interactions between them are thread-safe.
     *
     * @param callback      The callback to run.
     * @param periodSeconds The period at which to run the callback in seconds.
     */
    public static void addPeriodic(Runnable callback, double periodSeconds) {
        m_callbacks.add(new Callback(callback, m_startTime, periodSeconds, 0.0));
    }

    /**
     * Adds a callback to run at a specific period with a starting time offset.
     *
     * <p>
     * This is scheduled on the robot's Notifier, so the robot loop and the callback
     * run
     * synchronously. Interactions between them are thread-safe.
     *
     * @param callback      The callback to run.
     * @param periodSeconds The period at which to run the callback in seconds.
     * @param offsetSeconds The offset from the common starting time in seconds.
     *                      This is useful for
     *                      scheduling a callback in a different timeslot relative
     *                      to the robot loop.
     */
    public static void addPeriodic(Runnable callback, double periodSeconds, double offsetSeconds) {
        m_callbacks.add(new Callback(callback, m_startTime, periodSeconds, offsetSeconds));
    }
}