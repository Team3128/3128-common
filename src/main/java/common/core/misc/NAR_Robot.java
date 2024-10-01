package common.core.misc;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.reflect.Method;
import java.time.LocalDateTime;
import java.util.List;
import java.util.PriorityQueue;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * Team 3128's Custom Robot class
 * 
 * <p> Includes AdvantageKit and addPeriodic()
 * 
 * <p>NOTES:
 * <ul>
 * <li> TimedRobot implements the IterativeRobotBase robot program framework.
 * <li> The TimedRobot class is intended to be subclassed by a user creating a robot program.
 * <li> Periodic() functions from the base class are called on an interval by a Notifier instance.
 * <li> AdvantageKit logging is disabled by default. 
 * </ul>
 * 
 * <p> ----------------------------------------------
 *  
 * @since 2023 Charged Up
 * @author Mason Lam, Wallim
 */
public class NAR_Robot extends IterativeRobotBase {

    public static boolean logWithAdvantageKit = false;

    @SuppressWarnings("MemberName")
    static class Callback implements Comparable<Callback> {
        public Runnable func;
        public double period;
        public double expirationTime;

        /**
         * Construct a callback container.
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

    // The C pointer to the notifier object. We don't use it directly, it is
    // just passed to the JNI bindings.
    private final int m_notifier = NotifierJNI.initializeNotifier();

    private static double m_startTime;

    private static final PriorityQueue<Callback> m_callbacks = new PriorityQueue<>();

    private Method periodicAfterUser0 = null;

    private GcStatsCollector gcStatsCollector = new GcStatsCollector();

    /** Constructor for TimedRobot. */
    protected NAR_Robot() {
        this(kDefaultPeriod);
    }

    /**
     * Constructor for TimedRobot.
     *
     * @param period Period in seconds.
     */
    protected NAR_Robot(double period) {
        super(period);
        m_startTime = Timer.getFPGATimestamp();

        if(logWithAdvantageKit) {
            Method periodicBeforeUser = null; // Method to get the periodicBeforeUser method from Logger
            Method periodicAfterUser = null; // Method to get the periodicAfterUser method from Logger
            try {
                periodicBeforeUser = Logger.class.getDeclaredMethod("periodicBeforeUser");
                periodicAfterUser = Logger.class.getDeclaredMethod("periodicAfterUser", long.class, long.class);
            } catch (NoSuchMethodException | SecurityException e) {
                e.printStackTrace();
            }

            periodicBeforeUser.setAccessible(true); // set the method to be accessible
            periodicAfterUser.setAccessible(true); // set the method to be accessible

            Method periodicBeforeUser0 = periodicBeforeUser;
            periodicAfterUser0 = periodicAfterUser;

            addPeriodic(() -> {
                try {
                    long loopCycleStart = Logger.getRealTimestamp();
                    if (logWithAdvantageKit) periodicBeforeUser0.invoke(null);
                    long userCodeStart = Logger.getRealTimestamp();
                    loopFunc();
                    long loopCycleEnd = Logger.getRealTimestamp();
                    if (logWithAdvantageKit) gcStatsCollector.update();
                    if (logWithAdvantageKit) periodicAfterUser0.invoke(null, loopCycleEnd - userCodeStart, userCodeStart - loopCycleStart);
                } catch (Exception e) {
                }
            }, period);

        } else {
            addPeriodic(this::loopFunc, period);
        }

        
        NotifierJNI.setNotifierName(m_notifier, "TimedRobot");

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
    }

    @Override
    public void close() {
        NotifierJNI.stopNotifier(m_notifier);
        NotifierJNI.cleanNotifier(m_notifier);
    }

    /** Provide an alternate "main loop" via startCompetition(). */
    @Override
    public void startCompetition() {
        long initStart = 0;
        if(logWithAdvantageKit) initStart = Logger.getRealTimestamp();
        
        robotInit();

        if (isSimulation()) {
            simulationInit();
        }

        long initEnd = 0;
        if(logWithAdvantageKit) initEnd = Logger.getRealTimestamp();

        if (logWithAdvantageKit) {
            try {
                Method registerFields = AutoLogOutputManager.class.getDeclaredMethod("registerFields", Object.class);
                registerFields.setAccessible(true);
                registerFields.invoke(null, this);
                periodicAfterUser0.invoke(null, initEnd - initStart, 0);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        // Tell the DS that the robot is ready to be enabled
        System.out.println("********** Robot program startup complete **********");
        DriverStationJNI.observeUserProgramStarting();

        // Loop forever, calling the appropriate mode-dependent function
        while (true) {
            // We don't have to check there's an element in the queue first because
            // there's always at least one (the constructor adds one). It's reenqueued
            // at the end of the loop.
            var callback = m_callbacks.poll();

            NotifierJNI.updateNotifierAlarm(m_notifier, (long) (callback.expirationTime * 1e6));

            long curTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
            if (curTime == 0) {
                break;
            }

            callback.func.run();

            callback.expirationTime += callback.period;
            m_callbacks.add(callback);

            // Process all other callbacks that are ready to run
            while ((long) (m_callbacks.peek().expirationTime * 1e6) <= curTime) {
                callback = m_callbacks.poll();

                callback.func.run();

                callback.expirationTime += callback.period;
                m_callbacks.add(callback);
            }
        }
    }

    public static enum LoggingState {
        FULLMATCH,
        SESSION,
        NONE
    }

    /**
     * Add a data receiver for Adv Kit logging to a USB drive.
     * 
     * @param port  true if USB drive is plugged into the top port, false if it is
     *              plugged into the bottom port.
     * @param state session logging or full match logging.
     */
    public static void addReceiver(boolean port, LoggingState state) {
        if (state == LoggingState.NONE) {
            return;
        }

        String info = "";
        if (state == LoggingState.FULLMATCH) {
            info += DriverStation.getMatchNumber() + "_" +
                    DriverStation.getMatchType() + "_" +
                    DriverStation.getEventName() + "_";
        }
        info += LocalDateTime.now().getMonthValue() + "_" +
                LocalDateTime.now().getDayOfMonth() + "_" +
                LocalDateTime.now().getYear() + "_T_" +
                LocalDateTime.now().getHour() + "_" +
                LocalDateTime.now().getMinute();
        String folder = "";
        if (state == LoggingState.SESSION) {
            folder = "sessions";
        } else {
            folder = "matches";
        }

        if (port) {
            try {
                Logger.addDataReceiver(new WPILOGWriter("/media/sda1/" + folder + "/" + info + ".wpilog"));
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else {
            try {
                Logger.addDataReceiver(new WPILOGWriter("/media/sda2/" + folder + "/" + info + ".wpilog"));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /** Ends the main loop in startCompetition(). */
    @Override
    public void endCompetition() {
        NotifierJNI.stopNotifier(m_notifier);
    }

    /**
     * Add a callback to run at a specific period.
     *
     * <p>
     * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
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
     * Add a callback to run at a specific period with a starting time offset.
     *
     * <p>
     * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
     * run
     * synchronously. Interactions between them are thread-safe.
     *
     * @param callback      The callback to run.
     * @param periodSeconds The period at which to run the callback in seconds.
     * @param offsetSeconds The offset from the common starting time in seconds.
     *                      This is useful for
     *                      scheduling a callback in a different timeslot relative
     *                      to TimedRobot.
     */
    public static void addPeriodic(Runnable callback, double periodSeconds, double offsetSeconds) {
        m_callbacks.add(new Callback(callback, m_startTime, periodSeconds, offsetSeconds));
    }

    private static final class GcStatsCollector {
    private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final long[] lastTimes = new long[gcBeans.size()];
    private final long[] lastCounts = new long[gcBeans.size()];
  
    public void update() {
      long accumTime = 0;
      long accumCounts = 0;
      for (int i = 0; i < gcBeans.size(); i++) {
        long gcTime = gcBeans.get(i).getCollectionTime();
        long gcCount = gcBeans.get(i).getCollectionCount();
        accumTime += gcTime - lastTimes[i];
        accumCounts += gcCount - lastCounts[i];
  
        lastTimes[i] = gcTime;
        lastCounts[i] = gcCount;
      }
  
      Logger.recordOutput("LoggedRobot/GCTimeMS", (double) accumTime);
      Logger.recordOutput("LoggedRobot/GCCounts", (double) accumCounts);
    }
  }
}