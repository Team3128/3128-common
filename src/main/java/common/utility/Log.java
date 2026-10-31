package common.utility;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Prints messages to the console in a consistent format, {@code [Severity] [Category] message}, so
 * that they are easy to scan and search. The console is also saved in the robot's log file.
 *
 * <p>Choose the method by severity:
 * <ul>
 * <li>{@link #fatal}: the robot cannot operate correctly. Also shown on the Driver Station.</li>
 * <li>{@link #recoverable}: something failed, but the robot continues. Also shown on the
 *     Driver Station.</li>
 * <li>{@link #unusual}: an unexpected condition, probably a bug, that caused no failure.</li>
 * <li>{@link #info}: useful information, for example, "Intake ready".</li>
 * <li>{@link #debug}: detail useful only for troubleshooting; hidden unless enabled.</li>
 * </ul>
 *
 * <p>Debug messages print only when {@link #logDebug} is true and their {@link Type} is enabled,
 * for example, {@code Log.logDebug = true; Log.Type.enable(Log.Type.STATE_MACHINE_PRIMARY);}. This shows
 * the detail for one area without flooding the console with others.
 *
 * @since Rapid React 2022
 * @author Jude Lifset, Teja Yaramada
 */
public class Log {

	/** Whether {@link #debug} messages print. */
	public static boolean logDebug = false;

	/**
	 * Areas of the code whose debug messages can be enabled separately. Each has an indent
	 * level, so more detailed messages are indented further in the console.
	 */
	public enum Type {
		PRIMARY(0),
		SECONDARY(1),
		TERTIARY(2),

		/** State changes. */
		STATE_MACHINE_PRIMARY(1),
		/** The steps of a state change, such as cancelling a running transition. */
		STATE_MACHINE_SECONDARY(2),
		MECHANISM(2),
		/** Control loops starting and stopping. */
		CONTROLLER(2),
		/** Motor settings as they are applied. */
		MOTOR(3),
		VISION(1);

		private boolean enabled;
		private int level;

		private Type(int level) {
			this.enabled = false;
			this.level = level;
		}

		/** The number of tabs by which messages of this type are indented. */
		public int getLevel() {
			return this.level;
		}

		public void enable() {
			this.enabled = true;
		}

		public void disable() {
			this.enabled = false;
		}

		public boolean getEnabled() {
			return this.enabled;
		}

		/** Enables debug messages for several types at once. */
		public static void enable(Type... type) {
			List.of(type).forEach(logType -> logType.enable());
		}

		/** Disables debug messages for several types at once. */
		public static void disable(Type... type) {
			List.of(type).forEach(logType -> logType.disable());
		}
	}

	/**
	 * Logs an error after which the robot cannot operate correctly. Also shown on the Driver Station.
	 *
	 * @param category The subject of the message, for example, "Intake"
	 * @param message A description of the failure
	 */
	public static void fatal(String category, String message) {
		log("Fatal", category, message);

		DriverStation.reportError("Fatal Error: " + message, true);
	}

	/**
	 * Logs an exception after which the robot cannot operate correctly, with the given message and the
	 * exception's name, message and stack trace. Also shown on the Driver Station.
	 */
	public static void fatalException(String category, String userMessage, Exception exception) {
		String exceptionMessage = String.format("%s -- %s: %s", userMessage, exception.getClass().getSimpleName(),
				exception.getMessage());
		log("Fatal", category, exceptionMessage);

		exception.printStackTrace();

		DriverStation.reportError("Fatal Error: " + exceptionMessage, true);
	}

	/**
	 * Logs a failure that may stop one feature while the robot as a whole continues working. Also
	 * shown on the Driver Station.
	 *
	 * @param category The subject of the message, for example, "Intake"
	 * @param message A description of the failure
	 */
	public static void recoverable(String category, String message) {
		log("Recoverable", category, message);

		DriverStation.reportError("Error: " + (message == null ? "null" : message), true);

	}

	/**
	 * Logs an unexpected condition that is probably a bug but caused no failure, for example, a
	 * request for a state transition that is not allowed.
	 *
	 * @param category The subject of the message, for example, "Intake"
	 * @param message A description of what happened
	 */
	public static void unusual(String category, String message) {
		log("Unusual", category, message);
	}

	/**
	 * Logs useful information that does not indicate a problem.
	 */
	public static void info(String category, String message) {
		log("Info", category, message);
	}

	/** Logs a numeric value, for example, a measurement. */
	public static void info(String category, double data) {
		log("Info", category, Double.toString(data));
	}

	/** Runs {@code toRun} and logs its duration in seconds, for example, to diagnose slow startup. */
	public static void profile(String category, Runnable toRun) {
		Timer timer = new Timer();
		timer.restart();
		toRun.run();
		timer.stop();
		info(category, timer.get());
	}

	/**
	 * Logs detail useful only for troubleshooting. Prints only when
	 * {@link #logDebug} and {@link Type#PRIMARY} are both enabled.
	 *
	 * @param category The subject of the message, for example, "Intake"
	 * @param message The detail
	 */
	public static void debug(String category, String message) {
		debug(Type.PRIMARY, category, message);
	}

	/**
	 * Logs detail useful only for troubleshooting. Prints only when
	 * {@link #logDebug} and {@code type} are both enabled.
	 *
	 * @param type The area of the code, which can be enabled separately
	 * @param category The subject of the message, for example, "Intake"
	 * @param message The detail
	 */
	public static void debug(Type type, String category, String message) {
		int level = type.getLevel();
		// A subsystem's state machine messages are indented under the robot-wide ones
		if(type.name().contains("STATE_MACHINE") && !category.contains("Robot")) level++;
		if(logDebug && type.getEnabled()) log(level, "Debug", category, message);
	}

	/** Prints a line of dashes to separate sections of the console. */
	public static void divider(int length) {
		System.out.println("-".repeat(length));
	}

	private static void log(String severity, String category, String message) {
		log(Type.PRIMARY.getLevel(), severity, category, message);
	}

	private static void log(int level, String severity, String category, String message) {
		System.out.println("\t\t".repeat(level) + String.format("[%s] [%s] %s", severity, category, message));
	}
}
