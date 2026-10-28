package common.utility;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Prints messages to the console in one format, {@code [Severity] [Category] message}, so they're
 * easy to scan and search. The console is also saved in the robot's log file.
 *
 * <p>Pick the method by how serious it is:
 * <ul>
 * <li>{@link #fatal}: the robot can't work properly. Also shows on the Driver Station.</li>
 * <li>{@link #recoverable}: something failed, but the robot keeps going. Also shows on the
 *     Driver Station.</li>
 * <li>{@link #unusual}: shouldn't happen, and is probably a bug, but nothing broke.</li>
 * <li>{@link #info}: worth knowing, e.g. "Intake ready".</li>
 * <li>{@link #debug}: only useful while fixing something; hidden unless turned on.</li>
 * </ul>
 *
 * <p>Debug messages only print when {@link #logDebug} is true and their {@link Type} is enabled,
 * e.g. {@code Log.logDebug = true; Log.Type.enable(Log.Type.STATE_MACHINE_PRIMARY);}. That lets you
 * see one area's details without flooding the console with every other's.
 *
 * @since Rapid React 2022
 * @author Jude Lifset, Teja Yaramada
 */
public class Log {

	/** Whether {@link #debug} messages print at all. */
	public static boolean logDebug = false;

	/**
	 * Areas of the code whose debug messages can be turned on separately. Each has an indent
	 * level, so more detailed messages are indented further in the console.
	 */
	public enum Type {
		PRIMARY(0),
		SECONDARY(1),
		TERTIARY(2),

		/** State changes. */
		STATE_MACHINE_PRIMARY(1),
		/** The steps of a state change, like cancelling a running transition. */
		STATE_MACHINE_SECONDARY(2),
		MECHANISM(2),
		/** Control loops starting and stopping. */
		CONTROLLER(2),
		/** Motor settings as they're applied. */
		MOTOR(3),
		VISION(1);

		private boolean enabled;
		private int level;

		private Type(int level) {
			this.enabled = false;
			this.level = level;
		}

		/** How many tabs its messages are indented by. */
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

		/** Turns on debug messages for several types at once. */
		public static void enable(Type... type) {
			List.of(type).forEach(logType -> logType.enable());
		}

		/** Turns off debug messages for several types at once. */
		public static void disable(Type... type) {
			List.of(type).forEach(logType -> logType.disable());
		}
	}

	/**
	 * Logs an error the robot can't work properly after. It also shows on the Driver Station.
	 *
	 * @param category What it's about, e.g. "Intake"
	 * @param message What went wrong
	 */
	public static void fatal(String category, String message) {
		log("Fatal", category, message);

		// make it show up on the DS as well
		DriverStation.reportError("Fatal Error: " + message, true);
	}

	/**
	 * Logs an exception the robot can't work properly after, with your message and the
	 * exception's name, message and stack trace. It also shows on the Driver Station.
	 */
	public static void fatalException(String category, String userMessage, Exception exception) {
		String exceptionMessage = String.format("%s -- %s: %s", userMessage, exception.getClass().getSimpleName(),
				exception.getMessage());
		log("Fatal", category, exceptionMessage);

		exception.printStackTrace();

		// make it show up on the DS as well
		DriverStation.reportError("Fatal Error: " + exceptionMessage, true);
	}

	/**
	 * Logs a failure that might stop one feature, though the robot as a whole keeps working. It
	 * also shows on the Driver Station.
	 *
	 * @param category What it's about, e.g. "Intake"
	 * @param message What went wrong
	 */
	public static void recoverable(String category, String message) {
		log("Recoverable", category, message);

		DriverStation.reportError("Error: " + (message == null ? "null" : message), true);

	}

	/**
	 * Logs something that shouldn't happen normally and is probably a bug, but didn't break
	 * anything, e.g. asking a state machine for a move it doesn't allow.
	 *
	 * @param category What it's about, e.g. "Intake"
	 * @param message What happened
	 */
	public static void unusual(String category, String message) {
		log("Unusual", category, message);
	}

	/**
	 * Logs something worth knowing that doesn't mean anything is wrong.
	 */
	public static void info(String category, String message) {
		log("Info", category, message);
	}

	/** Logs a number worth knowing, e.g. a measurement. */
	public static void info(String category, double data) {
		log("Info", category, Double.toString(data));
	}

	/** Runs {@code toRun} and logs how many seconds it took, e.g. to find what's slowing startup. */
	public static void profile(String category, Runnable toRun) {
		Timer timer = new Timer();
		timer.restart();
		toRun.run();
		timer.stop();
		info(category, timer.get());
	}

	/**
	 * Logs a detail that's only useful while fixing something. Prints only when
	 * {@link #logDebug} and {@link Type#PRIMARY} are both on.
	 *
	 * @param category What it's about, e.g. "Intake"
	 * @param message The detail
	 */
	public static void debug(String category, String message) {
		debug(Type.PRIMARY, category, message);
	}

	/**
	 * Logs a detail that's only useful while fixing something. Prints only when
	 * {@link #logDebug} and {@code type} are both on.
	 *
	 * @param type Which area it's about, so it can be turned on separately
	 * @param category What it's about, e.g. "Intake"
	 * @param message The detail
	 */
	public static void debug(Type type, String category, String message) {
		int level = type.getLevel();
		// A subsystem's state machine messages are indented under the robot-wide ones
		if(type.name().contains("STATE_MACHINE") && !category.contains("Robot")) level++;
		if(logDebug && type.getEnabled()) log(level, "Debug", category, message);
	}

	/** Prints a line of dashes, to separate sections of the console. */
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
