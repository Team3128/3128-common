package common.utility;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * @since Rapid React 2022
 * @author Jude Lifset, Teja Yaramada
 */
public class Log {

	public static boolean logDebug = false;

	public enum Type {
		PRIMARY(0),
		SECONDARY(1),
		TERTIARY(2),

		STATE_MACHINE_PRIMARY(1),
		STATE_MACHINE_SECONDARY(2),
		MECHANISM(2),
		CONTROLLER(2),
		MOTOR(3),
		VISION(1);

		private boolean enabled;
		private int level;
		
		private Type(int level) {
			this.enabled = false;
			this.level = level;
		}

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

		public static void enable(Type... type) {
			List.of(type).forEach(logType -> logType.enable());
		}

		public static void disable(Type... type) {
			List.of(type).forEach(logType -> logType.disable());
		}
	}

	/**
	 * Log a FATAL error, after which the robot cannot (properly) function. <br>
	 * 
	 * @param category
	 * @param message
	 */
	public static void fatal(String category, String message) {
		log("Fatal", category, message);

		// make it show up on the DS as well
		DriverStation.reportError("Fatal Error: " + message, true);
	}

	/**
	 * Log a FATAL error due to an exception, after which the robot cannot
	 * (properly) function. <br>
	 * Prints your message, and the exception's name, message, and stacktrace.
	 * 
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
	 * Log a failure which may kill one function or one thread, however the robot as
	 * a whole can keep functioning.
	 * 
	 * @param category
	 * @param message
	 */
	public static void recoverable(String category, String message) {
		log("Recoverable", category, message);

		DriverStation.reportError("Error: " + (message == null ? "null" : message), true);

	}

	/**
	 * Log something which should not happen under normal circumstances and probably
	 * is a bug, but does not cause anything to crash.
	 * 
	 * @param category
	 * @param message
	 */
	public static void unusual(String category, String message) {
		log("Unusual", category, message);
	}

	/**
	 * Log a semi-important message which the user should probably see, but does not
	 * indicate anything is broken.
	 */
	public static void info(String category, String message) {
		log("Info", category, message);
	}

	/**
	 * Log a message which is not important during normal operation, but is useful
	 * if you're trying to debug the robot.
	 * 
	 * @param category
	 * @param message
	 */
	public static void debug(String category, String message) {
		debug(Type.PRIMARY, category, message);
	}

	/**
	 * Log a message which is not important during normal operation, but is useful
	 * if you're trying to debug the robot.
	 * 
	 * @param type
	 * @param category
	 * @param message
	 */
	public static void debug(Type type, String category, String message) {
		int level = type.getLevel();
		if(type.name().contains("STATE_MACHINE") && !category.contains("Robot")) level++;
		if(logDebug && type.getEnabled()) log(level, "Debug", category, message);
	}

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
