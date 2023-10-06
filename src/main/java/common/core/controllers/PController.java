package common.core.controllers;
import edu.wpi.first.math.controller.PIDController;

/**
 * Team 3128's Position Controller class extending the {@link Controller} class.
 * 
 * @since 2023 CHARGED UP
 * @author Mason Lam, William Yuan
 */
public class PController extends Controller {
    
        /**
         * Create a new object to control position PID logic for a subsystem.
         * <p>Sets kP, kI, kD, period values.
         * <p>Sets kS, kG values.
         * <p>Defaults kG_Function to 1.
         * 
         * @param kS The constant power required to overcome static friction.
         * @param kG The constant power required to overcome gravity.
         * @param kP The proportional coefficient of the on board PIDController.
         * @param kI The integral coefficient of the on board PIDController.
         * @param kD The derivative coefficient of the on board PIDController.
         * @param period The PID periodic update rate in seconds.
         */
        public PController(double kS, double kG, double kP, double kI, double kD, double period) {
            super(kP, kI, kD, period);
            setkS(()-> kS);
            setkG(()-> kG);
            setkG_Function(() -> 1);
        }
        /**
         * Create a new object to control position PID logic for a subsystem.
         * <p>Sets kP, kI, kD values.
         * <p>Sets kS, kG values.
         * <p>Defaults kG_Function to 1.
         * <p>Defaults period to 0.02.
         * 
         * @param kS The constant power required to overcome static friction.
         * @param kG The constant power required to overcome gravity.
         * @param kP The proportional coefficient of the on board PIDController.
         * @param kI The integral coefficient of the on board PIDController.
         * @param kD The derivative coefficient of the on board PIDController.
         */
        public PController(double kS, double kG, double kP, double kI, double kD) {
            this(kS, kG, kP, kI, kD, 0.02);
        }
        /**
         * Overrides the calculate method in {@link PIDController} to use PID and FF logic.
         * <p>Uses kG instead of kV.
         * 
         * @param measurement The desired setpoint for the subsystem.
         * 
         * @return the output from PID and FF.
         */
        @Override
        public double calculate(double measurement) {
            final double pid = super.calculate(measurement);
            final double ff = !atSetpoint() ? Math.copySign(getkS(), pid) : 0 + getkG() * kG_Function.getAsDouble();
            return pid + ff;
        }
        
    }
