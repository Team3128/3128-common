package common.core.controllers;

import edu.wpi.first.math.controller.PIDController;

/**
 * Team 3128's Velocity Controller class extending the {@link Controller} class.
 * 
 * @since 2023 CHARGED UP
 * @author Mason Lam, William Yuan
 */
public class VController extends Controller {
         /**
         * Create a new object to control velocity PID logic for a subsystem.
         * <p>Sets kP, kI, kD, period values.
         * <p>Sets kS, kV values.
         * 
         * @param kS The constant power required to overcome static friction.
         * @param kV The constant power required to maintain a set velocity.
         * @param kP The proportional coefficient of the on board PIDController.
         * @param kI The integral coefficient of the on board PIDController.
         * @param kD The derivative coefficient of the on board PIDController.
         * @param period The PID periodic update rate in seconds.
         */
        public VController(double kS, double kV, double kP, double kI, double kD, double period) {
            super(kP, kI, kD, period);
            setkS(()-> kS);
            setkV(()-> kV);
        }
        /**
         * Create a new object to control velocity PID logic for a subsystem.
         * <p>Sets kP, kI, kD values.
         * <p>Sets kS, kV values.
         * <p>Defaults period to 0.02.
         * 
         * @param kS The constant power required to overcome static friction.
         * @param kV The constant power required to maintain a set velocity.
         * @param kP The proportional coefficient of the on board PIDController.
         * @param kI The integral coefficient of the on board PIDController.
         * @param kD The derivative coefficient of the on board PIDController.
         */
        public VController(double kS, double kV, double kP, double kI, double kD) {
            this(kS, kV, kP, kI, kD, 0.02);
        }
        /**
         * Overrides the calculate method in {@link PIDController} to use PID and FF logic.
         * <p>Uses kV instead of kG.
         * 
         * @param measurement The desired setpoint for the subsystem.
         * 
         * @return the output from PID and FF.
         */
        @Override
        public double calculate(double measurement) {
            final double pid = super.calculate(measurement);
            final double ff = !atSetpoint() ? Math.copySign(getkS(), pid) : 0 + getkV() * getSetpoint();
            return pid + ff;
        }
        
    }
