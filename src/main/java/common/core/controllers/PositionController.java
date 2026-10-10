// package common.core.controllers;

// import common.hardware.motorcontroller.NAR_Motor;
// import edu.wpi.first.math.controller.PIDController;

// public class PositionController extends ControllerBase implements AutoCloseable {

//     private PIDController controller;
//     public double[] inputRange;

//     public PositionController(PIDFFConfig config, double tolerance, double[] inputRange) {
//         super(config, tolerance);
//         this.inputRange = inputRange;

//         this.controller = new PIDController(config.getkP(), config.getkI(), config.getkD());
//     }

//     @Override
//     public void setMeasurementSource(NAR_Motor motor) {
//         measurement = () -> motor.getPosition();
//     }
    
//     public void setSetpoint(double setpoint) {
//         controller.setSetpoint(Math.max(inputRange[0], Math.min(setpoint, inputRange[1])));
//     }

//     public void enableContinuousInput() {
//         controller.enableContinuousInput(inputRange[0], inputRange[1]);
//     }

//     public double[] getInputRange() {
//         return inputRange;
//     }

// }
