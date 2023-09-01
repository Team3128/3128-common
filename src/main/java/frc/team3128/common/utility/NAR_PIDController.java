package frc.team3128.common.utility;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;

public class NAR_PIDController extends PIDController {

    public final double kS;
    public final double kV;
    public final double kG;
    public final DoubleSupplier kG_Function;

    public NAR_PIDController(double kP, double kI, double kD, double kS, double kV, double kG) {
        super(kP, kI, kD);
        this.kS = kS;
        this.kV = kV;
        this.kG = kG;
        this.kG_Function = () -> 1;
    }

    public NAR_PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0);
    }
    
    @Override
    public double calculate(double measurement) {
        double output = super.calculate(measurement);
        output += kS * Math.signum(output);
        output += kV * getSetpoint();
        output += kG * kG_Function.getAsDouble();
        return output;
    }
}
