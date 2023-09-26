package common.core.controllers;

public class PController extends Controller {
    
        public PController(double kS, double kG, double kP, double kI, double kD, double period) {
            super(kP, kI, kD, period);
            this.kS = ()-> kS;
            this.kG = ()-> kG;
            kG_Function = () -> 1;
        }
    
        public PController(double kS, double kG, double kP, double kI, double kD) {
            this(kS, kG, kP, kI, kD, 0.02);
        }
        
        @Override
        public double calculate(double measurement) {
            final double pid = super.calculate(measurement);
            final double ff = !atSetpoint() ? Math.copySign(getkS(), pid) : 0 + getkG() * kG_Function.getAsDouble();
            return pid + ff;
        }
        
    }
