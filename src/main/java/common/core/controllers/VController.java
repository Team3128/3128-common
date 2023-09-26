package common.core.controllers;

public class VController extends Controller {
    
        public VController(double kS, double kV, double kP, double kI, double kD, double period) {
            super(kP, kI, kD, period);
            this.kS = ()-> kS;
            this.kV = ()-> kV;
        }
    
        public VController(double kS, double kV, double kP, double kI, double kD) {
            this(kS, kV, kP, kI, kD, 0.02);
        }
    
        @Override
        public double calculate(double measurement) {
            final double pid = super.calculate(measurement);
            final double ff = !atSetpoint() ? Math.copySign(getkS(), pid) : 0 + getkV() * getSetpoint();
            return pid + ff;
        }
        
    }
