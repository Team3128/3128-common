package common.hardware.motorcontroller;

import java.util.LinkedList;

import common.core.NAR_Robot;

public abstract class NAR_Motor {

    private static LinkedList<NAR_Motor> motors = new LinkedList<NAR_Motor>();
    static {
        NAR_Robot.addPeriodic(()-> {
            for (final NAR_Motor motor : motors) {
                motor.updateFollowers();
            }
        }, 0.02);
    }

    private LinkedList<NAR_Motor> followers;
    private double conversionFactor;

    public enum ControlMode {
        kDutyCycle,
        kVoltage,
        kVelocity,
        kPosition;
    }

    public NAR_Motor() {
        conversionFactor = 0;
        followers = new LinkedList<NAR_Motor>();
    }

    public void set(ControlMode mode, double value) {
        switch(mode) {
            case kDutyCycle:
                set(value);
                break;
            case kVoltage:
                setVoltage(value);
                break;
            case kVelocity:
                setVelocity(value);
                break;
            case kPosition:
                setPosition(value);
                break;
        }
    }

    public abstract void set(double speed);

    public abstract void setVoltage(double volts);

    public abstract void setVelocity(double velocity);

    public abstract void setPosition(double position);

    public double getPosition() {
        return getRawPosition() * conversionFactor;
    }

    public double getVelocity() {
        return getRawVelocity() * conversionFactor;
    }

    public abstract double getRawPosition();

    public abstract double getRawVelocity();

    public abstract double getOutput();

    private void updateFollowers() {
        final double output = getOutput();
        for (final NAR_Motor motor : followers) {
            motor.set(output);
        }
    }
}
