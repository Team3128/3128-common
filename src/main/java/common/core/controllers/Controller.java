package common.core.controllers;

import java.util.LinkedList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.math.controller.PIDController;

public class Controller extends PIDController {

    private final LinkedList<DoubleConsumer> consumers;
    private DoubleSupplier measurement;

    protected DoubleSupplier kS, kV, kG;
    protected DoubleSupplier kG_Function;

    public Controller(double kP, double kI, double kD, double period) {
        super(kP, kI, kD, period);
        kS = ()-> 0;
        kV = ()-> 0;
        kG = ()-> 0;
        kG_Function = () -> 1;
        consumers = new LinkedList<DoubleConsumer>();
    }

    public Controller(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.02);
    }

    public void setMeasurementSource(DoubleSupplier measurement) {
        this.measurement = measurement;
    }

    public void addMotor(NAR_Motor motor) {
        consumers.add(motor::set);
    }

    public void addOutput(DoubleConsumer output) {
        consumers.add(output);
    }

    public double useOutput() {
        if (measurement == null) return 0;
        final double output = calculate(measurement.getAsDouble());
        for (final DoubleConsumer consumer : consumers) {
            consumer.accept(output);
        }
        return output;
    }

    public void setkG_Function(DoubleSupplier kG_Function) {
        this.kG_Function = kG_Function;
    }

    public void setkS(double kS) {
        setkS(()-> kS);
    }

    public void setkS(DoubleSupplier kS) {
        this.kS = kS;
    }

    public void setkV(double kV) {
        setkV(()-> kV);
    }

    public void setkV(DoubleSupplier kV) {
        this.kV = kV;
    }

    public void setkG(double kG) {
        this.kG = ()-> kG;
    }

    public void setkG(DoubleSupplier kG) {
        this.kG = kG;
    }

    public double getkS() {
        return kS.getAsDouble();
    }

    public double getkV() {
        return kV.getAsDouble();
    }
    
    public double getkG() {
        return kG.getAsDouble();
    }
}
