package common.utility.sysid;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.Matrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

/**
 * Team 3128's wrapper class to store feedforward constants data.
 * @since 2024 Crescendo
 * @author Teja Yaramada, Audrey Zheng, William Yuan
 */
public class FFCharacterization {

    public enum Type {
        REGRESSION,
        LEAST_SQUARES;
    }

    private final String name;
    private final LinkedList<Double> velocityData = new LinkedList<>();
    private final LinkedList<Double> voltageData = new LinkedList<>();
    private final LinkedList<Double> timeData = new LinkedList<>();
    private final LinkedList<Double> accelerationData = new LinkedList<>();
    private final LinkedList<Double> adjustedVoltageData = new LinkedList<>();
    private PolynomialRegression voltageVelocityRegression;
    private PolynomialRegression velocityTimeRegression;
    private PolynomialRegression accelerationTimeRegression;
    private PolynomialRegression voltageAccelerationRegression;

    private Type type;
    private double rampRate;


    private double kS;
    private double kV;
    private double kA;


    /**
     * Creates a new FFCharacterization object to store data.
     * @param name Name of the Subsystem.
     */
    public FFCharacterization(String name, Type type, double rampRate) {
      this.name = name;
      this.type = type;
    }

    /**
     * Adds the subsystem's state at a specific time.
     * @param time Current time.
     * @param velocity Velocity of the subsystem.
     * @param voltage Voltage applied to the subsystem.
     */
    public void add(double time, double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
        timeData.add(time);
      }
    }

    /**
     * Display the feedforward constants calculated from the data.
     */
    public void print() {
        if (velocityData.size() == 0 || voltageData.size() == 0) {
            System.out.println("Test failed");
            return;
        }

        if(type.name().equals(Type.REGRESSION.name())){
            calculateRegression();
        } else {
            calculateLeastSquares();
        }
    }

    private void calculateRegression() {
        //calculates regression function for voltage v velocity
        voltageVelocityRegression = new PolynomialRegression(
            velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
            voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
            1
        );


        kS = voltageVelocityRegression.beta(0);
        kV = voltageVelocityRegression.beta(1);

        //calculates regression function for velocity v time
        velocityTimeRegression = new PolynomialRegression(
            timeData.stream().mapToDouble(Double::doubleValue).toArray(),
            velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
            4
        );

        //calculates acceleration data from derivative of velocity v time regression function
        accelerationData.addAll(
            new PolynomialDerivative(velocityTimeRegression).evaluate(timeData)
        );

        for(int i = 0; i < voltageData.size(); i++){
            adjustedVoltageData.add(i, (voltageData.get(i) - kS) - (kV * velocityData.get(i)));
        }

        //calculates regression function for voltage v acceleration
        voltageAccelerationRegression = new PolynomialRegression(
            accelerationData.stream().mapToDouble(Double::doubleValue).toArray(),
            adjustedVoltageData.stream().mapToDouble(Double::doubleValue).toArray(),
            1
        );

        kA = voltageAccelerationRegression.beta(1);

        System.out.println("FF Characterization Results (" + name + "):");
        System.out.println(
            "\tCount=" + Integer.toString(velocityData.size()) + ""
        );
        System.out.println(String.format("\tR2=%.5f", voltageVelocityRegression.R2()));            //R2
        System.out.println(String.format("\tkS=%.5f", kS));                                        //ks
        System.out.println(String.format("\tkV=%.5f", kV));                                        //kv
        System.out.println(String.format("\tkA=%.5f", kA));                                        //ka
    }

    private void calculateLeastSquares() {
        velocityTimeRegression = new PolynomialRegression(
            timeData.stream().mapToDouble(Double::doubleValue).toArray(),
            velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
            4
        );

        accelerationData.addAll(
            new PolynomialDerivative(velocityTimeRegression).evaluate(timeData)
        );

        accelerationTimeRegression = new PolynomialRegression(
            timeData.stream().mapToDouble(Double::doubleValue).toArray(),
            accelerationData.stream().mapToDouble(Double::doubleValue).toArray(),
            4
        );

        List<Double> velocityApproximates = velocityTimeRegression.predict(timeData);
        List<Double> accelerationApproximates = accelerationTimeRegression.predict(timeData);
        List<Double> ones = new ArrayList<>();
        for(int i = 0; i < timeData.size(); i++){
            ones.add(1.0);
        }
        List<Double> voltageData = timeData.stream().map(t -> rampRate*t).collect(Collectors.toList());
        double[] voltageApproximates = new double[timeData.size()];

        double[][] a = new double[timeData.size()][3];

        for(int i = 0; i < timeData.size(); i++){
            a[i][0] = ones.get(i);
            a[i][1] = velocityApproximates.get(i);
            a[i][2] = accelerationApproximates.get(i);
            voltageApproximates[i] = voltageData.get(i);
        }

        DMatrixRMaj A = new DMatrixRMaj(a);
        DMatrixRMaj b = new DMatrixRMaj(voltageApproximates);
        DMatrixRMaj x = new DMatrixRMaj(3, 1);

        boolean systemIsReal = CommonOps_DDRM.solve(A, b, x);

        System.out.println("FF Characterization Results (" + name + "):");
        System.out.println(
            "\tCount=" + Integer.toString(velocityData.size()) + ""
        );
        System.out.println("Sysem is " + (systemIsReal ? "real" : "unreal"));
        System.out.println("Solution: " + x);
    }

    /**
     * Returns the kS value calculated by the data.
     * @return Returns the kS value.
     */
    public double getkS() {
        return kS;
    }

    /**
     * Returns the kV value calculated by the data.
     * @return Returns the kV value.
     */
    public double getkV() {
        return kV;
    }

    /**
     * Returns the kA value calculated by the data.
     * @return Returns the kA value.
     */
    public double getkA() {
        return kA;
    }

    /**
     * Clears the data stored in the FFCharacterization object.
     */
    public void clear() {
        velocityData.clear();
        voltageData.clear();
        timeData.clear();
        accelerationData.clear();
        adjustedVoltageData.clear();
    }
}