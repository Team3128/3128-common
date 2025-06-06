package common.utility.sysid;

import java.util.LinkedList;

/**
 * Team 3128's wrapper class to store feedforward constants data.
 * @since 2024 Crescendo
 * @author Teja Yaramada, Audrey Zheng, William Yuan
 */
public class FFCharacterization {

    private final String name;
    private final LinkedList<Double> velocityData = new LinkedList<>();
    private final LinkedList<Double> angularVelocityData = new LinkedList<>();
    private final LinkedList<Double> voltageData = new LinkedList<>();
    private final LinkedList<Double> timeData = new LinkedList<>();
    private final LinkedList<Double> accelerationData = new LinkedList<>();
    private final LinkedList<Double> adjustedVoltageData = new LinkedList<>();
    private PolynomialRegression voltageVelocityRegression;
    private PolynomialRegression velocityTimeRegression;
    private PolynomialRegression voltageAccelerationRegression;

    private double kS;
    private double kV;
    private double kA;


    /**
     * Creates a new FFCharacterization object to store data.
     * @param name Name of the Subsystem.
     */
    public FFCharacterization(String name) {
      this.name = name;
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
     * Adds the subsystem's state at a specific time.
     * @param time Current time.
     * @param velocity Velocity of the subsystem.
     * @param voltage Voltage applied to the subsystem.
     */
    public void add(double time, double velocity, double voltage, double angularVelocity) {
        if (Math.abs(velocity) > 1E-4) {
          velocityData.add(Math.abs(velocity));
          angularVelocityData.add(Math.abs(angularVelocity));
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
        System.out.println(String.format("\tM=%.5f", 0));        //R2
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