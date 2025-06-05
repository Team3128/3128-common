package common.utility.sysid;

import java.util.ArrayList;
import java.util.List;

/**
 * Team 3128's class to take the derivative of a Polynomial Regression function.
 * @since 2024 Crescendo
 * @author Teja Yaramada, Audrey Zheng, William Yuan
 */
public class PolynomialDerivative {

    private final double[] derivativeCoefficients;

    /**
     * Creates a new PolynomialDerivative function object.
     * @param function The polynomial regression function to be differentiated.
     */
    public PolynomialDerivative(PolynomialRegression function){

        this.derivativeCoefficients = new double[function.degree()];

        //calculate derivative coefficients
        for (int i = 0; i < derivativeCoefficients.length; i++) {
            derivativeCoefficients[i] = function.beta(i + 1) * (i + 1);
        }
    }

    /**
     * Returns a y-value by plugging in the x-value into the derivative function.
     * @param x - value from the x-axis.
     * @return Derivative value at the specified x-value.
     */
    public double evaluate(double x){
        double y = 0;
        for(int i = 0; i < derivativeCoefficients.length; i++){
            y += derivativeCoefficients[i] * Math.pow(x, i);
        }
        return y;
    }

    /**
     * Retursn y-values by pluggin in the x-values into the derivative function.
     * @param inputs List of x-values.
     * @return Derivative values from the x-values.
     */
    public List<Double> evaluate(List<Double> inputs){
        final ArrayList<Double> output = new ArrayList<Double>(inputs.size());
        for(final Double xVal : inputs){
            output.add(evaluate(xVal));
        }
        return output;
    }
  }