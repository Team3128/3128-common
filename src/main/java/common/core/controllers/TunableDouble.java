package common.core.controllers;

import java.util.function.DoubleSupplier;

import common.utility.shuffleboard.NAR_Shuffleboard;

public class TunableDouble implements DoubleSupplier {

    public double tunableDouble;
    public String tabName;
    public DoubleSupplier tunableDoubleSupplier;

    public TunableDouble(double tunableDouble, String tabName, String name, int x, int y){
        this.tunableDouble = tunableDouble;
        this.tabName = tabName;
        NAR_Shuffleboard.addData(tabName, name, tunableDouble, x,y);
        NAR_Shuffleboard.addData(tabName, "TOGGLE", false, 1, 0).withWidget("Toggle Button").getEntry();
        tunableDoubleSupplier = NAR_Shuffleboard.debug(tabName, "Debug " + name, 0, 1,2);
    }

    @Override
    public double getAsDouble() {
        return (NAR_Shuffleboard.getBoolean(tabName, "TOGGLE").getAsBoolean()) ? tunableDoubleSupplier.getAsDouble() : tunableDouble;
    }

    
}
