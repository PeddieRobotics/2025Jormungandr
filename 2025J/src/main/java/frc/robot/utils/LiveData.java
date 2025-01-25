package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiveData {
    private double constant;
    private String name;

    public LiveData(double c, String n) {
        constant = c;
        name = n;
        SmartDashboard.putNumber(name, constant);
    }

    public double get() {
        return constant;
    }

    public void set(double c) {
        constant = c;
        SmartDashboard.putNumber(name, constant);
    }

}