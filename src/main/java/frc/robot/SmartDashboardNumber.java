package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardNumber {
    private final String key;
    private double value;
    private double defaultValue;

    public SmartDashboardNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        SmartDashboard.setPersistent(key);
        SmartDashboard.setDefaultNumber(key, defaultValue);
        this.value = SmartDashboard.getNumber(key, defaultValue);
    }

    public double getNumber() {
        value = SmartDashboard.getNumber(key, defaultValue);
        return value;
    }

    public void setNumber(double v) {
        value = v;
        SmartDashboard.putNumber(key, v);
    }
}
