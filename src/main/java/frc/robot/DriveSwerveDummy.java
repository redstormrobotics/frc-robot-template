package frc.robot;

public class DriveSwerveDummy implements DriveSwerve {
    public void init() {
    }

    public void enable() {
    }

    public void drive(MODE mode, Vector drive, Vector spin) {
    }

    // Sum of all drive train motors (goofy ahh unit)
    public double getTotalDriveCurrent() {
        return 0.0;
    }

    public double getDistanceEncoderPosition(WHEEL_ID wheelID) {
        return 0.0;
    }

    public double getAngleEncoderCount(WHEEL_ID wheelID) {
        return 0.0;
    }

    public double getWheelRPM(WHEEL_ID wheelID) {
        return 0.0;
    }

    // In Radians
    public double getWheelAngle(WHEEL_ID wheelID) {
        return 0.0;
    }

    @Override
    public void resetGyro() {
        
    }

    @Override
    public void setSpeedMode(SPEED speed) {
        
    }
}
