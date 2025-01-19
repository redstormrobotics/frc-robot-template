package frc.robot;

public class DriveSwerveDummy implements DriveSwerve {
    public void init() {
        System.out.println("WARNING: Using DriveSwerveDummy");
    }

    public void initTestMode() {}

    public void enable() {}

    public void turnOff() {}

    public void drive(MODE mode, Vector drive, double spin) {}

    public void resetGyroHeading(){}
    
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

    public double getWheelRPS(WHEEL_ID wheelID) {
        return 0.0;
    }

    // In Radians
    public double getWheelAngle(WHEEL_ID wheelID) {
        return 0.0;
    }

    public double getMaxWheelError() {
        return 0.0;
    }

    public boolean isAtPosition(){
        return true;
    }

    public double robotAngle(){
        return 0.0;
    }
    
    public double gyroAngle(){
        return 0.0;
    }
    
    @Override
    public void resetGyro() {}

    @Override
    public void setSpeedMode(SPEED speed) {}

    public void updatePose(){}

    public void logData(){}

    public void logTestData(){}
}
