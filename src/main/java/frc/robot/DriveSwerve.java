package frc.robot;

public interface DriveSwerve
{
    public enum MODE{
        DRIVE,
        BRAKE,
        DRIVEPERCENT,
        DRIVETOPOSITION,
        DRIVECONTINUE
    }

    public enum SPEED{
        SLOW,
        MED,
        FAST
    }

    public enum WHEEL_ID{
        FL, // Front Left
        FR, // Front Right
        BL, // Back Left
        BR // Back Right
    }

    public void init();

    public void initTestMode();

    public void enable();

    public void turnOff();

    public void drive( MODE mode, Vector drive, double spin);

    public void resetGyroHeading();

    // Sum of all drive train motors (goofy ahh unit)
    public double getTotalDriveCurrent();

    public double getDistanceEncoderPosition(WHEEL_ID wheelID);

    public double getAngleEncoderCount(WHEEL_ID wheelID);

    public double getWheelRPS(WHEEL_ID wheelID);

    // In Radians
    public double getWheelAngle(WHEEL_ID wheelID);

    public double getMaxWheelError();
    
    public boolean isAtPosition();

    public double gyroAngle();

    public double robotAngle();

    public void resetGyro();

    public void setSpeedMode(SPEED speed);

    public void updatePose();

    public void logData();

    public void logTestData();
}
