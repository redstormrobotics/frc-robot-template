package frc.robot;

public interface DriveSwerve
{
    public enum MODE{
        DRIVE,
        POSITIONDRIVE,
        BRAKE

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

    public void enable();

    public void drive( MODE mode, Vector drive, Vector spin);

    // Sum of all drive train motors (goofy ahh unit)
    public double getTotalDriveCurrent();

    public double getDistanceEncoderPosition(WHEEL_ID wheelID);

    public double getAngleEncoderCount(WHEEL_ID wheelID);

    public double getWheelRPM(WHEEL_ID wheelID);

    // In Radians
    public double getWheelAngle(WHEEL_ID wheelID);

    public void resetGyro();

    public void setSpeedMode(SPEED speed);


}