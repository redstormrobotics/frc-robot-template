/*============================================================
/ Red Storm Robotics 2023
/=============================================================
/ DRIVE TRAIN for TeleOp
/ 
/ This Drive Train module provides a consistent interface for
/ TeleOp Modes while allowing the user to switch between Raw
/ and Velocity-Controlled modes.
/=============================================================
/ Hardware:
/
/ 2 gearboxes (one for left, one for right) driven by multiple 
/   TalonFX motors, configured to drive in parallel. The actual
/   number of motors driving this gearbox can be specified when
/   this object is constructed.
/=============================================================
/ 3rd Party Dependencies:
/
/ http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json
/=============================================================
/ Permission is hereby granted, free of charge, to any 
/ person obtaining a copy of this software and associated 
/ documentation files (the "Software"), to deal in the 
/ Software without restriction, including without limitation 
/ the rights to use, copy, modify, merge, publish, distribute, 
/ sublicense, and/or sell copies of the Software, and to 
/ permit persons to whom the Software is furnished to do so, 
/ subject to the following conditions:
/ 
/ The above copyright notice and this permission notice shall 
/ be included in all copies or substantial portions of the Software.
/
/ THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY 
/ KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE 
/ WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
/ PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS 
/ OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR 
/ OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
/ OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
/ SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
/=================================================================*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSwerveImpl implements DriveSwerve {

    private final WPI_TalonFX frDrive;
    private final WPI_TalonFX frTurn;
    private final WPI_CANCoder frCANCoder;
    private final WPI_TalonFX flDrive;
    private final WPI_TalonFX flTurn;
    private final WPI_CANCoder flCANCoder;
    private final WPI_TalonFX brDrive;
    private final WPI_TalonFX brTurn;
    private final WPI_CANCoder brCANCoder;
    private final WPI_TalonFX blDrive;
    private final WPI_TalonFX blTurn;
    private final WPI_CANCoder blCANCoder;
    private final ADXRS450_Gyro gyro; //id 2

    private double CC_BR;
    private double CC_BL;
    private double CC_FR;
    private double CC_FL;

    private final double FROffset = (320.9765);
    private final double FLOffset = (78.5742);
    private final double BROffset = (303.3984);
    private final double BLOffset = (31.640);


    protected TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
    protected TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    public final static Gains kGains_Turn = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
    public final static Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
    public final static Gains kGains_Drive = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
    public final static Gains kGains_Driving = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
    // change values if needed
    public final static double kNeutralDeadband = 0.001;
    private final double ABSOLUTE_MAX_CURRENT = 15.0;
    public final static int kTimeoutMs = 30;

    private final double CANCODER_CPR = 4096;

    public void init() {
        initCorner(blDrive, blTurn, blCANCoder);
        initCorner(brDrive, brTurn, brCANCoder);
        initCorner(flDrive, flTurn, flCANCoder);
        initCorner(frDrive, frTurn, frCANCoder);
    }

    public void initCorner(WPI_TalonFX drive, WPI_TalonFX turn, WPI_CANCoder canCoder) {

        /** Distance Configs */

        /* Configure the left Talon's selected sensor as integrated sensor */
        // TODO: link turnMotor to our own canCoder
        turnMotorConfig.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
        turnMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        driveMotorConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local
        // Feedback
        // Source
        /*
         * Configure the Remote (Left) Talon's selected sensor as a remote sensor for
         * the right Talon
         */
        turnMotorConfig.remoteFilter0.remoteSensorDeviceID = turn.getDeviceID(); // Device ID of Remote Source
        turnMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder; // Remote Source
                                                                                                      // Type

        driveMotorConfig.remoteFilter0.remoteSensorDeviceID = turn.getDeviceID(); // Device ID of Remote Source
        driveMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Source
                                                                                                       // Type

        /* FPID for Velocity */
        turnMotorConfig.slot0.kF = kGains_Turn.kF;
        turnMotorConfig.slot0.kP = kGains_Turn.kP;
        turnMotorConfig.slot0.kI = kGains_Turn.kI;
        turnMotorConfig.slot0.kD = kGains_Turn.kD;
        turnMotorConfig.slot0.integralZone = kGains_Turn.kIzone;
        turnMotorConfig.slot0.closedLoopPeakOutput = kGains_Turn.kPeakOutput;

        driveMotorConfig.slot0.kF = kGains_Drive.kF;
        driveMotorConfig.slot0.kP = kGains_Drive.kP;
        driveMotorConfig.slot0.kI = kGains_Drive.kI;
        driveMotorConfig.slot0.kD = kGains_Drive.kD;
        driveMotorConfig.slot0.integralZone = kGains_Drive.kIzone;
        driveMotorConfig.slot0.closedLoopPeakOutput = kGains_Drive.kPeakOutput;

        /*
         * false means talon's local output is PID0 + PID1, and other side Talon is PID0
         * - PID1
         * This is typical when the master is the right Talon FX and using Pigeon
         * 
         * true means talon's local output is PID0 - PID1, and other side Talon is PID0
         * + PID1
         * This is typical when the master is the left Talon FX and using Pigeon
         */

        /* FPID for Heading */
        turnMotorConfig.slot1.kF = kGains_Turning.kF;
        turnMotorConfig.slot1.kP = kGains_Turning.kP;
        turnMotorConfig.slot1.kI = kGains_Turning.kI;
        turnMotorConfig.slot1.kD = kGains_Turning.kD;
        turnMotorConfig.slot1.integralZone = kGains_Turning.kIzone;
        turnMotorConfig.slot1.closedLoopPeakOutput = kGains_Turning.kPeakOutput;

        driveMotorConfig.slot1.kF = kGains_Driving.kF;
        driveMotorConfig.slot1.kP = kGains_Driving.kP;
        driveMotorConfig.slot1.kI = kGains_Driving.kI;
        driveMotorConfig.slot1.kD = kGains_Driving.kD;
        driveMotorConfig.slot1.integralZone = kGains_Driving.kIzone;
        driveMotorConfig.slot1.closedLoopPeakOutput = kGains_Driving.kPeakOutput;

        /* Config the neutral deadband. */
        turnMotorConfig.neutralDeadband = kNeutralDeadband;
        driveMotorConfig.neutralDeadband = kNeutralDeadband;

        /**
         * 1ms per loop. PID loop can be slowed down if need be.
         * For example,
         * - if sensor updates are too slow
         * - sensor deltas are very small per update, so derivative error never gets
         * large enough to be useful.
         * - sensor movement is very slow causing the derivative error to be near zero.
         */
        int closedLoopTimeMs = 1;
        turnMotorConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
        turnMotorConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
        turnMotorConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
        turnMotorConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

        driveMotorConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
        driveMotorConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
        driveMotorConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
        driveMotorConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

        /* Motion Magic Configs */ // og value for all = 2000
        turnMotorConfig.motionAcceleration = 2000; // (distance units per 100 ms) per second
        turnMotorConfig.motionCruiseVelocity = 2000; // distance units per 100 ms

        driveMotorConfig.motionAcceleration = 2000; // (distance units per 100 ms) per second
        driveMotorConfig.motionCruiseVelocity = 2000; // distance units per 100 ms

        // TODO FIGURE OUT CURRENT LIMITS
        turnMotorConfig.statorCurrLimit.currentLimit = ABSOLUTE_MAX_CURRENT;
        turnMotorConfig.statorCurrLimit.enable = false;

        driveMotorConfig.statorCurrLimit.currentLimit = ABSOLUTE_MAX_CURRENT;
        driveMotorConfig.statorCurrLimit.enable = false;
        /* APPLY the config settings */
        turn.configAllSettings(turnMotorConfig);
        drive.configAllSettings(driveMotorConfig);

        /* Set status frame periods to ensure we don't have stale data */
        turn.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
        turn.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, kTimeoutMs);
        turn.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);

        turn.setNeutralMode(NeutralMode.Brake);

        drive.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
        drive.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, kTimeoutMs);
        drive.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);

        drive.setNeutralMode(NeutralMode.Brake);

    }

    public void enable() {
        resetGyro();
    }

    public void drive(MODE mode, Vector drive, Vector spin) {
        switch (mode) {
            case DRIVE:
            case POSITIONDRIVE:
                driveDrive(mode, drive.getY(), drive.getX(), spin.getZ());
                break;
            case BRAKE:
                brake();
                break;
        }
    }

    // Sum of all drive train motors (goofy ahh unit)
    public double getTotalDriveCurrent() {
        return frDrive.getStatorCurrent() +
                frTurn.getStatorCurrent() +
                flDrive.getStatorCurrent() +
                flTurn.getStatorCurrent() +
                brDrive.getStatorCurrent() +
                brTurn.getStatorCurrent() +
                blDrive.getStatorCurrent() +
                blTurn.getStatorCurrent();
    }

    public double getDistanceEncoderPosition(WHEEL_ID wheelID) {
        switch (wheelID) {
            case FL:
                return flDrive.getSelectedSensorPosition();
            case FR:
                return frDrive.getSelectedSensorPosition();
            case BL:
                return blDrive.getSelectedSensorPosition();
            case BR:
                return brDrive.getSelectedSensorPosition();
        }
        return 0.0;
    }

    public double getAngleEncoderCount(WHEEL_ID wheelID) {
        switch (wheelID) {
            case FL:
                return flTurn.getSelectedSensorPosition();
            case FR:
                return frTurn.getSelectedSensorPosition();
            case BL:
                return blTurn.getSelectedSensorPosition();
            case BR:
                return brTurn.getSelectedSensorPosition();
        }
        return 0.0;
    }

    public double getWheelRPM(WHEEL_ID wheelID) {
        double RPMCOEF = 0.29296875;
        switch (wheelID) {
            case FL:
                return flDrive.getSelectedSensorVelocity() * RPMCOEF;
            case FR:
                return frDrive.getSelectedSensorVelocity() * RPMCOEF;
            case BL:
                return blDrive.getSelectedSensorVelocity() * RPMCOEF;
            case BR:
                return brDrive.getSelectedSensorVelocity() * RPMCOEF;
        }
        return 0.0;
    }

    // In Radians
    public double getWheelAngle(WHEEL_ID wheelID) {
        switch (wheelID) {
            case FL:
                return flTurn.getSelectedSensorPosition();
            case FR:
                return frTurn.getSelectedSensorPosition();
            case BL:
                return blTurn.getSelectedSensorPosition();
            case BR:
                return brTurn.getSelectedSensorPosition();
        }
        return 0.0;
    }

    

    public DriveSwerveImpl(
            WPI_TalonFX frDrive, WPI_TalonFX frTurn, WPI_CANCoder frCANCoder,
            WPI_TalonFX flDrive, WPI_TalonFX flTurn, WPI_CANCoder flCANCoder,
            WPI_TalonFX brDrive, WPI_TalonFX brTurn, WPI_CANCoder brCANCoder,
            WPI_TalonFX blDrive, WPI_TalonFX blTurn, WPI_CANCoder blCANCoder,
            ADXRS450_Gyro gyro) {
        this.frDrive = frDrive;
        this.frTurn = frTurn;
        this.frCANCoder = frCANCoder;
        this.flDrive = flDrive;
        this.flTurn = flTurn;
        this.flCANCoder = flCANCoder;
        this.brDrive = brDrive;
        this.brTurn = brTurn;
        this.brCANCoder = brCANCoder;
        this.blDrive = blDrive;
        this.blTurn = blTurn;
        this.blCANCoder = blCANCoder;
        this.gyro = gyro;

        gyro.calibrate();
    }

    public void driveDrive(MODE mode, double forward, double strafe, double turn) {
        SmartDashboard.putNumber("drive forward", forward);
        SmartDashboard.putNumber("drive strafe", strafe);
        SmartDashboard.putNumber("drive turn", turn);
        double turningValue = gyro.getAngle();
        SmartDashboard.putNumber("Gyro", turningValue);


        Vector targetFR = Convert(forward, strafe, turn, gyro, FROffset, -225);
        Vector targetFL = Convert(forward, strafe, turn, gyro, FLOffset, -135);
        Vector targetBR = Convert(forward, strafe, turn, gyro, BROffset,45);
        Vector targetBL = Convert(forward, strafe, turn, gyro, BLOffset,-45);


        // SmartDashboard.putNumber("FR X", targetFR.getX()); //135
        // SmartDashboard.putNumber("FR Angle", targetFR.getAngleXY()); //307
        // SmartDashboard.putNumber("FR Y", targetFR.getY()); //-180
        SmartDashboard.putNumber("FR Length", targetFR.getLengthXY());
        SmartDashboard.putNumber("FL Length", targetFL.getLengthXY());
        SmartDashboard.putNumber("BL Length", targetBL.getLengthXY());
        SmartDashboard.putNumber("BR Length", targetBR.getLengthXY());

        SmartDashboard.putNumber("FR Wheel Angle", frCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("FL Wheel Angle", flCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("BL Wheel Angle", blCANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("BR Wheel Angle", brCANCoder.getAbsolutePosition());

        

        driveCorner(mode, WHEEL_ID.FR, targetFR, frDrive, frTurn, frCANCoder);
        driveCorner(mode, WHEEL_ID.FL, targetFL, flDrive, flTurn, flCANCoder);
        driveCorner(mode, WHEEL_ID.BR, targetBR, brDrive, brTurn, brCANCoder);
        driveCorner(mode, WHEEL_ID.BL, targetBL, blDrive, blTurn, blCANCoder);
    }

    public void test(boolean reset) {
        double turningValue = gyro.getAngle();
        // Invert the direction of the turn if we are going backwards
        // turningValue = Math.copySign(turningValue, m_joystick.getY());
        SmartDashboard.putNumber("Gyro", turningValue);

        boolean move = false;

        if (reset) {
            move = true;
            CC_FR = frCANCoder.getAbsolutePosition();
            CC_FL = flCANCoder.getAbsolutePosition();
            CC_BR = brCANCoder.getAbsolutePosition();
            CC_BL = blCANCoder.getAbsolutePosition();

        } else {
            SmartDashboard.putNumber("fr_Offset", frCANCoder.getAbsolutePosition() - CC_FR);
            SmartDashboard.putNumber("fl_Offset", flCANCoder.getAbsolutePosition() - CC_FL);
            SmartDashboard.putNumber("br_Offset", brCANCoder.getAbsolutePosition() - CC_BR);
            SmartDashboard.putNumber("bl_Offset", blCANCoder.getAbsolutePosition() - CC_BL);
        }

        Vector targetFR = ConvertNoGyro(move, -225);
        Vector targetFL = ConvertNoGyro(move, -135);
        Vector targetRR = ConvertNoGyro(move, 45);
        Vector targetRL = ConvertNoGyro(move, -45);

        driveCorner(MODE.DRIVE, WHEEL_ID.FR, targetFR, frDrive, frTurn, frCANCoder);
        driveCorner(MODE.DRIVE, WHEEL_ID.FL, targetFL, flDrive, flTurn, flCANCoder);
        driveCorner(MODE.DRIVE, WHEEL_ID.BR, targetRR, brDrive, brTurn, brCANCoder);
        driveCorner(MODE.DRIVE, WHEEL_ID.BR, targetRL, blDrive, blTurn, blCANCoder);
    }

    public Vector Convert(double forward, double strafe, double turn, ADXRS450_Gyro gyro, double offset,
            double turnOffset) {
        SmartDashboard.putNumber("Z", turn);
        // Reduce z (spin by 0.5)
        double turnReduce = 0.3;
        Vector zv = Vector.fromPolar(-turn * turnReduce, turnOffset, 0.0);

        double speedReduce = 0.5;
        // Reduce xy by 0.5
        Vector xyv = Vector.fromCart(-strafe * speedReduce, forward * speedReduce, 0.0);

        xyv = Vector.fromPolar(xyv.getLengthXY(), xyv.getAngleXY() + gyro.getAngle(), xyv.getZ());

        Vector finv = xyv.addVector(zv);
        double angle = finv.getAngleXY();
        angle = angle + offset;
        while (angle > 360) {
            angle = angle - 360;
        }
        while (angle < 0) {
            angle = angle + 360;
        }

        return Vector.fromPolar(finv.getLengthXY(), angle, finv.getZ());
    }

    public Vector ConvertNoGyro(boolean move, double turnOffset) {
        // SmartDashboard.putNumber("Z", joy.getRawAxis(4));
        // Reduce z (spin by 0.5)
        double turnReduce = 0.3;
        Vector zv = Vector.fromPolar(turnOffset, 0 * turnReduce, 0.0);

        double speedReduce = 0.5;
        // Reduce xy by 0.5
        Vector xyv = Vector.fromCart(0 * speedReduce, 0.0 * speedReduce, 0.0);
        if (move) {
            xyv = Vector.fromCart(0 * speedReduce, 0.2 * speedReduce, 0.0);
        }

        Vector finv = xyv.addVector(zv);
        double angle = finv.getAngleXY();
        
        while (angle > 360) {
            angle = angle - 360;
        }
        while (angle < 0) {
            angle = angle + 360;
        }

        return Vector.fromPolar(finv.getLengthXY(), angle, finv.getZ());
    }

    public void driveCorner(MODE mode, WHEEL_ID corner, Vector target, WPI_TalonFX speed, final WPI_TalonFX turn_angle,
            WPI_CANCoder _CANCoder) // Turns Wheel
    {
        ControlMode driveMode = mode == MODE.DRIVE ? ControlMode.PercentOutput : ControlMode.Position;
        double angle = target.getAngleXY();
        double length = target.getLengthXY();
        double canPositionDegree = _CANCoder.getAbsolutePosition() % 360.0;
        if (canPositionDegree < 0.0) {
            canPositionDegree += 360.0;
        }
        double canPositionRadians = Math.toRadians(canPositionDegree);

        // SmartDashboard.putNumber(corner + " RawA", canPosition);
        // SmartDashboard.putNumber(corner + " Target Angle", angle);
        // SmartDashboard.putNumber(corner + " CANCoder Angle", canPosition);
        SmartDashboard.putNumber(corner + " Length", length);

        if (length > 0.02) {

            while (angle - canPositionRadians < -Math.PI) {
                angle = angle + Math.PI * 2;
            }

            while (angle - canPositionRadians > Math.PI) {
                angle = angle - Math.PI * 2;
            }

            double diffAbs = Math.abs(angle - canPositionRadians);
            double diff = angle - canPositionRadians;

            if (diff > Math.PI) {
                length = 0.0 - length;
                angle = angle - 2.0 * Math.PI;
            } else if (diff < -Math.PI) {
                length = 0.0 - length;
                angle = angle + 2.0 * Math.PI;
            }

            diffAbs = Math.abs(angle - canPositionRadians);

            SmartDashboard.putNumber(corner + " Diff", diffAbs);
            diffAbs = diffAbs / 50.0;

            double turnSpeed = 0.5;
            if (diffAbs > turnSpeed) {
                diffAbs = turnSpeed;
            }
            if (diffAbs < -turnSpeed) {
                diffAbs = -turnSpeed;
            }
            // TODO: convert turn to set position/PID
            // #wouldBeMoreEfficient, like absEncoder??? :O

            if (angle < canPositionRadians) {
                //turn_angle.set(ControlMode.Position, Math.toDegrees(diffAbs));
            Health.getHealth().info(corner + " target angle", diffAbs);
            } else {
                //turn_angle.set(ControlMode.Position, Math.toDegrees(-diffAbs));
                Health.getHealth().info(corner + " target angle", -diffAbs);
            }

            speed.set(driveMode, length);
            Health.getHealth().info(corner + " speed", length);

        } else {
            //turn_angle.set(ControlMode.Position, 0);
            speed.set(driveMode, 0);
            Health.getHealth().info(corner + " speed", 0.0);
            Health.getHealth().info(corner + " target angle", 0.0);
        }
        turn_angle.set(ControlMode.PercentOutput, 0.2);

    }

    private void setWheelToAngle(WPI_TalonFX turn, WPI_CANCoder canCoder, double targetAngle) {
        double currentClick = canCoder.getAbsolutePosition();
        double relCurrentClick = currentClick % CANCODER_CPR;
        double targetClick = targetAngle * (CANCODER_CPR / (2.0 * Math.PI));
        double clickDist = targetClick - relCurrentClick;
        if (Math.abs(clickDist) > (CANCODER_CPR / 2.0)) {
            clickDist = CANCODER_CPR - clickDist;
        }
        targetClick = currentClick + clickDist;
        turn.set(ControlMode.Position, targetClick);
    }

    @Override
    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void setSpeedMode(SPEED speed) {

    }

    private void brake() {
        frDrive.set(ControlMode.PercentOutput, 0);
        frTurn.set(ControlMode.PercentOutput, 0);
        flDrive.set(ControlMode.PercentOutput, 0);
        flTurn.set(ControlMode.PercentOutput, 0);
        brDrive.set(ControlMode.PercentOutput, 0);
        brTurn.set(ControlMode.PercentOutput, 0);
        blDrive.set(ControlMode.PercentOutput, 0);
        blTurn.set(ControlMode.PercentOutput, 0);
    }
}