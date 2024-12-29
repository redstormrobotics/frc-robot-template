/*
 * ============================================================ / Red Storm Robotics 2023
 * /============================================================= / DRIVE TRAIN for TeleOp / / This
 * Drive Train module provides a consistent interface for / TeleOp Modes while allowing the user to
 * switch between Raw / and Velocity-Controlled modes.
 * /============================================================= / Hardware: / / 2 gearboxes (one
 * for left, one for right) driven by multiple / TalonFX motors, configured to drive in parallel.
 * The actual / number of motors driving this gearbox can be specified when / this object is
 * constructed. /============================================================= / 3rd Party
 * Dependencies: / /
 * http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json
 * /============================================================= / Permission is hereby granted,
 * free of charge, to any / person obtaining a copy of this software and associated / documentation
 * files (the "Software"), to deal in the / Software without restriction, including without
 * limitation / the rights to use, copy, modify, merge, publish, distribute, / sublicense, and/or
 * sell copies of the Software, and to / permit persons to whom the Software is furnished to do so,
 * / subject to the following conditions: / / The above copyright notice and this permission notice
 * shall / be included in all copies or substantial portions of the Software. / / THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY / KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE / WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR / PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS / OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR / OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR / OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE / SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * /=================================================================
 */

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSwerveImpl implements DriveSwerve {

    private SwerveModuleFalcon frCorner;
    private SwerveModuleFalcon flCorner;
    private SwerveModuleFalcon brCorner;
    private SwerveModuleFalcon blCorner;
    private final Pigeon2 gyro; // id 2
    private double gyroOffset = 0.0;

    private double CC_BR;
    private double CC_BL;
    private double CC_FR;
    private double CC_FL;


    final private Vector frLocation = Vector.fromPolarDegrees(15.39, 90.0 + 39.52, 0.0);
    final private Vector brLocation = Vector.fromPolarDegrees(15.39, 180.0 + 42.37, 0.0);
    final private Vector blLocation = Vector.fromPolarDegrees(15.39, 270.0 + 42.37, 0.0);
    final private Vector flLocation = Vector.fromPolarDegrees(15.39, 90.0 - 39.52, 0.0);
    // get furthest distance
    final private double maxWheelDist =
            Math.max(Math.max(frLocation.getLengthXY(), flLocation.getLengthXY()),
                    Math.max(brLocation.getLengthXY(), blLocation.getLengthXY()));
    // calcuate corner rotation scaling factor
    final private double frRotationScaling = frLocation.getLengthXY() / maxWheelDist;
    final private double flRotationScaling = flLocation.getLengthXY() / maxWheelDist;
    final private double brRotationScaling = brLocation.getLengthXY() / maxWheelDist;
    final private double blRotationScaling = blLocation.getLengthXY() / maxWheelDist;

    public final double SPEED_FAST = 1.0;
    public final double SPEED_MED = 0.7;
    public final double SPEED_SLOW = 0.4;
    private double speed = SPEED_SLOW;

    private Vector targetPosition = null;

    private boolean atTargetPosition = false;


    private double turnReduce = 0.8;

    private double maxWheelError = 0.0;

    public DriveSwerveImpl(TalonFX frDrive, TalonFX frTurn, CANcoder frCANCoder, TalonFX flDrive,
            TalonFX flTurn, CANcoder flCANCoder, TalonFX brDrive, TalonFX brTurn,
            CANcoder brCANCoder, TalonFX blDrive, TalonFX blTurn, CANcoder blCANCoder,
            Pigeon2 gyro, Robot.ROBOTNAME robotName) {

        this.gyro = gyro;
        frCorner = new SwerveModuleFalcon("fr", frDrive, frTurn, frCANCoder,
                frLocation.getAngleXYDegree() / 360.0, robotName);
        brCorner = new SwerveModuleFalcon("br", brDrive, brTurn, brCANCoder,
                brLocation.getAngleXYDegree() / 360.0, robotName);
        blCorner = new SwerveModuleFalcon("bl", blDrive, blTurn, blCANCoder,
                blLocation.getAngleXYDegree() / 360.0, robotName);
        flCorner = new SwerveModuleFalcon("fl", flDrive, flTurn, flCANCoder,
                flLocation.getAngleXYDegree() / 360.0, robotName);
        gyro.reset();
    }

    public void init() {
        frCorner.init();
        flCorner.init();
        brCorner.init();
        blCorner.init();
        resetGyroHeading();
    }

    public void enable() {
        resetGyro();
    }

    public void turnOff(){
        frCorner.turnOff();
        flCorner.turnOff();
        brCorner.turnOff();
        blCorner.turnOff();
    }
    public void resetGyroHeading() {
        gyroOffset = +gyro.getYaw().getValueAsDouble() - 45.0;
    }

    public void drive(MODE mode, Vector drive, double spin) {
        switch (mode) {
            case DRIVE:
            case DRIVEPERCENT:
                driveDrive(mode, drive, spin);
                break;
            case BRAKE:
                brake();
                break;
            case DRIVETOPOSITION:
                targetPosition = drive;
                atTargetPosition = false;
            case DRIVECONTINUE:
                driveDrive(mode, drive, spin);
                
        }
    }

    // Sum of all drive train motors (goofy ahh unit)
    public double getTotalDriveCurrent() {
        return frCorner.getTotalCurrent() + flCorner.getTotalCurrent() + brCorner.getTotalCurrent()
                + blCorner.getTotalCurrent();
    }

    public double getDistanceEncoderPosition(WHEEL_ID wheelID) {
        switch (wheelID) {
            case FL:
                return flCorner.getDistanceEncoderPosition();
            case FR:
                return frCorner.getDistanceEncoderPosition();
            case BL:
                return blCorner.getDistanceEncoderPosition();
            case BR:
                return brCorner.getDistanceEncoderPosition();
        }
        return 0.0;
    }

    public double getAngleEncoderCount(WHEEL_ID wheelID) {
        switch (wheelID) {
            case FL:
                return flCorner.getAngleEncoderCount();
            case FR:
                return frCorner.getAngleEncoderCount();
            case BL:
                return blCorner.getAngleEncoderCount();
            case BR:
                return brCorner.getAngleEncoderCount();
        }
        return 0.0;
    }

    public double getWheelRPS(WHEEL_ID wheelID) {
        final double RPSCOEF = 0.29296875;
        switch (wheelID) {
            case FL:
                return flCorner.getWheelRPS() * RPSCOEF;
            case FR:
                return frCorner.getWheelRPS() * RPSCOEF;
            case BL:
                return blCorner.getWheelRPS() * RPSCOEF;
            case BR:
                return brCorner.getWheelRPS() * RPSCOEF;
        }
        return 0.0;
    }

    // In Radians
    public double getWheelAngle(WHEEL_ID wheelID) {
        switch (wheelID) {
            case FL:
                return flCorner.getWheelAngleRadians();
            case FR:
                return frCorner.getWheelAngleRadians();
            case BL:
                return blCorner.getWheelAngleRadians();
            case BR:
                return brCorner.getWheelAngleRadians();
        }
        return 0.0;
    }

    public double getMaxWheelError() {
        return maxWheelError;
    }
    public double gyroAngle(){
        return gyro.getYaw().getValueAsDouble();
    }
    public double robotAngle(){                      // zero is aiming at front-right module?
        return -gyro.getYaw().getValueAsDouble() + gyroOffset + 45.0; // gyro.getAngle returns degrees
    }
    public void driveDrive(MODE mode, Vector drive, double spin) {
        SmartDashboard.putNumber("DriveSwerve/forward", drive.getY()); // -1 to 1
        SmartDashboard.putNumber("DriveSwerve/strafe", drive.getX()); // -1 to 1
        SmartDashboard.putNumber("DriveSwerve/turn", spin); // -1 to 1
        SmartDashboard.putNumber("DriveSwerve/Total Drive CurdriveDriverent: ", getTotalDriveCurrent());


        double robotAngle = gyroOffset - gyro.getYaw().getValueAsDouble(); // gyro.getAngle returns degrees
        SmartDashboard.putNumber("DriveSwerve/gyro", robotAngle);

        double adjustedSpin = spin * turnReduce;
        Vector focDrive = Vector.fromPolarDegrees(drive.getLengthXY(),
                drive.getAngleXYDegree() + robotAngle, drive.getZ());

        Vector targetFR =
                translateToWheel("fr", focDrive, adjustedSpin, frLocation, frRotationScaling);
        Vector targetFL =
                translateToWheel("fl", focDrive, adjustedSpin, flLocation, flRotationScaling);
        Vector targetBR =
                translateToWheel("br", focDrive, adjustedSpin, brLocation, brRotationScaling);
        Vector targetBL =
                translateToWheel("bl", focDrive, adjustedSpin, blLocation, blRotationScaling);

        boolean optimize = mode != MODE.DRIVECONTINUE && mode != MODE.DRIVETOPOSITION;
        // Drive actual swerve motor modules based on calculations
        double frError = frCorner.driveAngle(targetFR, optimize);
        double flError = flCorner.driveAngle(targetFL, optimize);
        double brError = brCorner.driveAngle(targetBR, optimize);
        double blError = blCorner.driveAngle(targetBL, optimize);
        // find the largest error
        maxWheelError = Math.max(Math.max(Math.abs(frError), Math.abs(flError)),
                Math.max(Math.abs(brError), Math.abs(blError)));
        SmartDashboard.putNumber("DriveSwerve/Max wheel error", maxWheelError);
        // scale the drive speed for the mode
        double scaleFactor;
        switch (mode) {
            case BRAKE:
                scaleFactor = 0.0;
                break;
            case DRIVETOPOSITION:
            case DRIVECONTINUE:
                scaleFactor = (maxWheelError < 0.1) ? Math.cos(maxWheelError) : 0.0;
            default:
                scaleFactor = Math.cos(maxWheelError);
        }
        // set the driving scale
        frCorner.setDriveErrorScale(scaleFactor);
        flCorner.setDriveErrorScale(scaleFactor);
        brCorner.setDriveErrorScale(scaleFactor);
        blCorner.setDriveErrorScale(scaleFactor);
        // now that all corners are set, apply speed
        if (mode == MODE.DRIVE){
            frCorner.applyVelocity();
            flCorner.applyVelocity();
            brCorner.applyVelocity();
            blCorner.applyVelocity();
        } else if(mode == MODE.DRIVETOPOSITION){
            frCorner.setTargetPosition(targetFR);
            flCorner.setTargetPosition(targetFL);
            brCorner.setTargetPosition(targetBR);
            blCorner.setTargetPosition(targetBL);
        } else if(mode == MODE.DRIVECONTINUE){
                    boolean fr = frCorner.applyPosition();
                    boolean fl = flCorner.applyPosition();
                    boolean br = brCorner.applyPosition();
                    boolean bl = blCorner.applyPosition();
            atTargetPosition = 
                    fr && fl && br && bl;
        } else {
            frCorner.applySpeed();
            flCorner.applySpeed();
            brCorner.applySpeed();
            blCorner.applySpeed();
        }
        
    }
    
    public boolean isAtPosition(){
        return atTargetPosition;
    }

    public Vector translateToWheel(String name, Vector drive, double spin, Vector wheelLocation,
            double wheelRotationScale) {
        // calcuate spin vector, translated to wheel
        Vector zv = Vector.fromPolarDegrees(-spin * wheelRotationScale,
                wheelLocation.getAngleXYDegree() + 45.0, spin);
        // combine drive and spin vectors
        Vector finv = drive.addVector(zv);
        // Determine the heading angle in degrees for FOC
        double angle = finv.getAngleXYDegree();
        // Update combined vector, rotated around field angle
        SmartDashboard.putNumber("DriveSwerve/angle" + name, angle);
        SmartDashboard.putNumber("DriveSwerve/wa" + name, wheelLocation.getAngleXYDegree());
        return Vector.fromPolarDegrees(finv.getLengthXY(), angle, finv.getZ());
    }

    @Override
    public void resetGyro() {
        gyro.reset();
    }

    public void zeroPosition() {
        frCorner.zeroPosition();
        flCorner.zeroPosition();
        brCorner.zeroPosition();
        blCorner.zeroPosition();
    }

    @Override
    public void setSpeedMode(SPEED speed) {
        switch (speed) {
            //drive speeds
            case SLOW:
                this.speed = SPEED_SLOW;
                frCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_SLOW_RPS);
                flCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_SLOW_RPS);
                brCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_SLOW_RPS);
                blCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_SLOW_RPS);

                frCorner.setMaxVoltage (SwerveModuleFalcon.DRIVE_SLOW_VOLTAGE);
                flCorner.setMaxVoltage(SwerveModuleFalcon.DRIVE_SLOW_VOLTAGE);
                brCorner.setMaxVoltage(SwerveModuleFalcon.DRIVE_SLOW_VOLTAGE);
                blCorner.setMaxVoltage(SwerveModuleFalcon.DRIVE_SLOW_VOLTAGE);
                break;
            case FAST:
                this.speed = SPEED_FAST;
                frCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_FAST_RPS);
                flCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_FAST_RPS);
                brCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_FAST_RPS);
                blCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_FAST_RPS);

                frCorner.setMaxVoltage (SwerveModuleFalcon.DRIVE_FAST_VOLTAGE);
                flCorner.setMaxVoltage(SwerveModuleFalcon.DRIVE_FAST_VOLTAGE);
                brCorner.setMaxVoltage(SwerveModuleFalcon.DRIVE_FAST_VOLTAGE);
                blCorner.setMaxVoltage(SwerveModuleFalcon.DRIVE_FAST_VOLTAGE);
                break;
            case MED:
            default:
                this.speed = SPEED_MED;
                frCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_MED_RPS);
                flCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_MED_RPS);
                brCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_MED_RPS);
                blCorner.setMaxRPS(SwerveModuleFalcon.DRIVE_MED_RPS);

                frCorner.setMaxVoltage (SwerveModuleFalcon.DRIVE_MED_VOLTAGE);
                flCorner.setMaxVoltage(SwerveModuleFalcon.DRIVE_MED_VOLTAGE);
                brCorner.setMaxVoltage(SwerveModuleFalcon.DRIVE_MED_VOLTAGE);
                blCorner.setMaxVoltage(SwerveModuleFalcon.DRIVE_MED_VOLTAGE);
                break;
        }
    }

    private void brake() {
        frCorner.brake();
        flCorner.brake();
        brCorner.brake();
        blCorner.brake();
    }

    public void logData() {
        frCorner.logData();
        flCorner.logData();
        brCorner.logData();
        blCorner.logData();
    }

    public void logTestData() {
        SmartDashboard.putNumber("DriveSwerveTest/fr offset", frCorner.calcCanCoderAbsOffset());
        SmartDashboard.putNumber("DriveSwerveTest/fl offset", flCorner.calcCanCoderAbsOffset());
        SmartDashboard.putNumber("DriveSwerveTest/br offset", brCorner.calcCanCoderAbsOffset());
        SmartDashboard.putNumber("DriveSwerveTest/bl offset", blCorner.calcCanCoderAbsOffset());
    }
}
