/*
 * ===============================================================================================
 * Red Storm Robotics 2022/3
 * ===============================================================================================
 * Drive Train Control Module (Velocity Control) / / This Drive Train module focuses on closed-loop
 * PID control / of the motor velocities for TANK drive. It is optimized to / use the Spark MAX and
 * Neo velocity controls.
 * ===============================================================================================
 * Hardware: / / 2 gearboxes (one for left, one for right) driven by multiple / SparkMax / Neo
 * motors, configured to drive in parallel. The / actual number of motors driving this gearbox can
 * be specified / when this object is constructed.
 * ===============================================================================================
 * 3rd Party Dependencies: / / https://software-metadata.revrobotics.com/REVLib-2023.json
 * ===============================================================================================
 * Permission is hereby granted, free of charge, to any / person obtaining a copy of this software
 * and associated / documentation files (the "Software"), to deal in the / Software without
 * restriction, including without limitation / the rights to use, copy, modify, merge, publish,
 * distribute, / sublicense, and/or sell copies of the Software, and to / permit persons to whom the
 * Software is furnished to do so, / subject to the following conditions: / / The above copyright
 * notice and this permission notice shall / be included in all copies or substantial portions of
 * the Software. / / THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY / KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE / WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR / PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS / OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR / OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR /
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE / SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 * ===============================================================================================
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class DriveTrainTankBasicController implements DriveTrainTank {

    // Drive Base
    private BaseMotorController rdrive;
    private BaseMotorController ldrive;
    private BaseMotorController rfollowers[];
    private BaseMotorController lfollowers[];
    private double SpeedLimit = 1.0;

    // DriveTrain Configuration
    protected final double SPEED_FAST = 1.0;
    protected final double SPEED_MED = 0.7;
    protected final double SPEED_SLOW = 0.4;

    public static final double PEAK_VELOCITY_INCHES_PER_SEC = 130;

    public DriveTrainTankBasicController(BaseMotorController left[], BaseMotorController right[]) {
        if ((left.length < 1) || (right.length < 1)) {
            ldrive = null;
            rdrive = null;
            lfollowers = new BaseMotorController[0];
            rfollowers = new BaseMotorController[0];
            return;
        } else {
            ldrive = left[0];
            if (left.length > 1) {
                lfollowers = new BaseMotorController[left.length - 1];
                for (int i = 0; i < left.length - 1; ++i) {
                    lfollowers[i] = left[i + 1];
                }
            } else {
                lfollowers = new BaseMotorController[0];
            }

            rdrive = right[0];
            if (right.length > 1) {
                rfollowers = new BaseMotorController[right.length - 1];
                for (int i = 0; i < right.length - 1; ++i) {
                    rfollowers[i] = right[i + 1];
                }
            } else {
                rfollowers = new BaseMotorController[0];
            }
        }
    }

    public void Init() {
        if (rdrive == null)
            return;

        // Swapping direction on the right drive motors because they face the other way
        rdrive.setInverted(true);
        for (BaseMotorController motor : rfollowers) {
            motor.setInverted(true);
        }
        // Tell some of the motors to follow the main motors so we can be lazy
        for (BaseMotorController motor : rfollowers) {
            motor.follow(rdrive);

        }
        ldrive.setInverted(false);
        for (BaseMotorController motor : lfollowers) {
            motor.setInverted(false);
            motor.follow(ldrive);
        }

        enableThis();
    }

    // Not supported by Neo's
    public void configBrakeOnStop() {}

    public void configCoastOnStop() {}

    // ===============================================================================================

    public void drive(double forward, double right) {
        drive(forward, right, SpeedLimit, false);
    }

    // limit should be between 0-1, 1 = 100%, 0.5 = 50% speed
    public void drive(double forward, double right, double limit) {
        drive(forward, right, limit, false);
    }

    // limit should be between 0-1, 1 = 100%, 0.5 = 50% speed
    public void drive(double forward, double right, double limit, boolean useBrake) {
        if (rdrive == null)
            return;

        // TODO brake code

        double leftDrive = adjustLeftDrive(forward, right, limit);
        double rightDrive = adjustRightDrive(forward, right, limit);
        if (Math.abs(leftDrive) < .1 && Math.abs(rightDrive) < .1) {
            stopAll();
            return;
        }
        Health.trace("leftDrive", leftDrive);
        Health.trace("rightDrive", rightDrive);

        ldrive.set(ControlMode.PercentOutput, leftDrive);
        rdrive.set(ControlMode.PercentOutput, rightDrive);
    }

    public void setSpeedBoost() {}

    public void setSpeedCreep() {}

    public void setNormalSpeed() {}

    public double adjustLeftDrive(double forward, double right, double limit) {
        double adjustedSpeed =
                (((forward + right) / Math.max(1.0, Math.abs(forward) + Math.abs(right)))) * limit;
        return adjustedSpeed;
    }

    public double adjustRightDrive(double forward, double right, double limit) {
        double adjustedSpeed =
                (((forward - right) / Math.max(1.0, Math.abs(forward) + Math.abs(right)))) * limit;
        return adjustedSpeed;
    }

    public void setSpeedLimit(SPEED speedLimit) {
        switch (speedLimit) {
            case SLOW:
                SpeedLimit = SPEED_SLOW;
                break;
            case MEDIUM:
                SpeedLimit = SPEED_MED;
                break;
            case FAST:
                SpeedLimit = SPEED_FAST;
                break;
            default:
                SpeedLimit = SPEED_MED;
                break;
        }
    }

    public void setSpeedLimit(double speedLimit) {
        SpeedLimit = speedLimit;
    }

    public void stopAll() {
        ldrive.set(ControlMode.PercentOutput, 0.0);
        rdrive.set(ControlMode.PercentOutput, 0.0);

    }

    public void brake() {
        stopAll();
    }

    // Configure Drive Wheels for Velocity Control (Acceleration-Controlled Tele-Op)
    private void enableThis() {
        if (rdrive == null)
            return;
        // TODO configure velocity control
    }

    public void setTargetPosition(double leftInches, double rightInches) {}

    public boolean areWeThereYet() {
        return true;
    }

    @Override
    public void regoToHoldingPosition() {
        // TODO Auto-generated method stub

    }

    @Override
    public void markEncoderLeftPosition() {
        // TODO Auto-generated method stub

    }

    @Override
    public void markEncoderRightPosition() {
        // TODO Auto-generated method stub

    }

    @Override
    public double getMarkedEncoderLeftPosition() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getMarkedEncoderRightPosition() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void updateBrakePosition(double leftInches, double rightInches) {
        // TODO Auto-generated method stub

    }
}
