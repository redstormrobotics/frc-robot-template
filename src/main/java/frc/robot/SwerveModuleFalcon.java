package frc.robot;

import java.util.HashMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.DriveSwerve.MODE;
import frc.robot.DriveSwerve.WHEEL_ID;


public class SwerveModuleFalcon {

    static public final double DRIVE_SLOW_RPS = 0.5;
    static public final double DRIVE_MED_RPS = 1.0;
    static public final double DRIVE_FAST_RPS = 5.0;

    static public final double DRIVE_SLOW_VOLTAGE = 2.0;
    static public final double DRIVE_MED_VOLTAGE = 7.0;
    static public final double DRIVE_FAST_VOLTAGE = 12.0;

    private static final double TURN_MOTOR_GEAR_RATIO = 7.0;
    private static final double DRIVE_MOTOR_GEAR_RATIO_L1 = 8.14;
    private static final double DRIVE_MOTOR_GEAR_RATIO_L2 = 6.75;
    private static final double DRIVE_MOTOR_GEAR_RATIO_L3 = 6.12;

    static private final double WHEEL_DIAM_INCH = 4.0 * Math.PI;
    static private final double WHEEL_DIAM_METER = Units.inchesToMeters(WHEEL_DIAM_INCH);

    private final boolean useCanCoder = true;

    private final String name;
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;
    private final TalonFXConfiguration driveConfig;
    private final TalonFXConfiguration turnConfig;
    private double canCoderRelOffset;
    private final double canCoderAbsOffset;
    private final double cornerOffsetRotations;
    private final CANcoderConfiguration canCoderConfig;
    private final NeutralOut stopRequest = new NeutralOut().withUpdateFreqHz(50.0);
    private final VelocityVoltage driveRequest = new VelocityVoltage(0.0).withSlot(0).withUpdateFreqHz(50.0);
    private final PositionVoltage turnRequest = new PositionVoltage(0.0).withSlot(0).withUpdateFreqHz(50.0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0.0).withUpdateFreqHz(50.0);
    private final MotionMagicVoltage drivePosRequest = new MotionMagicVoltage(0.0).withSlot(1).withUpdateFreqHz(50.0);

    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<Angle> turnPosition;

    private final String sdDriveTargetRPSKey;
    private final String sdDriveMaxRPSKey;
    private final String sdDriveRPSKey;
    private final String sdDriveMaxKey;
    private final String sdDriveErrorScale;
    private final String sdDriveTempKey;
    private final String sdDriveStatorCurKey;
    private final String sdDriveSupplyCurKey;
    private final String sdDriveCLErrorKey;
    private final String sdDriveSensorPosKey;
    private final String sdDriveSensorMotorVoltageKey;
    private final String sdDriveVoltageKey;
    private final String sdDriveSensorVelKey;
    private final String sdDriveIntAccumKey;
    private final String sdDriveErrorDerivKey;
    private final String sdDriveFaultsKey;
    private final String sdDriveTargetRPS;

    private final String sdTurnTargetPosKey;
    private final String sdTurnTempKey;
    private final String sdTurnStatorCurKey;
    private final String sdTurnSupplyCurKey;
    private final String sdTurnCLErrorKey;
    private final String sdTurnSensorPosKey;
    private final String sdTurnAdjustSensorPosKey;
    private final String sdTurnSensorVelKey;
    private final String sdTurnIntAccumKey;
    private final String sdTurnErrorDerivKey;
    private final String sTurnFaultsKey;
    private final String sdTargetAngleRadians;
    private final String sdTargetAngleError;

    private final String sdCanCoderAbsPosKey;
    private final String sdcanCoderPosKey;
    private final String sdCanCoderVelKey;
    private final String sdCanCoderMagneticKey;

    private double maxVoltage = 6.0;
    private double maxRPS = 10.0;
    private double targetSpeed = 0.0;
    private double driveRPS = 0.0;
    private double driveVoltage = 0.0;
    private double driveErrorScale = 1.0;
    private double targetAngleRadians = 0.0;
    private double targetAngleError = 0.0;
    private double targetDrivePosition = 0.0;
    private double startingDrivePosition = 0.0;
    private boolean recordedStart = false;
    private boolean running = false;
    private boolean driveReversed = false;

    public SwerveModuleFalcon(String name, TalonFX driveMotor, TalonFX turnMotor, CANcoder canCoder,
            double cornerOffsetRotations, Robot.ROBOTNAME robotName) {
        this.name = name;
        if (driveMotor == null) {
            throw new RuntimeException("Unable to setup SwerveModuleFalcon" + name + " driveMotor is null");
        }
        if (turnMotor == null) {
            throw new RuntimeException("Unable to setup SwerveModuleFalcon" + name + " turnMotor is null");
        }
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.canCoder = canCoder;
        this.sdDriveTargetRPSKey = name + "/Drive/RPS Target";
        this.sdDriveMaxRPSKey = name + "/Drive/RPS Max";
        this.sdDriveRPSKey = name + "/Drive/Driven RPS";
        this.sdDriveMaxKey = name + "/Drive/Drive RPS";
        this.sdDriveErrorScale = name + "/Drive/Drive Error Scale";
        this.sdDriveTempKey = name + "/Drive/Temp";
        this.sdDriveSensorMotorVoltageKey = name + "/Drive/MotorVoltage";
        this.sdDriveVoltageKey = name + "/Drive/Driven Volts";
        this.sdDriveStatorCurKey = name + "/Drive/Current stator";
        this.sdDriveSupplyCurKey = name + "/Drive/Current supply";
        this.sdDriveCLErrorKey = name + "/Drive/Error closed loop";
        this.sdDriveSensorPosKey = name + "/Drive/Position sensor";
        this.sdDriveSensorVelKey = name + "/Drive/Velocity sensor";
        this.sdDriveIntAccumKey = name + "/Drive/Integral accumulator sum";
        this.sdDriveErrorDerivKey = name + "/Drive/Error derivative";
        this.sdDriveFaultsKey = name + "/Drive/Fault";
        this.sdDriveTargetRPS = name + "/Drive/targetRPS";

        this.sdTurnTempKey = name + "/Turn/Temp";
        this.sdTurnTargetPosKey = name + "/Turn/Position target";
        this.sdTurnStatorCurKey = name + "/Turn/Current statort";
        this.sdTurnSupplyCurKey = name + "/Turn/Current supply";
        this.sdTurnCLErrorKey = name + "/Turn/Loop error closed";
        this.sdTurnSensorPosKey = name + "/Turn/Position sensor";
        this.sdTurnAdjustSensorPosKey = name + "/Turn/Position Sensor Adjusted";
        this.sdTurnSensorVelKey = name + "/Turn/Velocity sensor";
        this.sdTurnIntAccumKey = name + "/Turn/Integral accumulator sum";
        this.sdTurnErrorDerivKey = name + "/Turn/Error derivative";
        this.sTurnFaultsKey = name + "/Turn/Fault";
        this.sdTargetAngleRadians = name + "/targetAngleRadians";
        this.sdTargetAngleError = name + "/sdTargetAngleError";

        this.sdCanCoderAbsPosKey = name + "/CANCoder/Position absolute";
        this.sdcanCoderPosKey = name + "/CANCoder/Position ticks";
        this.sdCanCoderVelKey = name + "/CANCoder/Velocity";
        this.sdCanCoderMagneticKey = name + "/CANCoder/Magnetic strength";

        driveConfig = new TalonFXConfiguration();
        
        driveConfig.FutureProofConfigs = true;

        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5; // the time at the peak supply current
        driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        driveConfig.CurrentLimits.StatorCurrentLimit = 80.0;

        driveConfig.Voltage.PeakForwardVoltage = 15.99;
        driveConfig.Voltage.PeakReverseVoltage = -15.99;
        driveConfig.Voltage.SupplyVoltageTimeConstant = 1.0;
        driveConfig.MotorOutput.PeakForwardDutyCycle = 1.00;
        driveConfig.MotorOutput.PeakReverseDutyCycle = -1.00;

        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.RotorToSensorRatio = 1.0;
        driveConfig.Feedback.SensorToMechanismRatio = DRIVE_MOTOR_GEAR_RATIO_L1;

        driveConfig.Slot0.kP = 2.0;
        driveConfig.Slot0.kI = 0.0;
        driveConfig.Slot0.kD = 0.0;
        driveConfig.Slot0.kV = 0.0;
        driveConfig.Slot0.kA = 0.0;
        driveConfig.Slot0.kS = 0.0;
        driveConfig.Slot0.kG = 0.0;

        driveConfig.Slot1.kP = 2.5;
        driveConfig.Slot1.kI = 0.0;
        driveConfig.Slot1.kD = 0.0;
        driveConfig.Slot1.kV = 0.0;
        driveConfig.Slot1.kA = 0.0;
        driveConfig.Slot1.kS = 0.0;
        driveConfig.Slot1.kG = 0.0;
        // Peak output of 40 amps
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveConfig.MotionMagic.MotionMagicCruiseVelocity = 10;
        driveConfig.MotionMagic.MotionMagicAcceleration = 20;
        driveConfig.MotionMagic.MotionMagicJerk = 50;

        turnConfig = new TalonFXConfiguration();
        turnConfig.FutureProofConfigs = true;

        turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        turnConfig.CurrentLimits.SupplyCurrentLimit = 0.5;

        if (useCanCoder) {
            // setup the turn motor to use the cancoder as its encoder for PID control
            turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            turnConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
            // set the turn ratios for the motor and sensor so the sensor returns the correct
            // rotations
            turnConfig.Feedback.RotorToSensorRatio = TURN_MOTOR_GEAR_RATIO;
            turnConfig.Feedback.SensorToMechanismRatio = 1.0;
        } else {
            // setup the turn motor to use its internal encoder
            turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            // set the turn ratios for the motor and sensor so the sensor returns the correct
            // rotations
            turnConfig.Feedback.RotorToSensorRatio = 1.0;
            turnConfig.Feedback.SensorToMechanismRatio = TURN_MOTOR_GEAR_RATIO;
        }

        turnConfig.Slot0.kP = 186.368; // An error of 1 rotation per second results in 2V output
        turnConfig.Slot0.kI = 0.0;
        turnConfig.Slot0.kD = 0.3;
        turnConfig.Slot0.kV = 0.0;
        turnConfig.Slot0.kA = 0.0;
        turnConfig.Slot0.kS = 0.0;
        turnConfig.Slot0.kG = 0.0;

        turnConfig.Voltage.PeakForwardVoltage = 20;
        turnConfig.Voltage.PeakReverseVoltage = -20;

        /*
         * Torque-based velocity does not require a feed forward, as torque will accelerate the
         * rotor up to the desired velocity by itself
         */
        turnConfig.Slot1.kP = 186.368;
        turnConfig.Slot1.kI = 0.0;
        turnConfig.Slot1.kD = 0.3;
        turnConfig.Slot1.kV = 0.0;
        turnConfig.Slot1.kA = 0.0;
        turnConfig.Slot1.kS = 0.0;
        turnConfig.Slot1.kG = 0.0;
        // Peak output of 40 amps
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        HashMap<String, Double> canCoderOffsets = new HashMap<String, Double>();
        switch (robotName) {
            case DEFAULT:
                canCoderOffsets.put("bl", 0.4111 + 0.75 / 2.0);
                canCoderOffsets.put("br", -4.6483 + 0.75 / 2.0);
                canCoderOffsets.put("fl", -2.1448 + 0.75 / 2.0);
                canCoderOffsets.put("fr", -4.8636+ 0.75 / 2.0);
                break;
            default:
                break;
        }

        Double absOffset = canCoderOffsets.get(name);
        if (absOffset == null) {
            Health.error(name + "/canCoderError",
                    "unable to look up cancoder offset for " + canCoder.getDeviceHash());
            // this.canCoderAbsOffset = 0.0 + cornerOffsetRotations;
            this.canCoderAbsOffset = 0.0;
        } else {
            this.canCoderAbsOffset = absOffset;
            Health.info(name + "/canCoderOffset", -this.canCoderAbsOffset);
            System.out.println("FalconSwerve " + name + " found canCoder offset " + absOffset
                    + " for " + canCoder.getDeviceHash());
        }

        this.cornerOffsetRotations = cornerOffsetRotations;
        canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.FutureProofConfigs = true;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = 0.0;

        drivePosition = driveMotor.getPosition();
        turnPosition = turnMotor.getPosition();

        System.out.println("FalconSwerve " + name + " set canCoder offset "
                + canCoderConfig.MagnetSensor.MagnetOffset + " from magent offset "
                + this.canCoderAbsOffset + " and corner offset " + this.cornerOffsetRotations);
    }

    public void init() {
        canCoderConfig.MagnetSensor.MagnetOffset = 0.0 - this.canCoderAbsOffset - this.cornerOffsetRotations;
        applySettings();
    }

    public void initTestMode() {
        canCoderConfig.MagnetSensor.MagnetOffset = 0.0;
        applySettings();
    }

    // Return the magentic offset to use to point straight (corner agnostic)
    public double getCancoderAngle() {
        return turnPosition.waitForUpdate(2.0).getValue().baseUnitMagnitude();
    }

    public void setMaxRPS(double RPS) {
        this.maxRPS = RPS;
        this.driveRPS = maxRPS * targetSpeed * driveErrorScale;
    }

    public void setMaxVoltage(double voltage) {
        this.maxVoltage = voltage;
        this.driveVoltage = maxVoltage * targetSpeed * driveErrorScale;
    }

    public void setDriveErrorScale(double error) {
        this.driveErrorScale = error;
        // System.out.println("FalconSwerveModule "+name + " setting driveErrorScale
        // "+driveErrorScale);
    }

    public void setTargetSpeed(double speed) {
        this.targetSpeed = speed;
        // System.out.println("FalconSwerveModule "+name + " setting targetRPS "+targetRPS);
    }

    public void turnOff() {
        running = false;
        setTargetSpeed(0.0);
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    public void turnOn() {
        running = true;
        driveMotor.setControl(driveRequest.withVelocity(driveRPS));
    }

    public void setTargetPosition(Vector target) {
        driveMotor.setControl(drivePosRequest.withPosition(driveMotor.getPosition().getValue()));
        targetDrivePosition = target.getLengthXY() / WHEEL_DIAM_INCH;
        targetAngleRadians = target.getAngleXYRadian();
        System.out.println(name + " target pos:" + targetDrivePosition + " at "
                + driveMotor.getPosition().getValue() + " from target " + target.getLengthXY() + " "
                + target.getAngleXYRadian());
        recordedStart = false;
    }

    public boolean applyPosition() {
        boolean areWeThereYet = false;
        double targetPosition = startingDrivePosition + (driveReversed ? -targetDrivePosition : targetDrivePosition);
        if (Math.abs(driveErrorScale) > 0.995) {
            if (!recordedStart) {
                startingDrivePosition = driveMotor.getPosition().waitForUpdate(2.0).getValueAsDouble();
                recordedStart = true;
            }
            if (Math.abs(driveMotor.getPosition().getValueAsDouble() - targetPosition) < 0.25) {
                System.out.println(name + " at target pos " );
                areWeThereYet = true;
                driveMotor.setControl(
                        drivePosRequest.withPosition(driveMotor.getPosition().getValue()));
            } else {
                driveMotor.setControl(drivePosRequest.withPosition(targetPosition));
                System.out.println(name + " going to  " + targetPosition);

            }
        } else { 
            System.out.println(name + " angle err too large " + driveErrorScale);
        }
        SmartDashboard.putBoolean("DriveSwerve/Are we there yet? (Drive)", areWeThereYet);
        return areWeThereYet;
    }

    public boolean applyRelativePosition() {
        boolean areWeThereYet = false;
        double err = driveMotor.getPosition().getValueAsDouble() - targetDrivePosition;
        double driveSpeed = Math.min(Math.max(10.0, (err * 0.2)), -10.0);
        SmartDashboard.putNumber(name + "/driveSpeed", driveSpeed);
        SmartDashboard.putNumber(name + "/err", err);
        if (Math.abs(targetAngleError) < 0.1) {
            System.out.println("Hello from applyRelativePostion: " + driveSpeed);
            SmartDashboard.putNumber(name + "/driveSpeed", driveSpeed);
            if (Math.abs(err) < 0.05) {
                areWeThereYet = true;
                driveMotor.setControl(
                        drivePosRequest.withPosition(driveMotor.getPosition().getValue()));
            } else {
                driveMotor.setControl(driveVoltageRequest.withOutput(driveSpeed));
            }
        }
        SmartDashboard.putBoolean("DriveSwerve/Are we there yet? (Drive)", areWeThereYet);
        return areWeThereYet;
    }

    public void applyVelocity() {
        if (targetSpeed > 0.01 || targetSpeed < -0.01) {
            running = true;
            this.driveRPS = maxRPS * targetSpeed * driveErrorScale;
            driveMotor.setControl(driveRequest.withVelocity(driveRPS));
            // System.out.println("FalconSwerveModule "+name + " applying drive velocity
            // "+driveRPS);
        } else {
            running = false;
            driveMotor
                    .setControl(drivePosRequest.withPosition(driveMotor.getPosition().getValue()));
            // driveMotor.setControl(stopRequest);
            // System.out.println("FalconSwerveModule "+name + " applying stop as targetRPS is too
            // low "+targetRPS);
        }
    }


    public void applySpeed() {
        if (targetSpeed > 0.01 || targetSpeed < -0.01) {
            running = true;
            this.driveVoltage = maxVoltage * targetSpeed * driveErrorScale;
            driveMotor.setControl(driveVoltageRequest.withOutput(driveVoltage));
            // System.out.println("FalconSwerveModule "+name + " applying drive velocity
            // "+driveRPS);
        } else {
            running = false;
            driveMotor.setControl(driveVoltageRequest.withOutput(0.0));
            // driveMotor.setControl(stopRequest);
            // System.out.println("FalconSwerveModule "+name + " applying stop as targetRPS is too
            // low "+targetRPS);
        }
    }

    public void setPosition(double targetRadians) {
        if (running) {
            targetAngleRadians = targetRadians;
            // turnMotor.set(ControlMode.Position, dir);
        }
    }

    public double getTotalCurrent() {
        return driveMotor.getSupplyCurrent().getValueAsDouble()
                + turnMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getDistanceEncoderPosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getAngleEncoderCount() {
        return turnMotor.getPosition().getValueAsDouble();
    }

    public double getWheelRPS() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getWheelAngleRadians() {
        return turnMotor.getPosition().getValueAsDouble() * Math.PI * 2.0;
    }

    public double getWheelAngleError() {
        return targetAngleError;
    }

    public double getDriveRPS() {
        return driveRPS;
    }
    public double getDriveTemp(){
        return this.driveMotor.getDeviceTemp().getValueAsDouble();
    }

    public double driveAngle(Vector target, Boolean nonoptimal) // Turns Wheel
    {
        // Fetch Target Pose (angle in radians, normalized length)
        double targetRadians = target.getAngleXYRadian();
        double targetPower = target.getLengthXY();

        // Determine the current wheel position from the cancoder (in radians)
        double canPositionRadians = turnMotor.getPosition().getValueAsDouble() * Math.PI * 2.0;
        if (targetPower > 0.02) {
            double idealRadians;
            boolean shouldSpinPos = targetRadians > canPositionRadians;
            if (shouldSpinPos) {
                idealRadians = targetRadians - (Math.PI * 2.0);
                while (idealRadians > canPositionRadians) {
                    targetRadians = idealRadians;
                    idealRadians = idealRadians - (Math.PI * 2.0);
                }
            } else {
                idealRadians = targetRadians + (Math.PI * 2.0);
                while (idealRadians < canPositionRadians) {
                    targetRadians = idealRadians;
                    idealRadians = idealRadians + (Math.PI * 2.0);
                }
            }
            double targetRadErr = Math.abs(canPositionRadians - targetRadians);
            double idealRadErr = Math.abs(canPositionRadians - idealRadians);
            if (targetRadErr < idealRadErr) {
                if (nonoptimal && targetRadErr > Math.PI / 2.0) {
                    driveReversed = true;
                    targetAngleRadians = idealRadians + (shouldSpinPos ? Math.PI : -Math.PI);
                    targetPower = -targetPower;
                } else {
                    targetAngleRadians = targetRadians;
                    driveReversed = false;
                }
            } else {
                if (nonoptimal && idealRadErr > Math.PI / 2.0) {
                    driveReversed = true;
                    targetAngleRadians = targetRadians - (shouldSpinPos ? Math.PI : -Math.PI);
                    targetPower = -targetPower;
                } else {
                    driveReversed = false;
                    targetAngleRadians = idealRadians;
                }
            }
            targetAngleError = targetAngleRadians - canPositionRadians;
            double targetRotations = (targetAngleRadians / (2.0 * Math.PI));
            turnMotor.setControl(turnRequest.withPosition(targetRotations));
            setTargetSpeed(targetPower);

        } else {
            double currPos = turnMotor.getPosition().getValueAsDouble();
            targetAngleRadians = currPos * Math.PI * 2.0;
            targetAngleError = 0.0;
            turnMotor.setControl(turnRequest.withPosition(currPos));
            // turnMotor.setControl(stopRequest);
            setTargetSpeed(0.0);
        }
        return targetAngleError;
    }

    public void brake() {
        driveMotor.setControl(driveRequest.withVelocity(0.0));
        turnMotor.setControl(turnRequest.withPosition(0.0));
    }

    public void applySettings() {
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = driveMotor.getConfigurator().apply(driveConfig);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(name + " could not apply configs drive, error code: " + status.toString());
        }
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = turnMotor.getConfigurator().apply(turnConfig);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(name + " could not apply configs turn, error code: " + status.toString());
        }
        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = canCoder.getConfigurator().apply(canCoderConfig);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(name + " could not apply configs sensor, error code: " + status.toString());
        }
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble()*WHEEL_DIAM_METER,
            new Rotation2d(getWheelAngleRadians())
        );
    }

    public void logData() {
        SmartDashboard.putNumber("DriveSwerve/driveRPS", driveRPS);

        SmartDashboard.putNumber(sdDriveTargetRPSKey, targetSpeed);
        SmartDashboard.putNumber(sdDriveMaxRPSKey, maxRPS);
        SmartDashboard.putNumber(sdDriveRPSKey, driveRPS);
        SmartDashboard.putNumber(sdDriveVoltageKey, driveVoltage);
        SmartDashboard.putNumber(sdDriveErrorScale, driveErrorScale);
        SmartDashboard.putNumber(sdDriveTempKey, driveMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(sdDriveStatorCurKey, driveMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(sdDriveSupplyCurKey, driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(sdDriveCLErrorKey, driveMotor.getClosedLoopError().getValue());
        SmartDashboard.putNumber(sdDriveSensorPosKey, driveMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(sdDriveSensorMotorVoltageKey, driveMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber(sdDriveIntAccumKey, driveMotor.getClosedLoopIntegratedOutput().getValue());
        SmartDashboard.putNumber(sdDriveSensorVelKey, driveMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(sdDriveErrorDerivKey, driveMotor.getClosedLoopError().getValue());
        SmartDashboard.putBoolean(sTurnFaultsKey, driveMotor.getFaultField().getValue() == 0);


        SmartDashboard.putNumber(sdTurnTargetPosKey, targetAngleRadians);
        SmartDashboard.putNumber(sdTurnTempKey, turnMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(sdTurnStatorCurKey, turnMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(sdTurnSupplyCurKey, turnMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(sdTurnCLErrorKey, turnMotor.getClosedLoopError().getValue());
        SmartDashboard.putNumber(sdTurnSensorPosKey, turnMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(sdTurnAdjustSensorPosKey, turnMotor.getPosition().getValueAsDouble() + canCoderRelOffset);
        SmartDashboard.putNumber(sdTurnIntAccumKey, turnMotor.getClosedLoopIntegratedOutput().getValue());
        SmartDashboard.putNumber(sdTurnErrorDerivKey, turnMotor.getClosedLoopError().getValue());
        SmartDashboard.putNumber(sdTargetAngleRadians, targetAngleRadians);
        SmartDashboard.putNumber(sdTargetAngleError, targetAngleError);
        SmartDashboard.putBoolean(sTurnFaultsKey, turnMotor.getFaultField().getValue() == 0);

        SmartDashboard.putNumber(sdCanCoderAbsPosKey, getWheelAngleRadians());
        SmartDashboard.putNumber(sdcanCoderPosKey, canCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(sdCanCoderVelKey, canCoder.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(sdCanCoderMagneticKey,
                canCoder.getMagnetHealth().getValueAsDouble());

        SmartDashboard.putBoolean(name + " enabled", running);
    }
}
