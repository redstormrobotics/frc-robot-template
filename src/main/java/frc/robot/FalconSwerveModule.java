package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FalconSwerveModule {

    private final String name;
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turnMotor;
    private final WPI_CANCoder canCoder;
    private final TalonFXConfiguration driveConfig;
    private final TalonFXConfiguration turnConfig;

    private final SmartDashboardNumber drive_currentLimit_val;
    private final SmartDashboardNumber drive_pallowableError_val;
    private final SmartDashboardNumber drive_peakOutput_val;
    private final SmartDashboardNumber drive_loopPeriodMS_val;
    private final SmartDashboardNumber drive_iz_val;
    private final SmartDashboardNumber drive_p_val;
    private final SmartDashboardNumber drive_i_val;
    private final SmartDashboardNumber drive_d_val;
    private final SmartDashboardNumber drive_f_val;
    private final SmartDashboardNumber drive_maxIntAccum_val;
    private final SmartDashboardNumber drive_motionCruse_val;
    private final SmartDashboardNumber drive_motionAccel_val;
    private final SmartDashboardNumber drive_neutralDeadband_val;

    private final SmartDashboardNumber turn_currentLimit_val;
    private final SmartDashboardNumber turn_pallowableError_val;
    private final SmartDashboardNumber turn_peakOutput_val;
    private final SmartDashboardNumber turn_loopPeriodMS_val;
    private final SmartDashboardNumber turn_iz_val;
    private final SmartDashboardNumber turn_p_val;
    private final SmartDashboardNumber turn_i_val;
    private final SmartDashboardNumber turn_d_val;
    private final SmartDashboardNumber turn_f_val;
    private final SmartDashboardNumber turn_maxIntAccum_val;
    private final SmartDashboardNumber turn_motionCruse_val;
    private final SmartDashboardNumber turn_motionAccel_val;
    private final SmartDashboardNumber turn_neutralDeadband_val;

    private final String sdDriveTargetRPMKey;
    private final String sdDriveMaxRPMKey;
    private final String sdDriveTargetSpeedKey;
    private final String sdDriveTempKey;
    private final String sdDriveStatorCurKey;
    private final String sdDriveSupplyCurKey;
    private final String sdDriveCLErrorKey;
    private final String sdDriveSensorPosKey;
    private final String sdDriveSensorVelKey;
    private final String sdDriveIntAccumKey;
    private final String sdDriveErrorDerivKey;
    private final String sdDriveFaultsKey;
    
    private final String sdTurnTargetPosKey;
    private final String sdTurnTempKey;
    private final String sdTurnStatorCurKey;
    private final String sdTurnSupplyCurKey;
    private final String sdTurnCLErrorKey;
    private final String sdTurnSensorPosKey;
    private final String sdTurnSensorVelKey;
    private final String sdTurnIntAccumKey;
    private final String sdTurnErrorDerivKey;
    private final String sTurnFaultsKey;

    private final String sdCanCoderAbsPosKey;
    private final String sdcanCoderPosKey;
    private final String sdCanCoderVelKey;
    private final String sdCanCoderMagneticKey;


    private double maxRPM = 6000.0;
    private double targetRPM = 6000.0;
    private double speed = 0.0;
    private double dir = 0.0;
    private boolean running = false;

    public FalconSwerveModule(String name, WPI_TalonFX driveMotor, WPI_TalonFX turnMotor, WPI_CANCoder canCoder) {
        this.name = name;
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.canCoder = canCoder;

        this.sdDriveTargetRPMKey = name + "/Drive/RPM Target";
        this.sdDriveMaxRPMKey = name + "/Drive/RPM Max";
        this.sdDriveTargetSpeedKey = name + "/Drive/Speed target";
        this.sdDriveTempKey = name + "/Drive/Temp";
        this.sdDriveStatorCurKey = name + "/Drive/Current stator";
        this.sdDriveSupplyCurKey = name + "/Drive/Current supply";
        this.sdDriveCLErrorKey = name + "/Drive/Error closed loop";
        this.sdDriveSensorPosKey = name + "/Drive/Position sensor";
        this.sdDriveSensorVelKey = name + "/Drive/Velocity sensor";
        this.sdDriveIntAccumKey = name + "/Drive/Integral accumulator sum";
        this.sdDriveErrorDerivKey = name + "/Drive/Error derivative";
        this.sdDriveFaultsKey = name + "/Drive/Fault";

        this.sdTurnTempKey = name + "/Turn/Temp";
        this.sdTurnTargetPosKey = name + "/Turn/Position target";
        this.sdTurnStatorCurKey = name + "/Turn/Current statort";
        this.sdTurnSupplyCurKey = name + "/Turn/Current supply";
        this.sdTurnCLErrorKey = name + " /Turn/Loop error closed";
        this.sdTurnSensorPosKey = name + "/Turn/Position sensor";
        this.sdTurnSensorVelKey = name + "/Turn/Velocity sensor";
        this.sdTurnIntAccumKey = name + "/Turn/Integral accumulator sum";
        this.sdTurnErrorDerivKey = name + "/Turn/Error derivative";
        this.sTurnFaultsKey = name + "/Turn/Fault";
    
        this.sdCanCoderAbsPosKey = name + "/CANCoder/Position absolute";
        this.sdcanCoderPosKey = name + "/CANCoder/Position ticks";
        this.sdCanCoderVelKey = name + "/CANCoder/Velocity";
        this.sdCanCoderMagneticKey = name + "/CANCoder/Magnetic strength";

        drive_currentLimit_val = new SmartDashboardNumber(name + "/Drive/Error allowable (ticks)", 5.0);
        drive_pallowableError_val = new SmartDashboardNumber(name + "/Drive/Error allowable ticks", 0.001);
        drive_peakOutput_val = new SmartDashboardNumber(name + "/Drive/Current peak output", 1.0);
        drive_loopPeriodMS_val = new SmartDashboardNumber(name + "/Drive/PID loop period MS", 1);
        drive_iz_val = new SmartDashboardNumber(name + "/Drive/Integral zone", 300);
        drive_p_val = new SmartDashboardNumber(name + "/Drive/Gain P", 0.1);
        drive_i_val = new SmartDashboardNumber(name + "/Drive/Gain I", 0.001);
        drive_d_val = new SmartDashboardNumber(name + "/Drive/Gain D", 5.0);
        drive_f_val = new SmartDashboardNumber(name + "/Drive/Gain F", 1023.0/6800.0);
        drive_maxIntAccum_val = new SmartDashboardNumber(name + "/Drive/integral accumulator max", 0);
        drive_motionCruse_val = new SmartDashboardNumber(name + "/Drive/Mot Cruse ticks_100ms", 6000);
        drive_motionAccel_val = new SmartDashboardNumber(name + "/Drive/Mot Accel ticks_100ms", 2000);
        drive_neutralDeadband_val = new SmartDashboardNumber(name + "/Drive/Mot Deadband [0.001-0.25]", 0.25);

        turn_currentLimit_val = new SmartDashboardNumber(name + "/Turn/Current limit", 5.0);
        turn_pallowableError_val = new SmartDashboardNumber(name + "/Turn/Error allowable ticks", 0.001);
        turn_peakOutput_val = new SmartDashboardNumber(name + "/Turn/Current peak output", 1.0);
        turn_loopPeriodMS_val = new SmartDashboardNumber(name + "/Turn/PID loop period MS", 1);
        turn_iz_val = new SmartDashboardNumber(name + "/Turn/Integral zone", 200);
        turn_p_val = new SmartDashboardNumber(name + "/Turn/Gain P", 0.1);
        turn_i_val = new SmartDashboardNumber(name + "/Turn/Gain I", 0.0);
        turn_d_val = new SmartDashboardNumber(name + "/Turn/Gain D", 0.0);
        turn_f_val = new SmartDashboardNumber(name + "/Turn/Gain F", 0.0);
        turn_maxIntAccum_val = new SmartDashboardNumber(name + "/Turn/Integral accumulator max", 0);
        turn_motionCruse_val = new SmartDashboardNumber(name + "/Turn/Mot Cruse ticks_100ms", 6000);
        turn_motionAccel_val = new SmartDashboardNumber(name + "/Turn/Mot Accel ticks_100ms", 2000);
        turn_neutralDeadband_val = new SmartDashboardNumber(name + "/Turn/Mot Deadband [0.001-0.25]", 0.25);

        driveConfig = new TalonFXConfiguration();
        turnConfig = new TalonFXConfiguration();

        driveConfig.supplyCurrLimit.enable = true;
        driveConfig.supplyCurrLimit.triggerThresholdTime = 0.5; // the time at the peak supply current before the limit triggers, in sec
        driveConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        driveConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        driveConfig.feedbackNotContinuous = false;
        driveConfig.nominalOutputForward = 1.0;
        driveConfig.nominalOutputReverse = -1.0;
        driveConfig.integratedSensorOffsetDegrees = -70.84;
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveConfig.auxiliaryPID.selectedFeedbackSensor= FeedbackDevice.IntegratedSensor;


        turnConfig.supplyCurrLimit.enable = true;
        turnConfig.supplyCurrLimit.triggerThresholdTime = 0.5; // the time at the peak supply current before the limit triggers, in sec
        turnConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        turnConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        turnConfig.feedbackNotContinuous = false;
        turnConfig.nominalOutputForward = 1.0;
        turnConfig.nominalOutputReverse = -1.0;
        turnConfig.integratedSensorOffsetDegrees = -70.84;
        turnConfig.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
        turnConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        turnConfig.remoteFilter1.remoteSensorDeviceID = canCoder.getDeviceID();
        turnConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.CANCoder;
        turnConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        turnConfig.auxiliaryPID.selectedFeedbackSensor= FeedbackDevice.IntegratedSensor;

    }

    public void init() {
        // reset all settings
        driveMotor.configFactoryDefault();
        turnMotor.configFactoryDefault();
        canCoder.configFactoryDefault();

        driveMotor.setInverted(false);
        turnMotor.setInverted(false);
        
        updateSettings();
        
        driveMotor.configAllSettings(driveConfig);
        turnMotor.configAllSettings(turnConfig);
    }

    public void setRPM(double rpm) {
        this.maxRPM = rpm;
        this.targetRPM = rpm * speed;
        SmartDashboard.putNumber(sdDriveTargetRPMKey, rpm);
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        this.targetRPM = maxRPM * speed;
    }

    public void turnOff() {
        running = false;
        speed = 0.0;
        targetRPM = 0.0;
        driveMotor.set(ControlMode.Velocity, targetRPM);
    }

    public void turnOn() {
        running = true;
        driveMotor.set(ControlMode.Velocity, targetRPM);
    }

    public void setPosition(double pos) {
        if(running) {
            dir = pos;
            turnMotor.set(ControlMode.Position, dir);
        }
    }

    public void updateSettings() {
        driveConfig.supplyCurrLimit.triggerThresholdCurrent = drive_currentLimit_val.getNumber(); // the peak supply current, in amps
        driveConfig.supplyCurrLimit.currentLimit = drive_currentLimit_val.getNumber(); // the current to maintain if the peak supply limit is triggered
        driveConfig.motionCruiseVelocity = drive_motionCruse_val.getNumber();
        driveConfig.motionAcceleration = drive_motionAccel_val.getNumber();
        driveConfig.neutralDeadband = drive_neutralDeadband_val.getNumber();
        driveConfig.slot0.allowableClosedloopError = drive_pallowableError_val.getNumber();
        driveConfig.slot0.closedLoopPeakOutput = drive_peakOutput_val.getNumber();
        driveConfig.slot0.closedLoopPeriod = (int) drive_loopPeriodMS_val.getNumber();
        driveConfig.slot0.integralZone = drive_iz_val.getNumber();
        driveConfig.slot0.kD = drive_d_val.getNumber();
        driveConfig.slot0.kF = drive_f_val.getNumber();
        driveConfig.slot0.kI = drive_i_val.getNumber();
        driveConfig.slot0.kP = drive_p_val.getNumber();
        driveConfig.slot0.maxIntegralAccumulator = drive_maxIntAccum_val.getNumber();

        turnConfig.supplyCurrLimit.triggerThresholdCurrent = turn_currentLimit_val.getNumber(); // the peak supply current, in amps
        turnConfig.supplyCurrLimit.currentLimit = turn_currentLimit_val.getNumber(); // the current to maintain if the peak supply limit is triggered
        turnConfig.motionCruiseVelocity = turn_motionCruse_val.getNumber();
        turnConfig.motionAcceleration = turn_motionAccel_val.getNumber();
        turnConfig.neutralDeadband = turn_neutralDeadband_val.getNumber();
        turnConfig.slot0.allowableClosedloopError = turn_pallowableError_val.getNumber();
        turnConfig.slot0.closedLoopPeakOutput = turn_peakOutput_val.getNumber();
        turnConfig.slot0.closedLoopPeriod = (int) turn_loopPeriodMS_val.getNumber();
        turnConfig.slot0.integralZone = turn_iz_val.getNumber();
        turnConfig.slot0.kD = turn_d_val.getNumber();
        turnConfig.slot0.kF = turn_f_val.getNumber();
        turnConfig.slot0.kI = turn_i_val.getNumber();
        turnConfig.slot0.kP = turn_p_val.getNumber();
        turnConfig.slot0.maxIntegralAccumulator = turn_maxIntAccum_val.getNumber();

        driveMotor.configAllSettings(driveConfig);
        turnMotor.configAllSettings(turnConfig);
    }

    public void logData() {
        SmartDashboard.putNumber(sdDriveTargetRPMKey, targetRPM);
        SmartDashboard.putNumber(sdDriveMaxRPMKey, maxRPM);
        SmartDashboard.putNumber(sdDriveTargetSpeedKey, speed);
        SmartDashboard.putNumber(sdDriveTempKey, driveMotor.getTemperature());
        SmartDashboard.putNumber(sdDriveStatorCurKey, driveMotor.getStatorCurrent());
        SmartDashboard.putNumber(sdDriveSupplyCurKey, driveMotor.getSupplyCurrent());
        SmartDashboard.putNumber(sdDriveCLErrorKey, driveMotor.getClosedLoopError());
        SmartDashboard.putNumber(sdDriveSensorPosKey, driveMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber(sdDriveIntAccumKey, driveMotor.getIntegralAccumulator());
        SmartDashboard.putNumber(sdDriveSensorVelKey, driveMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber(sdDriveErrorDerivKey, driveMotor.getErrorDerivative());
        Faults driveFaults = new Faults();
        driveMotor.getFaults(driveFaults);
        SmartDashboard.putBoolean(sdDriveFaultsKey, driveFaults.toBitfield() != 0);
        
        SmartDashboard.putNumber(sdTurnTargetPosKey, dir);
        SmartDashboard.putNumber(sdTurnTempKey, turnMotor.getTemperature());
        SmartDashboard.putNumber(sdTurnStatorCurKey, turnMotor.getStatorCurrent());
        SmartDashboard.putNumber(sdTurnSupplyCurKey, turnMotor.getSupplyCurrent());
        SmartDashboard.putNumber(sdTurnCLErrorKey, turnMotor.getClosedLoopError());
        SmartDashboard.putNumber(sdTurnSensorPosKey, turnMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber(sdTurnIntAccumKey, turnMotor.getIntegralAccumulator());
        SmartDashboard.putNumber(sdTurnSensorVelKey, turnMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber(sdTurnErrorDerivKey, turnMotor.getErrorDerivative());
        Faults turnFaults = new Faults();
        turnMotor.getFaults(turnFaults);
        SmartDashboard.putBoolean(sTurnFaultsKey, turnFaults.toBitfield() != 0);

        SmartDashboard.putNumber(sdCanCoderAbsPosKey, canCoder.getAbsolutePosition());
        SmartDashboard.putNumber(sdcanCoderPosKey, canCoder.getPosition());
        SmartDashboard.putNumber(sdCanCoderVelKey, canCoder.getVelocity());
        SmartDashboard.putNumber(sdCanCoderMagneticKey, canCoder.getMagnetFieldStrength().value);
        
        SmartDashboard.putBoolean(name + " enabled", running);
    }
}
