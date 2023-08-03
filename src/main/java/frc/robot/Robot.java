/*
 * ===============================================================================================
 * Red Storm Robotics 2022/23
 * ===============================================================================================
 * ROBOT / / This is where all objects are initialized for the entire / System.
 * ===============================================================================================
 * Hardware: / / See configuration of entire system
 * ===============================================================================================
 * 3rd Party Dependencies: /
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

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Robot is configured to automatically run this class, and to call the functions corresponding
 * to each mode, as described in the TimedRobot documentation. If you change the name of this class
 * or the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	// User-Side Controls
	private boolean runnable;
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();
	private final SendableChooser<Boolean> m_driverWindows = new SendableChooser<>();
	private Gamepad gp0;
	private Gamepad gp1;

	// Robot Hardware Attached:
	private Config config;
	private Health health;
	private DriveTrainTank driveTrain;
	private PowerDistribution pdu;

	// Operating Modes
	private ModeAuton modeAuton;
	private ModeTeleOp modeTeleOp;
	private ModeDisabled modeDisabled;
	private ModeSimulation modeSimulation;
	private ModeTest modeTest;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		runnable = false;

		// enable controls logging
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog(), true);

		// Initialize User-Side Controls
		m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		m_chooser.addOption("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		m_driverWindows.setDefaultOption("Windows", true);
		m_driverWindows.addOption("Linux", false);
		SmartDashboard.putData("Driver Platform", m_driverWindows);

		// Initialize Gamepads
		gp0 = new Gamepad(0);
		gp1 = new Gamepad(1);

		// Initialization of all Hardware
		health = new Health(0);
		config = new Config(0);

		// initialize drive train
		try {
			BaseMotorController driveFR = safelyCreateVictorSPX(10);
			BaseMotorController driveBR = safelyCreateVictorSPX(11);
			BaseMotorController driveBL = safelyCreateVictorSPX(12);
			BaseMotorController driveFL = safelyCreateVictorSPX(13);
			if (driveFL != null && driveFR != null) {
				// we can create a drive train
				BaseMotorController[] left =
						driveBL != null ? new BaseMotorController[] {driveFL, driveBL}
								: new BaseMotorController[] {driveFL};
				BaseMotorController[] right =
						driveBR != null ? new BaseMotorController[] {driveFR, driveBR}
								: new BaseMotorController[] {driveFR};

				driveTrain = new DriveTrainTankBasicController(left, right);
			} else {
				health.addError("Unable to find motor controllers for drive train");
				driveTrain = new DriveTrainTankVirtual();
			}
		} catch (Exception e) {
			health.addError("Drive train failed", e);
			driveTrain = new DriveTrainTankVirtual();
		}
		// Setup Power Distrubution Hub
		try {
			pdu = new PowerDistribution();
			pdu.clearStickyFaults();
		} catch (Exception e) {
			health.warning("PDU", "Unable to find PDU: " + e.getMessage());
			pdu = null;
		}

		// Initialization of all Modes
		try {
			modeAuton = new ModeAuton(config, driveTrain);
			modeTeleOp = new ModeTeleOp(config, gp0, gp1, driveTrain);
			modeSimulation = new ModeSimulation(config);
			modeTest = new ModeTest(config);
			modeDisabled = new ModeDisabled(config);
			runnable = true;
		} catch (Exception e) {
			health.addError("Failed to Create Modes: " + e.getMessage());
			runnable = false;
		}

		System.out.println("robotInit() complete");
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		health.loop(pdu);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different
	 * autonomous modes using the dashboard. The sendable chooser code works with the Java
	 * SmartDashboard.
	 * 
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the switch structure
	 * below with additional strings. If using the SendableChooser make sure to add them to the
	 * chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		modeAuton.Initialize(runnable);
		modeAuton.selectAuton(m_autoSelected);
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		modeAuton.Periodic();
	}

	@Override
	public void teleopInit() {
		gp0.selectWindows(m_driverWindows.getSelected());
		gp1.selectWindows(m_driverWindows.getSelected());
		modeTeleOp.Initialize(runnable);
	}

	@Override
	public void teleopPeriodic() {
		modeTeleOp.Periodic();
	}

	@Override
	public void disabledInit() {
		modeDisabled.Initialize(runnable);
	}

	@Override
	public void disabledPeriodic() {
		modeDisabled.Periodic();
	}

	@Override
	public void testInit() {
		gp0.selectWindows(m_driverWindows.getSelected());
		gp1.selectWindows(m_driverWindows.getSelected());
		modeTest.Initialize(runnable);
	}

	@Override
	public void testPeriodic() {
		modeTest.Periodic();
	}

	@Override
	public void simulationInit() {
		gp0.selectWindows(m_driverWindows.getSelected());
		gp1.selectWindows(m_driverWindows.getSelected());
		modeSimulation.Initialize(runnable);
	}

	@Override
	public void simulationPeriodic() {
		modeSimulation.Periodic();
	}

	// Wrapper to create motors in a detectable way
	protected TalonFX safelyCreateTalonFX(int ID, boolean inverted) {
		try {
			TalonFX motor = new TalonFX(ID);
			motor.setInverted(inverted);
			if (motor.getFirmwareVersion() > 0) {
				return motor;
			}
		} catch (Exception e) {
			// Ran into a problem. Return a null below
		}
		return null;
	}

	protected TalonFX safelyCreateTalonFX(int ID) {
		return safelyCreateTalonFX(ID, false);
	}

	protected VictorSPX safelyCreateVictorSPX(int ID, boolean inverted) {
		try {
			VictorSPX motor = new VictorSPX(ID);
			motor.setInverted(inverted);
			if (motor.getFirmwareVersion() > 0) {
				return motor;
			}
		} catch (Exception e) {
			// Ran into a problem. Return a null below
		}
		return null;
	}

	protected VictorSPX safelyCreateVictorSPX(int ID) {
		return safelyCreateVictorSPX(ID, false);
	}

	protected TalonSRX safelyCreateTalonSRX(int ID, boolean inverted) {
		try {
			TalonSRX motor = new TalonSRX(ID);
			motor.setInverted(inverted);
			if (motor.getFirmwareVersion() > 0) {
				return motor;
			}
		} catch (Exception e) {
			// Ran into a problem. Return a null below
		}
		return null;
	}

	protected TalonSRX SafelyCreateTalonSRX(int ID) {
		return safelyCreateTalonSRX(ID, false);
	}

	protected CANSparkMax safelyCreateSparkMax(int ID) throws Exception {
		CANSparkMax motor = null;
		for (int tries = 0; tries < 3; tries++) {
			if (motor == null) {
				motor = new CANSparkMax(ID, MotorType.kBrushless);
			}

			if (motor.getFirmwareVersion() > 0) {
				return motor;
			}
			Timer.delay(0.05);
		}
		throw new Exception("SparkMax " + String.valueOf(ID) + " Not Found");
	}

}
