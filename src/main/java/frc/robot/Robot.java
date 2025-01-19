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

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.hal.PowerDistributionVersion;
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

	public enum ROBOTNAME {
		DEFAULT, // Empty robot
	};

	// User-Side Controls
	private boolean runnable;
	private String m_autoSelected;
	private final SendableChooser<String> autonChooser = new SendableChooser<>();
	private final SendableChooser<Boolean> platformChooser = new SendableChooser<>();
	private final SendableChooser<Integer> logLevelChooser = new SendableChooser<>();

	// UI
	private Gamepad gp0;
	private Gamepad gp1;

	// Robot Hardware Attached:
	private Config config;
	private Health health;
	private DriveSwerve driveSwerve;
	private PowerDistribution pdu;

	// Operating Modes
	private ModeAuton modeAuton;
	private ModeTeleOp modeTeleOp;
	private ModeDisabled modeDisabled;
	private ModeSimulation modeSimulation;
	private ModeTest modeTest;

	// CAN bus names
	private final String RIOCAN = "rio";
	private final String DRIVECAN = "drive";

    // ===============================================================================================
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 * All hardware ID and ports should be assigned in this file.
	 */
	@Override
	public void robotInit() {
		runnable = false;

		// enable controls logging
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog(), true);

		// Add the robot specific settings here
		ROBOTNAME robotName = ROBOTNAME.DEFAULT;
		String driveCan = RIOCAN;

		// Initialize User-Side Controls
		platformChooser.setDefaultOption("Windows", true);
		platformChooser.addOption("Linux", false);
		SmartDashboard.putData("Driver Platform", platformChooser);

		// Initialize Gamepads
		gp0 = new Gamepad(0);
		gp1 = new Gamepad(1);

		// Initialize Smart Dashboard Driver Controller Platform Selection
		platformChooser.setDefaultOption("Windows", true);
		platformChooser.addOption("Linux", false);
		SmartDashboard.putData("Driver Platform", platformChooser);

		// Initialize Health Log Level Selection
		logLevelChooser.setDefaultOption("Info", Health.INFO);
		logLevelChooser.addOption("Debug", Health.DEBUG);
		logLevelChooser.addOption("Trace", Health.TRACE);
		SmartDashboard.putData("Log Level", logLevelChooser);

		// Initialization of all Hardware
		health = new Health(0);
		config = new Config(0);

		Pigeon2 gyro = null;
		// initialize drive train
		DriveSwerve driveSwerve;
		try {
			TalonFX driveFR = safelyCreateTalonFX(10, driveCan);
			TalonFX driveFL = safelyCreateTalonFX(13, driveCan);
			TalonFX driveBR = safelyCreateTalonFX(16, driveCan);
			TalonFX driveBL = safelyCreateTalonFX(19, driveCan);
			TalonFX turnFR = safelyCreateTalonFX(11, driveCan);
			TalonFX turnFL = safelyCreateTalonFX(14, driveCan);
			TalonFX turnBR = safelyCreateTalonFX(17, driveCan);
			TalonFX turnBL = safelyCreateTalonFX(20, driveCan);
			CANcoder CANCoderFR = safelyCreateCANCoder(12, driveCan);
			CANcoder CANCoderFL = safelyCreateCANCoder(15, driveCan);
			CANcoder CANCoderBR = safelyCreateCANCoder(18, driveCan);
			CANcoder CANCoderBL = safelyCreateCANCoder(21, driveCan);
			gyro = safelyCreateGyro(driveCan);
			driveSwerve = new DriveSwerveImpl(driveFR, turnFR, CANCoderFR, driveFL, turnFL,
					CANCoderFL, driveBR, turnBR, CANCoderBR, driveBL, turnBL, CANCoderBL, gyro,
					robotName);
			driveSwerve.init();
			System.out.println("drivetrain online");
		} catch (Exception e) {
			health.addError("Drive train failed", e);
			driveSwerve = new DriveSwerveDummy(); // dummy method in case of error
		}
		// Setup Power Distrubution Hub
		try {
			pdu = new PowerDistribution();
			PowerDistributionVersion pduVer = pdu.getVersion();
			if (pduVer != null && pduVer.hardwareMajor != 0) {
				Health.warning("PDU",
						"Found PDU version " + pduVer.hardwareMajor + "." + pduVer.hardwareMinor
								+ " (" + pduVer.firmwareFix + "." + pduVer.firmwareMinor + "."
								+ pduVer.firmwareFix + ")");
				pdu.clearStickyFaults();
			} else {
				pdu = null;
				Health.warning("PDU", "No PDU found! ");
			}
		} catch (Exception e) {
			Health.warning("PDU", "Unable to find PDU: " + e.getMessage());
			pdu = null;
		}

		// Initialization of all Modes
		try {
			modeAuton = new ModeAuton(config, driveSwerve);
			modeTeleOp = new ModeTeleOp(config, gp0, gp1, driveSwerve);
			modeSimulation = new ModeSimulation(config);
			modeTest = new ModeTest(config, driveSwerve);
			modeDisabled = new ModeDisabled(config);
			runnable = true;
		} catch (Exception e) {
			health.addError("Failed to Create Modes: " + e.getMessage());
			runnable = false;
		}

		// Initialize UI Auton Selection
		String auton_options[] = modeAuton.getAutonList();
		autonChooser.setDefaultOption(auton_options[0], auton_options[0]);
		for (int i = 1; i < auton_options.length; i++) {
			autonChooser.addOption(auton_options[i], auton_options[i]);
		}
		SmartDashboard.putData("Auton", autonChooser);

		System.out.println("robotInit() complete");
	}

    // ===============================================================================================

	@Override
	public void robotPeriodic() {
		health.loop(pdu);
	}

	@Override
	public void autonomousInit() {
		grabUiControls();		
		m_autoSelected = autonChooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
		modeAuton.initialize(runnable);
		modeAuton.selectAuton(m_autoSelected);
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		modeAuton.periodic();
	}

	@Override
	public void teleopInit() {
		grabUiControls();
		modeTeleOp.initialize(runnable);
	}

	@Override
	public void teleopPeriodic() {
		modeTeleOp.periodic();
	}

	@Override
	public void disabledInit() {
		grabUiControls();
		modeDisabled.initialize(runnable);
	}

	@Override
	public void disabledPeriodic() {
		modeDisabled.periodic();
	}

	@Override
	public void testInit() {
		grabUiControls();
		modeTest.initialize(runnable);
	}

	@Override
	public void testPeriodic() {
		modeTest.periodic();
	}

	@Override
	public void simulationInit() {
		grabUiControls();
		modeSimulation.initialize(runnable);
	}

	@Override
	public void simulationPeriodic() {
		modeSimulation.periodic();
	}

    // ===============================================================================================

	// Update Selections From UI (if present)
	protected void grabUiControls() {
		try {
			boolean isWindows = "Windows".equals(platformChooser.getSelected());
			gp0.selectWindows(isWindows);
			gp1.selectWindows(isWindows);
		} catch (Exception e) {
			health.addError("Could not config controls");
		}

		try {
			int healthValue = logLevelChooser.getSelected();
			Health.verbosity(healthValue);
		} catch (Exception e) {
			Health.verbosity(Health.INFO);
		}
	}

    // ===============================================================================================

	// Wrapper to create motors in a detectable way
	protected TalonFX safelyCreateTalonFX(int ID, String canbus) {
		try {
			TalonFX motor = new TalonFX(ID);
			if (motor.isConnected()) {
				System.out.println("Created TalonFX with " + ID);
				return motor;
			} else {
				System.out.println("Unable to connect to TalonFX with " + ID);
				return null;
			}
		} catch (Exception e) {
			// Ran into a problem. Return a null below
			System.out.println("Failed to create TalonFX with " + ID);
		}
		return null;
	}

	// create Falcon motor. We use these motors a lot
	protected TalonFX safelyCreateTalonFX(int ID) {
		return safelyCreateTalonFX(ID, RIOCAN);
	}

	protected VictorSPX safelyCreateVictorSPX(int ID, boolean inverted) {
		try {
			VictorSPX motor = new VictorSPX(ID);
			motor.setInverted(inverted);
			if (motor.getFirmwareVersion() > 0) {
				System.out.println("Created VictorSPX with " + ID);
				return motor;
			} else {
				System.out.println("Unable to connect to VictorSPX with " + ID);
				return null;
			}
		} catch (Exception e) {
			// Ran into a problem. Return a null below
			System.out.println("Failed to create VictorSPX with " + ID);
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
				System.out.println("Created TalonSRX with " + ID);
				return motor;
			} else {
				System.out.println("Unable to connect to TalonSRX with " + ID);
				return null;
			}
		} catch (Exception e) {
			// Ran into a problem. Return a null below
			System.out.println("Failed to create TalonSRX with " + ID);
		}
		return null;
	}

	protected TalonSRX SafelyCreateTalonSRX(int ID) {
		return safelyCreateTalonSRX(ID, false);
	}

	// create Spark Max. Brushless NEO550
	protected SparkMax safelyCreateSparkMaxBrushless(int ID) throws Exception {
		SparkMax motor = null;
		for (int tries = 0; tries < 3; tries++) { // trying 3 times
			if (motor == null) {
				motor = new SparkMax(ID, MotorType.kBrushless); // assigned motor as brushless
			}

			if (motor.getFirmwareVersion() > 0) { // if it has firmware, proceed
				return motor;
			}
			Timer.delay(0.05); // delay between tries
		}
		return null;
	}

	// create Spark Max. Brushed
	protected SparkMax safelyCreateSparkMaxBrushed(int ID) throws Exception {
		SparkMax motor = null;
		try {
			for (int tries = 0; tries < 3; tries++) { // trying 3 times
				if (motor == null) {
					motor = new SparkMax(ID, MotorType.kBrushed); // assigning motor as brushed
				}

				if (motor.getFirmwareVersion() > 0) { // if it has firmware, proceed
					return motor;
				}
				Timer.delay(0.05); // delay between tries
			}
		} catch (Throwable t) {
			System.out.println("unable to create spark max with id 40");
		}
		return null;
	}


	protected CANcoder safelyCreateCANCoder(int ID) {
		return safelyCreateCANCoder(ID, RIOCAN);
	}

	protected CANcoder safelyCreateCANCoder(int ID, String canbus) {
		try {
			CANcoder encoder = new CANcoder(ID, canbus);
			if (encoder.isConnected()) {
				System.out.println("Created CANCoder for ID " + ID);
				return encoder;
			} else {
				System.out.println("Unable to connect to CANCoder " + ID);
				return null;
			}
		} catch (Exception e) {
			// Ran into a problem. Return a null below
			System.out.println("Failed to create CANCoder " + ID);
		}
		return null;
	}

	protected Pigeon2 safelyCreateGyro() {
		return safelyCreateGyro(RIOCAN);
	}

	// creating gyro
	protected Pigeon2 safelyCreateGyro(String canbus) {
		try {
			Pigeon2 gyro = new Pigeon2(2, canbus);
			if(gyro.isConnected())
			{
				System.out.println("Created Pigeon2 gyro");
				return gyro;
			} else {
				System.out.println("Failed to connect to Pigeon2 gyro");
				return null;
			}
		} catch (Exception e) {
			// Ran into a problem. Return a null below
			System.out.println("Failed to create Pigeon2 gyro");
		}
		return null;
	}

}
