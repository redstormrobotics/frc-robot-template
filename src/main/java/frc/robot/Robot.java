/*============================================================
/ Red Storm Robotics 2022/23
/=============================================================
/ ROBOT
/
/ This is where all objects are initialized for the entire
/ System. 
/=============================================================
/ Hardware:
/
/ See configuration of entire system
/=============================================================
/ 3rd Party Dependencies:
/
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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Robot is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	// User-Side Controls
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();
	private Gamepad gp1;
	private Gamepad gp2;

	// Robot Hardware Attached:
	private Config config;
	private Health health;

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
	public void robotInit() 
	{
		// Initialize User-Side Controls
		m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		m_chooser.addOption("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		System.out.println("robotInit() complete");
		gp1 = new Gamepad(0);
		gp2 = new Gamepad(1);

		// Initialization of all Hardware
		health = new Health(0);
		config = new Config(0);

		// Initialization of all Modes
		modeAuton = new ModeAuton(config);
		modeTeleOp = new ModeTeleOp(config, gp1, gp2);
		modeSimulation = new ModeSimulation(config);
		modeTest = new ModeTest(config);
		modeDisabled = new ModeDisabled(config);
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() 
	{
		health.Loop();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different
	 * autonomous modes using the dashboard. The sendable chooser code works with the Java
	 * SmartDashboard.
	 * 
	 * <p>You can add additional auto modes by adding additional comparisons to the switch structure
	 * below with additional strings. If using the SendableChooser make sure to add them to the
	 * chooser code above as well.
	 */
	@Override
	public void autonomousInit() 
	{
		m_autoSelected = m_chooser.getSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		modeAuton.Initialize();
		modeAuton.selectAuton(m_autoSelected);
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() 
	{
		modeAuton.Periodic();
	}

	@Override
	public void teleopInit() 
	{
		modeTeleOp.Initialize();
	}

	@Override
	public void teleopPeriodic() 
	{
		modeTeleOp.Periodic();
	}

	@Override
	public void disabledInit() 
	{
		modeDisabled.Initialize();
	}

	@Override
	public void disabledPeriodic() 
	{
		modeDisabled.Periodic();
	}

	@Override
	public void testInit() 
	{
		modeTest.Initialize();
	}

	@Override
	public void testPeriodic() 
	{
		modeTest.Periodic();
	}

	@Override
	public void simulationInit() 
	{
		modeSimulation.Initialize();
	}

	@Override
	public void simulationPeriodic()
	{		
		modeSimulation.Periodic();
	}

}
