/*
 * ===============================================================================================
 * Red Storm Robotics 2020/21
 * ===============================================================================================
 * AUTONOMOUS MODE / / This contains the Autonomous controls and drivers
 * ===============================================================================================
 * Hardware: / / DriveTrainTank
 * ===============================================================================================
 * 3rd Party Dependencies: None
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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auton.*;

public class ModeAuton extends Mode {

    private final Auton[] autonList;
    private Auton autonSelected;
    private final SendableChooser<String> autoChooser;
    
    private int lastStep;
    private int currStep;
    private Step[] currSteps;

    public ModeAuton(Config config, DriveSwerve driveTrain) {
        super(config);
        // Add new auton modes here
        this.autonList = new Auton[] {
            new AutonEmpty(),
            new AutonBackup(driveTrain),
        };
        this.autonSelected = this.autonList[0];
        this.autoChooser = new SendableChooser<>();
        for (int i = 0; i < autonList.length; i++) {
            this.autoChooser.addOption(autonList[i].getName(), autonList[i].getName());
        }
        this.autoChooser.setDefaultOption(autonList[0].getName(), autonList[0].getName());
        SmartDashboard.putData("Auton", this.autoChooser);

        currSteps = new Step[0];
        lastStep = -1;
        currStep = 0;
    }

    protected boolean init() {
        currSteps = autonSelected.getSteps();
        lastStep = -1;
        currStep = 0;
        return selectAuton(autoChooser.getSelected());
    }

    public String[] getAutonList() {
        String[] retval = new String[autonList.length];
        for (int i = 0; i < retval.length; i++) {
            retval[i] = autonList[i].getName();
        }
        return retval;
    }

    public boolean selectAuton(String mode) {
        for (int i = 0; i < autonList.length; i++) {
            if (autonList[i].getName().equals(mode)) {
                autonSelected = autonList[i];
                Health.warning("AutonMode", autonSelected.getName());
                currSteps = autonSelected.getSteps();
                currStep = 0;
                return true;
            }
        }
        autonSelected = autonList[0];
        currSteps = autonSelected.getSteps();
        currStep = 0;
        Health.warning("AutonMode", autonSelected.getName() + " (Defaulted)");
        return false;
    }

    protected void loop() {
        if(currStep < currSteps.length) {
            if(currSteps[currStep].isDone()) {
                currStep++;
            }
        }
        if(lastStep < currStep) {
            currSteps[currStep].enterStep();
            lastStep = currStep;
        }
    }


}
