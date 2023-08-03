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

import edu.wpi.first.wpilibj.Timer;

public class ModeAuton extends Mode {

    private Timer autonTimer = new Timer();
    private DriveTrainTank driveTrain;
    private final AutonMode[] autonList;
    private AutonMode autonSelected;

    public ModeAuton(Config config, DriveTrainTank driveTrain) {
        super(config);
        this.driveTrain = driveTrain;
        this.autonList = new AutonMode[] {new AutoModeNothing(), new AutoModeBackup(),};
        this.autonSelected = this.autonList[0];
    }

    protected boolean Init() {
        autonTimer.reset();
        autonTimer.start();
        return true;
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
                return true;
            }
        }
        autonSelected = autonList[0];
        Health.warning("AutonMode", autonSelected.getName() + " (Defaulted)");
        return false;
    }

    public interface AutonMode {
        public String getName();

        public void Loop();
    }

    public class AutoModeNothing implements AutonMode {
        public String getName() {
            return "Nothing";
        }

        public void Loop() {
            if (autonTimer.get() > 3.0) {
                done();
            }
        }
    }

    protected void Loop() {
        // Run Valid Auton Modes by Name
        if (autonSelected != null) {
            autonSelected.Loop();
        } else {
            done();
        }
    }

    private void done() {

    }

    // ===============================================================================================
    // Add Modes Below
    // ===============================================================================================

    public class AutoModeBackup implements AutonMode {
        public String getName() {
            return "Backup";
        }

        public void Loop() {
            if (autonTimer.get() < 1.0) {
                Health.warning("AutonStatus", "Drive backwards");
                driveTrain.drive(-0.2, 0.0);
            } else {
                Health.warning("AutonStatus", "Stop");
                driveTrain.drive(0.0, 0.0);
                done();
            }
        }
    }
}
