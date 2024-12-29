package frc.robot.auton;

import frc.robot.DriveSwerve;
import frc.robot.Vector;

public class AutonBackup implements Auton {

    final private DriveSwerve driveTrain;

    public AutonBackup(DriveSwerve driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public String getName() {
        return "Backup Default";
    }

    @Override
    public Step[] getSteps() {
        return new Step[] {
            new StepSleep(2.0),
            new StepDriveTo(driveTrain, Vector.fromCart(0.0, -6.0, 0))
        };
    }
}
