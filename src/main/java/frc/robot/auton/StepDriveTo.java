package frc.robot.auton;

import frc.robot.Vector;
import frc.robot.DriveSwerve;
import frc.robot.DriveSwerve.MODE;

public class StepDriveTo implements Step {

    final private DriveSwerve driveTrain;
    final private Vector dir;

    public StepDriveTo(DriveSwerve driveTrain, Vector dir) {
        this.driveTrain = driveTrain;
        this.dir = dir;
    }

    @Override
    public void enterStep() {
        driveTrain.drive(MODE.DRIVETOPOSITION, dir, 0.0);
    }

    @Override
    public boolean isDone() {
        return driveTrain.isAtPosition();
    }
    
}
