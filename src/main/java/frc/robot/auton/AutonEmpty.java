package frc.robot.auton;

public class AutonEmpty implements Auton {

    @Override
    public String getName() {
        return "Do nothing";
    }

    @Override
    public Step[] getSteps() {
        return new Step[0];
    }
}
