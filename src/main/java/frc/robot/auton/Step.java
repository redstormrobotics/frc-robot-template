package frc.robot.auton;

public interface Step {
    // this gets called just once 
    public void enterStep();

    // is called every loop, return true if ready to move to the next step
    public boolean isDone();
}
