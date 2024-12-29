package frc.robot.auton;

import edu.wpi.first.wpilibj.Timer;

public class StepSleep implements Step {

    final private Timer timer;
    final double wait;

    public StepSleep(double seconds) {
        timer = new Timer();
        wait = seconds;
    }

    @Override
    public void enterStep() {
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isDone() {
        return timer.hasElapsed(wait);
    }


    
}
