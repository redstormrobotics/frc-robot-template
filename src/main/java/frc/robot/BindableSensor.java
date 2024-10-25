package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class BindableSensor implements BindableButton {

    private DigitalInput sensor;
    private boolean inverted;

    public BindableSensor(DigitalInput sensor){
        this.sensor = sensor;
        this.inverted=false;
    }
    public BindableSensor(DigitalInput sensor, boolean inverted){
        this.sensor = sensor;
        this.inverted=inverted;
    }
    public boolean isPressed(){
        return inverted? !sensor.get(): sensor.get();
    }
}
