package frc.robot;

public class BindableButtonbImpl implements BindableButton, BindableAxis {
    private Gamepad g;
    private Gamepad.Button b;
    private double AxialOut;

    //Takes in a gamepad (Ex: gp1) object and a gamepad button (Ex: Gamepad.Button.A)
    public BindableButtonbImpl(Gamepad G, Gamepad.Button B){
        g=G;
        b=B;
        AxialOut=1.0;
    }

    public BindableButtonbImpl(Gamepad G, Gamepad.Button B, double A){
        g=G;
        b=B;
        AxialOut=A;
    }

    
    
    //returns whether the button on the gamepad is pressed
    public boolean isPressed(){
        return this.g.isPressed(this.b); 
    }

    public double getPosition(){
        if(this.g.isPressed(this.b)){
            return this.AxialOut;
        }
        return 0.0;
    }
}
