package frc.robot;

public class BindableDirection implements BindableButton{
    private Gamepad g;
    private Gamepad.Direction d; //Dpad direction


    //Takes in a gamepad (Ex: gp1) object and a gamepad button (Ex: Gamepad.Button.A)
    public BindableDirection(Gamepad G, Gamepad.Direction D){
        g=G;
        d=D;
    }
    
    //returns axis position for analog inputs 
    public boolean isPressed(){
        return g.DpadDirectionIsPressed(this.d);
    }
}
