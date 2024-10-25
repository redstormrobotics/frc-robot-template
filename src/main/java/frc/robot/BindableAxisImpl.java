package frc.robot;

public class BindableAxisImpl implements BindableAxis{
    
    private Gamepad g;

    // R/L: Analog stick X/Y: axis
    //the axes with an A are adjusted
    public enum axis{   
        RX, RY, LX, LY, ARX, ARY, ALX, ALY;
    }
    private axis a;

    //Takes in a gamepad (Ex: gp1) object and an enum axis (Ex: LX)
    public BindableAxisImpl(Gamepad G, axis A){
        g=G;
        a=A;
    }
    
    //returns axis position for analog inputs 
    public double getPosition(){
        switch (this.a) {
            case RX:
                return this.g.getRightX();
            case LX:
                return this.g.getLeftX();
            case RY:
                return this.g.getRightY();
            case LY:
                return this.g.getLeftY();
            case ARX:
                return this.g.getAdjustedRightX();
            case ALX:
                return this.g.getAdjustedLeftX();    
            case ARY:
                return this.g.getAdjustedRightY();
            case ALY:
                return this.g.getAdjustedLeftY();
            default:
                return 0.0;
        }
    }
}
