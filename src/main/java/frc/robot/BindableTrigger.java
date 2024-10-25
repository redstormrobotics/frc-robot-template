package frc.robot;

public class BindableTrigger implements BindableButton, BindableAxis{
    private Gamepad g;  
    private double dist; //distance threshold to return pressed
    public enum trigger{
        LT, RT;
    }
    private trigger t;

    //Takes in a gamepad (Ex: gp1), Enum trigger (Ex: RT) and a Double
    public BindableTrigger(Gamepad G, trigger T, double d){
        g=G;
        t=T;
        dist=d;

    }
    
    //returns trigger position for analog inputs 
    public double getPosition(){
        switch (this.t) {
            case RT:
                return this.g.getRT();
            case LT:
                return this.g.getLT();
            default:
                return 0.0;
        }
    }
    //returns if the trigger is past the threshold or pulled
    public boolean isPressed(){
           switch (this.t) {
            case RT:
                return this.g.getRT()>=this.dist;
            case LT:
                return this.g.getLT()>=this.dist;
            default:
                return false;
        }     
    }
}
