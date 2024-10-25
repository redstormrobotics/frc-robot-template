package frc.robot;

public class BindableOR implements BindableButton {
    private BindableButton[] inputs;

    public BindableOR(BindableButton[] Buttons) {
        this.inputs = Buttons;
    }
    public boolean isPressed(){
        for (int i=0; i<this.inputs.length; i++) {
            if(this.inputs[i].isPressed()){
                return true;
            }
        }
         return false;
    }
}
