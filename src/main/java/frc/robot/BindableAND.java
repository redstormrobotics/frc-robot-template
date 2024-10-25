package frc.robot;

public class BindableAND implements BindableButton {
    private BindableButton[] inputs;

    public BindableAND(BindableButton[] Buttons) {
        this.inputs = Buttons;
    }
    public boolean isPressed(){
        for (int i=0; i<this.inputs.length; i++) {
            if(!this.inputs[i].isPressed()){
                return false;
            }
        }
         return true;
    }
}
