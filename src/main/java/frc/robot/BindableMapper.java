package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BindableMapper {
    private Gamepad gp0;
    private Gamepad gp1;
    private HashMap<Integer, DigitalInput>digitalinputs;
    private HashMap<DRIVE_BINDINGS, HashMap<BTN_ACTION, BindableButton>> drvbtnbindset;
    private HashMap<DRIVE_BINDINGS, HashMap<AXIS_ACTION, BindableAxis>> drvaxisbindset;
    private HashMap<MECH_BINDINGS, HashMap<BTN_ACTION, BindableButton>> mechbtnbindset;
    private HashMap<MECH_BINDINGS, HashMap<AXIS_ACTION, BindableAxis>> mechaxisbindset;
    private SendableChooser<String> m_bindset;
    private SendableChooser<String> d_bindset;

    public enum BTN_ACTION {
        //Put button actions here
        ACTIVATE_THINGY
    }

    public enum AXIS_ACTION {
        //put joystick/analog actions here
        DRIVEX,
        DRIVEY,
        TURN,
    }

    public enum MECH_BINDINGS {
        MECH_DEFAULT, // Default stable controls
        CAM, // Cam's personal preferences (Don't touch these or he may flay you alive...)
    }
    public enum DRIVE_BINDINGS {
        DRIVE_DEFAULT, // Default stable controls
    }

    public BindableMapper(Gamepad gp0, Gamepad gp1, SendableChooser<String> m_chosen, SendableChooser<String> d_chosen, HashMap<Integer, DigitalInput>digitalinputs) {
        this.gp0 = gp0;
        this.gp1 = gp1;
        this.digitalinputs=digitalinputs;
        // Button bindings

        //base sensors
        HashMap<BTN_ACTION, BindableButton> baseSensorButtons = new HashMap<>();
        //baseSensorButtons.put(BTN_ACTION.<thing>, new BindableSensor(digitalinputs.get(sensor #)), true));


        // CAM
        HashMap<BTN_ACTION, BindableButton> cam = new HashMap<>(baseSensorButtons);
        cam.put(BTN_ACTION.ACTIVATE_THINGY, new BindableButtonbImpl(gp1, Gamepad.Button.X, 1.0));

        // MECH_DEFAULT
        HashMap<BTN_ACTION, BindableButton> mdef = new HashMap<>(baseSensorButtons);
        mdef.put(BTN_ACTION.ACTIVATE_THINGY, new BindableButtonbImpl(gp1, Gamepad.Button.A, 1.0));
        // Axis bindings

        // CAM
        HashMap<AXIS_ACTION,BindableAxis> camaxis= new HashMap<>();

        // MECH_DEFAULT
        HashMap<AXIS_ACTION,BindableAxis> mdefaxis= new HashMap<>();

        // DRIVE_DEFAULT
        HashMap<AXIS_ACTION,BindableAxis> ddefaxis= new HashMap<>();
        ddefaxis.put(AXIS_ACTION.DRIVEX, new BindableAxisImpl(gp0, BindableAxisImpl.axis.LX));
        ddefaxis.put(AXIS_ACTION.DRIVEY, new BindableAxisImpl(gp0, BindableAxisImpl.axis.LY));
        ddefaxis.put(AXIS_ACTION.TURN, new BindableAxisImpl(gp0, BindableAxisImpl.axis.RX));

        // Adding bindings to bindsets

        this.mechbtnbindset = new HashMap<>();
        this.mechbtnbindset.put(MECH_BINDINGS.CAM, cam);
        this.mechbtnbindset.put(MECH_BINDINGS.MECH_DEFAULT, mdef);

        this.drvaxisbindset = new HashMap<>();
        this.drvaxisbindset.put(DRIVE_BINDINGS.DRIVE_DEFAULT, ddefaxis);

        // Adding Bindsets to ShuffleBoard
        
        //MECH
        this.m_bindset = m_chosen;
        this.m_bindset.addOption("CAM", MECH_BINDINGS.CAM.name());
        this.m_bindset.setDefaultOption("DEFAULT", MECH_BINDINGS.MECH_DEFAULT.name());

        //DRIVE
        this.d_bindset = d_chosen;
        this.d_bindset.setDefaultOption("DEFAULT", DRIVE_BINDINGS.DRIVE_DEFAULT.name());

        SmartDashboard.putData("Mech Binds", m_bindset);
        SmartDashboard.putData("Drive Binds", d_bindset);
    }

    public BindableButton getButton(BTN_ACTION verb, boolean isMech) {
        if(isMech){
            String bind = m_bindset.getSelected();
            try {
                MECH_BINDINGS bset = MECH_BINDINGS.valueOf(bind);
                if (bset != null) {
                    HashMap<BTN_ACTION, BindableButton> btnset = mechbtnbindset.get(bset);
                    if (btnset != null) {
                        BindableButton b = btnset.get(verb);
                        if (b != null) {
                            return b;
                        }
                    } else {
                        Health.getHealth().addError("INVALID BUTTON CALL " + bind + " " + verb);
                        return new BindableButtonDummy();
                    }
                }
                Health.getHealth().addError("NO MECH BINDSET SELECTED, PLEASE SELECT A BINDSET.");
                return new BindableButtonDummy();
            } catch (Exception e) {
                Health.getHealth().addError("NO MECH BINDSET SELECTED, PLEASE SELECT A BINDSET.");
                return new BindableButtonDummy();
            }
        }else{
            String bind = d_bindset.getSelected();
            try {
                DRIVE_BINDINGS bset = DRIVE_BINDINGS.valueOf(bind);
                if (bset != null) {
                    HashMap<BTN_ACTION, BindableButton> btnset = drvbtnbindset.get(bset);
                    if (btnset != null) {
                        BindableButton b = btnset.get(verb);
                        if (b != null) {
                            return b;
                        }
                    } else {
                        Health.getHealth().addError("INVALID BUTTON CALL " + bind + " " + verb);
                        return new BindableButtonDummy();
                    }
                }
                Health.getHealth().addError("NO DRIVE BINDSET SELECTED, PLEASE SELECT A BINDSET.");
                return new BindableButtonDummy();
            } catch (Exception e) {
                Health.getHealth().addError("NO DRIVE BINDSET SELECTED, PLEASE SELECT A BINDSET.");
                return new BindableButtonDummy();
            }
        }

    }

    public BindableAxis getAxis(AXIS_ACTION verb, boolean isMech) {
        if(isMech){  // for mech bindings
            String bind = m_bindset.getSelected();
            try {
                MECH_BINDINGS aset = MECH_BINDINGS.valueOf(bind);
                if (aset != null) {
                    HashMap<AXIS_ACTION, BindableAxis> axisset = mechaxisbindset.get(MECH_BINDINGS.valueOf(bind));
                    if (axisset != null) {
                        BindableAxis a = axisset.get(verb);
                        if (a != null) {
                            return a;
                        }
                    } else {
                        Health.getHealth().addError("INVALID AXIS CALL " + bind + " " + verb);
                        return new BindableAxisDummy();
                    }
                }
                Health.getHealth().addError("NO MECH BINDSET SELECTED, PLEASE SELECT A BINDSET.");
                return new BindableAxisDummy();
            } catch (Exception e) {
                Health.getHealth().addError("NO MECH BINDSET SELECTED, PLEASE SELECT A BINDSET.");
                return new BindableAxisDummy();
            }
        }else{ //For drive bindings
            String bind = d_bindset.getSelected();
            try {
                DRIVE_BINDINGS aset = DRIVE_BINDINGS.valueOf(bind);
                if (aset != null) {
                    HashMap<AXIS_ACTION, BindableAxis> axisset = drvaxisbindset.get(DRIVE_BINDINGS.valueOf(bind));
                    if (axisset != null) {
                        BindableAxis a = axisset.get(verb);
                        if (a != null) {
                            return a;
                        }
                    } else {
                        Health.getHealth().addError("INVALID AXIS CALL " + bind + " " + verb);
                        return new BindableAxisDummy();
                    }
                }
                Health.getHealth().addError("NO DRIVE BINDSET SELECTED, PLEASE SELECT A BINDSET.");
                return new BindableAxisDummy();
            } catch (Exception e) {
                Health.getHealth().addError("NO DRIVE BINDSET SELECTED, PLEASE SELECT A BINDSET.");
                return new BindableAxisDummy();
            }
        }
    }

    // public void logTelemetry() {
    //     if (this.sensor1 != null) {
    //         SmartDashboard.putBoolean("sensor1", sensor1.get());
    //     }
    //     if (this.sensor0 != null) {
    //         SmartDashboard.putBoolean("sensor0", sensor0.get());
    //     }
    // }

}
