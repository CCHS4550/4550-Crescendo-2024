package frc.helpers;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticsSystem {
    private boolean on;

    private Solenoid solenoid1;
    private Solenoid solenoid2;

    /**
     * A class that makes managing solenoids slightly easier. Each mechanism using solenoids will need 2 solenoids in order to work.
     * @param type What type of solenoid is being used. Will almost always be CTREPCM
     * @param port1 What port the first solenoid is plugged into
     * @param port2 What port the second solenoid is plugged into
     */
    public PneumaticsSystem(PneumaticsModuleType type, int port1, int port2){
        solenoid1 = new Solenoid(type, port1);
        solenoid2 = new Solenoid(type, port2);
        on = false;
    }

    /**
     * Toggles the system from on to off or vice versa
     */
    public void toggle(){
        on = !on;
        solenoid1.set(on);
        solenoid2.set(!on);
    }

    /**
     * Sets the system to on or off
     * @param set what state the system is set to
     */
    public void set(boolean set){
        on = set;
        solenoid1.set(set);
        solenoid2.set(!set);
    }

    /**
     * 
     * @return the current state of the system.
     */
    public boolean on(){ return on; }

    private boolean triggered = false;
    /**
     * Will toggle the system once per trigger. If trigger is true, will not toggle until it is false, then true again.
     * @param trigger What will trigger the system to toggle. Suggest passing in a button or axis input.
     */
    public void triggerSystem(boolean trigger){
        if(!trigger){
            triggered = false;
            return;
        }
        if(triggered) return;
        triggered = true;
        toggle();
    }

    public void setOne(boolean set){
        solenoid1.set(set);
    }

    public void setTwo(boolean set){
        solenoid2.set(set);
    }
}
