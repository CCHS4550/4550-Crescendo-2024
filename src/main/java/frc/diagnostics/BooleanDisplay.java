package frc.diagnostics;

import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class BooleanDisplay extends ShuffleManager{

    private SimpleWidget widget;
    private GenericEntry entry;

    /**
     * 
     * @param tab
     * @param title
     * @param defaultValue
     * @param min
     * @param max
     */
    public BooleanDisplay(String tab, String title, Boolean defaultValue){
        widget =
        Shuffleboard.getTab(tab)
            .add(title, defaultValue)
            .withWidget("Boolean Box")
            .withPosition(pos.x, pos.y)
            .withSize(1, 1);
        entry = widget.getEntry();
        pos.translate(1, 0);
        if(pos.x >= 7) pos.setLocation(0, pos.y + 1);
    }

    /**
     * 
     * @param title
     * @param defaultValue
     * @param min
     * @param max
     */
    public BooleanDisplay(String title, Boolean defaultValue){
        System.out.print(pos);
        widget =
        Shuffleboard.getTab("Config")
            .add(title, defaultValue)
            .withWidget("Boolean Box")
            .withPosition(pos.x, pos.y)
            .withSize(1, 1);
        entry = widget.getEntry();
        pos.translate(1, 0);
        if(pos.x >= 7) pos.setLocation(1, pos.y + 2);
    }

    public boolean value(){
        return entry.getBoolean(true);
    }

    public void set(boolean value){
        entry.setBoolean(value);
    }
}