package frc.diagnostics;

import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import java.util.HashMap;
import java.util.Map;

public class DoubleSlider extends ShuffleManager {

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
    public DoubleSlider(String tab, String title, double defaultValue, double min, double max){
        widget =
        Shuffleboard.getTab(tab)
            .add(title, defaultValue)
            .withWidget("Number Slider")
            .withPosition(pos.x, pos.y)
            .withSize(2, 1);
        entry = widget.getEntry();
        Map<String, Object> properties = new HashMap<String, Object>();
        properties.put("Max", max);
        properties.put("Min", min);
        widget.withProperties(properties);
        pos.translate(2, 0);
        if(pos.x >= 7) pos.setLocation(0, pos.y + 1);
    }

    /**
     * 
     * @param title
     * @param defaultValue
     * @param min
     * @param max
     */
    public DoubleSlider(String title, double defaultValue, double min, double max){
        System.out.print(pos);
        widget =
        Shuffleboard.getTab("Config")
            .add(title, defaultValue)
            .withWidget("Number Slider")
            .withPosition(pos.x, pos.y)
            .withSize(2, 1);
        entry = widget.getEntry();
        Map<String, Object> properties = new HashMap<String, Object>();
        properties.put("Max", max);
        properties.put("Min", min);
        widget.withProperties(properties);
        pos.translate(2, 0);
        if(pos.x >= 7) pos.setLocation(1, pos.y + 2);
    }

    public double value(){
        return entry.getDouble(0);
    }
}