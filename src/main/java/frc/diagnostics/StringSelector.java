package frc.diagnostics;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Repurposed class of CommandSelector used for selecting a String through ShuffleBoard. 
 */
public class StringSelector extends ShuffleManager{
    private SendableChooser<String> chooser;

    public StringSelector(int notUsed, String tab, String title, String... strings){
        chooser = new SendableChooser<String>();
        Shuffleboard.getTab(tab)
            .add(title, chooser)
            .withWidget("ComboBox Chooser")
            .withPosition(pos.x, pos.y)
            .withSize(1, 1);
        pos.translate(1, 0);
        if(pos.x >= 7) pos.setLocation(0, pos.y + 1);
        for(int i = 0; i < strings.length; i++){
            chooser.addOption(strings[i], strings[i]);
        }
    }

    public StringSelector(String title, String... strings){
        chooser = new SendableChooser<String>();
        Shuffleboard.getTab("Config")
            .add(title, chooser)
            .withWidget("ComboBox Chooser")
            .withPosition(pos.x, pos.y)
            .withSize(1, 1);
        pos.translate(1, 0);
        if(pos.x >= 7) pos.setLocation(0, pos.y + 1);
        for(int i = 0; i < strings.length; i++){
            chooser.addOption(strings[i], strings[i]);
        }
    }

    public String value(){
        return chooser.getSelected();
    }

    public void add(String... strings){
        for(int i = 0; i < strings.length; i++){
            chooser.addOption(strings[i], strings[i]);
        }
    }
}
