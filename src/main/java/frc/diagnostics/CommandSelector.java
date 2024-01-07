package frc.diagnostics;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandSelector extends ShuffleManager{
    private SendableChooser<Command> chooser;

    /**
     * 
     * @param tab 
     * @param title
     * @param defaultValue
     * @param min
     * @param max
     */
    public CommandSelector(String tab, String title, Command... commands){
        chooser = new SendableChooser<Command>();
        Shuffleboard.getTab(tab)
            .add(title, chooser)
            .withWidget("ComboBox Chooser")
            .withPosition(pos.x, pos.y)
            .withSize(1, 1);
        pos.translate(1, 0);
        if(pos.x >= 7) pos.setLocation(0, pos.y + 1);
        for(int i = 0; i < commands.length; i++){
            chooser.addOption(commands[i].getName(), commands[i]);
        }
    }

    /**
     * 
     * @param title
     * @param defaultValue
     * @param min
     * @param max
     */
    public CommandSelector(String title, Command... commands){
        System.out.println(pos);
        chooser = new SendableChooser<Command>();
        Shuffleboard.getTab("Config")
            .add(title, chooser)
            .withWidget("ComboBox Chooser")
            .withPosition(pos.x, pos.y)
            .withSize(1, 1);
        pos.translate(1, 0);
        if(pos.x >= 7) pos.setLocation(1, pos.y + 2);
        for(int i = 0; i < commands.length; i++){
            System.out.println(i + "/" + commands.length);
            chooser.addOption(commands[i].getName(), commands[i]);
        }
    }

    public Command value(){
        return chooser.getSelected();
    }

    public void add(Command... commands){
        for(int i = 0; i < commands.length; i++){
            chooser.addOption(commands[i].getName(), commands[i]);
        }
    }
}