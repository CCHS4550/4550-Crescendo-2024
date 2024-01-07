package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;

public class Limelight {
    private String name;
    private Transform3d cameraOffset;

    public Limelight (String name, Transform3d cameraOffset){
        this.name = name;
        this.cameraOffset = cameraOffset;
    }

    // public void update

    public String getName() {
        return name;
    }
}