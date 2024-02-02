package frc.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.LimelightHelpers;

public class Limelight extends SubsystemBase{
    private String name;
    private Transform3d cameraOffset;
    public Limelight(String n, Transform3d offset) {
        name = n;
        cameraOffset = offset;
    }
    public Pose3d getCameraPoseTargetSpace() {
        return LimelightHelpers.toPose3D(NetworkTableInstance.getDefault().getTable(name).getEntry("camerapose_targetspace").getDoubleArray(new double[6]));
    }
}
