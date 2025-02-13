package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.drive.Drive;

public class NeuralVision {
    private final Drive drive;
    private final NetworkTable table;
    private double ty = 0.0;

    public NeuralVision(Drive drive) {
        this.drive = drive;
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }
    
/* Functions */

    public void refreshValues() { // Grabs the new values from the limelight
        ty = table.getEntry("ty").getDouble(0.0); // Get tx, default to 0.0 if not present
    }

    public double getTx() { // Returns tx. Intended for use after adjustValues()
        return ty;
    }

    public int getCurrentPipeline() { // Gets the current pipeline in use by the limelight
        return (int) table.getEntry("pipeline").getDouble(0);
    }

    public void switchPipeline(int pipeline) { // Switches the pipeline in use by the limelight to the specified pipeline
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public void alignToAlgae() { // Loops until the robot is facing the target algae within the sensitivity range, as defined in VisionConstants. 
        refreshValues(); // Get the initial tx
        double tx = getTx(); // Assign the initial tx
        while (tx > VisionConstants.alignSensitivity || tx < -VisionConstants.alignSensitivity) { // While the target is not aligned

            if (tx > VisionConstants.alignSensitivity) {  // If the target is to the right
                final Rotation2d targetRotation = (drive.getRotation()).plus(Rotation2d.fromDegrees(1)); 
                drive.setPose(new Pose2d(drive.getPose().getTranslation(), targetRotation)); // Rotate to the right by 1 degree

            } else if (tx < -VisionConstants.alignSensitivity) { // If the target is to the left
                final Rotation2d targetRotation = (drive.getRotation()).minus(Rotation2d.fromDegrees(1));
                drive.setPose(new Pose2d(drive.getPose().getTranslation(), targetRotation));
            }

            refreshValues(); // Update tx to check if the target is aligned
            tx = getTx();
        }
    }
}
