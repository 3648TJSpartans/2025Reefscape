package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.drive.Drive;

public class NeuralVision {
    private final Drive drive;
    private final NetworkTable table;
    private double tx = 0.0;
    private double ty = 0.0;

    public NeuralVision(Drive drive) {
        this.drive = drive;
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }
    
/* Functions */

    public void refreshValues() { // Grabs the new values from the limelight
        tx = table.getEntry("tx").getDouble(0.0); // Get tx, default to 0.0 if not present
        ty = table.getEntry("ty").getDouble(0.0); // Get ty, default to 0.0 if not present
    }

    public double getTx() { // Returns tx. Intended for use after adjustValues()
        return tx;
    }

    public double getTy() { // Returns ty. Intended for use after adjustValues()
        return ty;
    }

    public int getCurrentPipeline() { // Gets the current pipeline in use by the limelight
        return (int) table.getEntry("pipeline").getDouble(0);
    }

    public void switchPipeline(int pipeline) { // Switches the pipeline in use by the limelight to the specified pipeline
        table.getEntry("pipeline").setNumber(pipeline);
    }
/* Probably unneeeded, there goes 2 days of work

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
*/

    double getTargetRotation() {
        refreshValues();
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetAngle = getTx() * kP;

        // invert since tx is positive when the target is to the right of the crosshair
        targetAngle *= -1.0;

        return targetAngle;
    }

    double getTargetDistance() {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0); 

    double angleToGoalDegrees = VisionConstants.limelightMountAngleDeg + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceToGoalInches = (VisionConstants.targetHeightIn - VisionConstants.limelightMountHeightIn) / Math.tan(angleToGoalRadians);
    return distanceToGoalInches;
    }

    public Pose2d getTargetPose() {
        double distance = getTargetDistance();
        double angle = getTargetRotation();
        return new Pose2d(distance, 0, new Rotation2d(angle));
    }
}
