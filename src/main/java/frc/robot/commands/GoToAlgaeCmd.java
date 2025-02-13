package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.NeuralVision;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionConstants;

public class GoToAlgaeCmd extends Command {
    private final NeuralVision nvision;
    private final Drive drive;
    // private final VisionConstants visionConstants;
    private int oldPipeline; 

    public GoToAlgaeCmd(NeuralVision nvision, Drive drive) { // Not static imports
        this.nvision = nvision;
        this.drive = drive;
        // addRequirements(vision);
    }

    @Override
    public void initialize() {
        oldPipeline = nvision.getCurrentPipeline();
        nvision.switchPipeline(1); // Switches to pipeline 1 (Neural network)
    }

    @Override
    public void execute() {
        nvision.adjustValues();
        double tx = nvision.getTx(); // Gets tx (horizontal offset from target)
        if (tx > (VisionConstants.alignSensitivity)) { // If tx is greater than the align sensitivity, turn right until it isn't
            final Rotation2d targetRotation = (drive.getRotation()).plus(Rotation2d.fromDegrees(1)); // Gets current rotation and sets target rotation to 1 degree more
            drive.setPose(new Pose2d(drive.getPose().getTranslation(), targetRotation)); // Sets the new rotation

        } else if (tx < -(VisionConstants.alignSensitivity)) { // If tx is less than the align sensitivity turn left until it isn't
            final Rotation2d targetRotation = (drive.getRotation()).minus(Rotation2d.fromDegrees(1)); // Gets current rotation and sets target rotation to 1 degree less
            drive.setPose(new Pose2d(drive.getPose().getTranslation(), targetRotation)); // Sets the new rotation
            
        }
    }

    @Override
    public void end(boolean interrupted) {
        nvision.switchPipeline(oldPipeline); // Switches back to pipeline 0 (Driver camera)
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}