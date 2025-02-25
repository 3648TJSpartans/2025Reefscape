package frc.robot.util.objectiveTracking;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;

public class ObjectiveTracker {
    public enum Reefpoint {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L
    }

    private Pose2d[] rightReefPoints = PoseConstants.rightReefPoints;
    private Pose2d[] leftReefPoints = PoseConstants.leftReefPoints;
    private Pose2d Defualt_A = rightReefPoints[0];
    private Pose2d Defualt_B = rightReefPoints[0];

    public ObjectiveTracker() {

    }

}
