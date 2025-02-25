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
    private Pose2d Defualt_B = leftReefPoints[0];
    private Pose2d Defualt_C = rightReefPoints[1];
    private Pose2d Defualt_D = leftReefPoints[1];
    private Pose2d Defualt_E = rightReefPoints[2];
    private Pose2d Defualt_F = leftReefPoints[2];
    private Pose2d Defualt_G = rightReefPoints[3];
    private Pose2d Defualt_H = leftReefPoints[3];
    private Pose2d Defualt_I = rightReefPoints[4];
    private Pose2d Defualt_J = leftReefPoints[4];
    private Pose2d Defualt_K = rightReefPoints[5];
    private Pose2d Defualt_L = leftReefPoints[5];

    private ObjectiveTrackerObject[] defualtObjectives;

    public ObjectiveTracker() {
        defualtObjectives = new ObjectiveTrackerObject[] {
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4)
        };
    }

}
