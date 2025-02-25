package frc.robot.util.objectiveTracking;

import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    private ObjectiveTrackerObject[] objectives;

    public ObjectiveTracker() {
        objectives = new ObjectiveTrackerObject[] {
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 1),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 2),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 3),
                new ObjectiveTrackerObject(Defualt_A, false, Reefpoint.A, 4),

                new ObjectiveTrackerObject(Defualt_B, false, Reefpoint.B, 1),
                new ObjectiveTrackerObject(Defualt_B, false, Reefpoint.B, 2),
                new ObjectiveTrackerObject(Defualt_B, false, Reefpoint.B, 3),
                new ObjectiveTrackerObject(Defualt_B, false, Reefpoint.B, 4),

                new ObjectiveTrackerObject(Defualt_C, false, Reefpoint.C, 1),
                new ObjectiveTrackerObject(Defualt_C, false, Reefpoint.C, 2),
                new ObjectiveTrackerObject(Defualt_C, false, Reefpoint.C, 3),
                new ObjectiveTrackerObject(Defualt_C, false, Reefpoint.C, 4),

                new ObjectiveTrackerObject(Defualt_D, false, Reefpoint.D, 1),
                new ObjectiveTrackerObject(Defualt_D, false, Reefpoint.D, 2),
                new ObjectiveTrackerObject(Defualt_D, false, Reefpoint.D, 3),
                new ObjectiveTrackerObject(Defualt_D, false, Reefpoint.D, 4),

                new ObjectiveTrackerObject(Defualt_E, false, Reefpoint.E, 1),
                new ObjectiveTrackerObject(Defualt_E, false, Reefpoint.E, 2),
                new ObjectiveTrackerObject(Defualt_E, false, Reefpoint.E, 3),
                new ObjectiveTrackerObject(Defualt_E, false, Reefpoint.E, 4),

                new ObjectiveTrackerObject(Defualt_F, false, Reefpoint.F, 1),
                new ObjectiveTrackerObject(Defualt_F, false, Reefpoint.F, 2),
                new ObjectiveTrackerObject(Defualt_F, false, Reefpoint.F, 3),
                new ObjectiveTrackerObject(Defualt_F, false, Reefpoint.F, 4),

                new ObjectiveTrackerObject(Defualt_G, false, Reefpoint.G, 1),
                new ObjectiveTrackerObject(Defualt_G, false, Reefpoint.G, 2),
                new ObjectiveTrackerObject(Defualt_G, false, Reefpoint.G, 3),
                new ObjectiveTrackerObject(Defualt_G, false, Reefpoint.G, 4),

                new ObjectiveTrackerObject(Defualt_H, false, Reefpoint.H, 1),
                new ObjectiveTrackerObject(Defualt_H, false, Reefpoint.H, 2),
                new ObjectiveTrackerObject(Defualt_H, false, Reefpoint.H, 3),
                new ObjectiveTrackerObject(Defualt_H, false, Reefpoint.H, 4),

                new ObjectiveTrackerObject(Defualt_I, false, Reefpoint.I, 1),
                new ObjectiveTrackerObject(Defualt_I, false, Reefpoint.I, 2),
                new ObjectiveTrackerObject(Defualt_I, false, Reefpoint.I, 3),
                new ObjectiveTrackerObject(Defualt_I, false, Reefpoint.I, 4),

                new ObjectiveTrackerObject(Defualt_J, false, Reefpoint.J, 1),
                new ObjectiveTrackerObject(Defualt_J, false, Reefpoint.J, 2),
                new ObjectiveTrackerObject(Defualt_J, false, Reefpoint.J, 3),
                new ObjectiveTrackerObject(Defualt_J, false, Reefpoint.J, 4),

                new ObjectiveTrackerObject(Defualt_K, false, Reefpoint.K, 1),
                new ObjectiveTrackerObject(Defualt_K, false, Reefpoint.K, 2),
                new ObjectiveTrackerObject(Defualt_K, false, Reefpoint.K, 3),
                new ObjectiveTrackerObject(Defualt_K, false, Reefpoint.K, 4),

                new ObjectiveTrackerObject(Defualt_L, false, Reefpoint.L, 1),
                new ObjectiveTrackerObject(Defualt_L, false, Reefpoint.L, 2),
                new ObjectiveTrackerObject(Defualt_L, false, Reefpoint.L, 3),
                new ObjectiveTrackerObject(Defualt_L, false, Reefpoint.L, 4)
        };
        updateLoggedPoses();
    }

    public ObjectiveTrackerObject[] getObjectives() {
        ArrayList<ObjectiveTrackerObject> out = new ArrayList<ObjectiveTrackerObject>();

        for (ObjectiveTrackerObject objective : objectives) {
            if (!objective.getFilled()) {
                out.add(objective);
            }
        }
        ObjectiveTrackerObject[] outArr = new ObjectiveTrackerObject[out.size()];
        return out.toArray(outArr);
    }

    public ObjectiveTrackerObject[] getObjectivesFilled() {
        ArrayList<ObjectiveTrackerObject> out = new ArrayList<ObjectiveTrackerObject>();

        for (ObjectiveTrackerObject objective : objectives) {
            if (objective.getFilled()) {
                out.add(objective);
            }
        }
        ObjectiveTrackerObject[] outArr = new ObjectiveTrackerObject[out.size()];
        return out.toArray(outArr);
    }

    public ObjectiveTrackerObject[] getObjectives(Supplier<Integer> level) {
        int levelNum = level.get();
        ArrayList<ObjectiveTrackerObject> out = new ArrayList<ObjectiveTrackerObject>();
        for (ObjectiveTrackerObject objective : objectives) {
            if (objective.getLevel() == levelNum && !objective.getFilled()) {
                out.add(objective);
            }
        }
        ObjectiveTrackerObject[] outArr = new ObjectiveTrackerObject[out.size()];
        return out.toArray(outArr);
    }

    public ObjectiveTrackerObject[] getObjectives(Reefpoint reefpoint) {
        ArrayList<ObjectiveTrackerObject> out = new ArrayList<ObjectiveTrackerObject>();
        for (ObjectiveTrackerObject objective : objectives) {
            if (objective.getReefpoint() == reefpoint && !objective.getFilled()) {
                out.add(objective);
            }
        }
        ObjectiveTrackerObject[] outArr = new ObjectiveTrackerObject[out.size()];
        return out.toArray(outArr);
    }

    @AutoLogOutput(key = "ObjectiveTracker/Poses")
    public Pose2d[] getObjectivePoses() {
        ObjectiveTrackerObject[] objectivePoses = getObjectives();
        Pose2d[] out = new Pose2d[objectivePoses.length];
        for (int i = 0; i < objectivePoses.length; i++) {
            out[i] = objectivePoses[i].getPose();
        }
        return out;
    }

    @AutoLogOutput(key = "ObjectiveTracker/Poses")
    public Pose2d[] getObjectivePoses(Supplier<Integer> level) {
        ObjectiveTrackerObject[] objectivePoses = getObjectives(level);
        Pose2d[] out = new Pose2d[objectivePoses.length];
        for (int i = 0; i < objectivePoses.length; i++) {
            out[i] = objectivePoses[i].getPose();
        }
        return out;
    }

    @AutoLogOutput(key = "ObjectiveTracker/Poses")
    public Pose2d[] getObjectivePoses(Reefpoint reefpoint) {
        ObjectiveTrackerObject[] objectivePoses = getObjectives(reefpoint);
        Pose2d[] out = new Pose2d[objectivePoses.length];
        for (int i = 0; i < objectivePoses.length; i++) {
            out[i] = objectivePoses[i].getPose();
        }
        return out;
    }

    public void updateLoggedPoses() {
        ObjectiveTrackerObject[] objectivePoses = getObjectives();
        Pose3d[] out = new Pose3d[objectivePoses.length];
        for (int i = 0; i < objectivePoses.length; i++) {
            out[i] = objectivePoses[i].getPose3d();
        }
        Logger.recordOutput("ObjectiveTracker/UnfilledPoses", out);
        objectivePoses = getObjectivesFilled();
        out = new Pose3d[objectivePoses.length];
        for (int i = 0; i < objectivePoses.length; i++) {
            out[i] = objectivePoses[i].getPose3d();
        }
        Logger.recordOutput("ObjectiveTracker/FilledPoses", out);
    }

    public void setObjectiveValue(ObjectiveTrackerObject objectiveTrackerObject) {
        for (ObjectiveTrackerObject value : objectives) {
            if (value.getReefpoint() == objectiveTrackerObject.getReefpoint()
                    && value.getLevel() == objectiveTrackerObject.getLevel()) {
                value = objectiveTrackerObject;
            }
        }
        updateLoggedPoses();
    }
}
