package frc.robot.util.objectiveTracking;

import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.objectiveTracking.ObjectiveTracker.Reefpoint;

public class ObjectiveTrackerObject {
    private final Pose2d pose;
    private boolean filled;
    private final Reefpoint reefpoint;
    private final int level;
    private final Pose3d pose3d;

    public ObjectiveTrackerObject(Pose2d pose, boolean filled, Reefpoint reefpoint, int level) {
        this.pose = pose;
        this.reefpoint = reefpoint;
        this.filled = filled;
        this.level = level;
        this.pose3d = new Pose3d(pose);
        Logger.recordOutput("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Filled", filled);
        Logger.recordOutput("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Pose2d", pose);
        Logger.recordOutput("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Pose3d", pose3d);
    }

    public Pose2d getPose() {
        return pose;
    }

    public Reefpoint getReefpoint() {
        return reefpoint;
    }

    public boolean getFilled() {
        return filled;
    }

    public void setFilled(boolean filled) {
        Logger.recordOutput("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Filled", filled);
        this.filled = filled;
    }

    public ObjectiveTrackerObject asFilled(boolean filled) {
        Logger.recordOutput("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Filled", filled);
        this.filled = filled;
        return this;
    }

    public int getLevel() {
        return level;
    }
}