package frc.robot.util.objectiveTracking;

import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.objectiveTracking.ObjectiveTracker.Reefpoint;

public class ObjectiveTrackerObject {
    private final Pose2d pose;
    private boolean filled;
    private final Reefpoint reefpoint;
    private final int level;

    public ObjectiveTrackerObject(Pose2d pose, boolean filled, Reefpoint reefpoint, int level) {
        this.pose = pose;
        this.reefpoint = reefpoint;
        this.filled = filled;
        this.level = level;
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
        this.filled = filled;
    }

    public int getLevel() {
        return level;
    }
}