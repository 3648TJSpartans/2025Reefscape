package frc.robot.util.objectiveTracking;

import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.TunableBoolean;
import frc.robot.util.TunableNumber;
import frc.robot.util.objectiveTracking.ObjectiveTracker.Reefpoint;

public class ObjectiveTrackerObject {
    private final Pose2d pose;
    private boolean filled;
    private final Reefpoint reefpoint;
    private final int level;
    private final Pose3d pose3d;
    private final NetworkTable table;
    BooleanSubscriber filledSub;

    public ObjectiveTrackerObject(Pose2d pose, boolean filled, Reefpoint reefpoint, int level, NetworkTable table) {
        this.pose = pose;
        this.reefpoint = reefpoint;
        this.filled = filled;
        this.level = level;
        this.pose3d = new Pose3d(pose.getX(), pose.getY(), getHeight(level), new Rotation3d(pose.getRotation()));
        this.table = table;
        // SmartDashboard.putBoolean("ObjectiveTracker/Objectives/" +
        // reefpoint.toString() + "/" + level + "/Filled",
        // filled);
        table.getBooleanTopic("testEntry").publish();
        filledSub = table.getBooleanTopic("testEntry").subscribe(filled);
        Logger.recordOutput("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Filled", filled);
        Logger.recordOutput("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Pose2d", pose);
        Logger.recordOutput("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Pose3d", pose3d);
    }

    public void updateValues() {
        // filled = !filled;

        filled = filledSub.get();
        table.getEntry("testEntry").setBoolean(filled);
        // boolean foo = new TunableNumber("foo", 0.0) == 0.0;
        Logger.recordOutput("TestEntry", filled);
        System.out.println("TestEntry " + filled);
    }

    public Pose2d getPose() {
        return pose;
    }

    public Pose3d getPose3d() {
        return pose3d;
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
        SmartDashboard.putBoolean("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Filled",
                filled);
    }

    public ObjectiveTrackerObject asFilled(boolean filled) {
        Logger.recordOutput("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Filled", filled);
        this.filled = filled;
        SmartDashboard.putBoolean("ObjectiveTracker/Objectives/" + reefpoint.toString() + "/" + level + "/Filled",
                filled);
        return this;
    }

    public int getLevel() {
        return level;
    }

    public double getHeight(int level) {
        switch (level) {
            case 1:
                return .46;
            case 2:
                return .81;
            case 3:
                return 1.21;
            case 4:
                return 1.83;
            default:
                return 0;
        }
    }
}