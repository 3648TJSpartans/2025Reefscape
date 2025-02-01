package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class AlignCommands {
  public AlignCommands() {}

  public static Command goToPoint(Drive drive) {
    try {
      //This code is written like #@*%)#, and I'm proud, so just ignore it Eric ~ Micah :)
      List<Waypoint> waypoints =
          PathPlannerPath.waypointsFromPoses(
             //drive.getPose(),
             new Pose2d(2.423, 4.42, Rotation2d.fromDegrees(0)),
              new Pose2d(2.3, 4.42, Rotation2d.fromDegrees(0)));
      PathConstraints pathConstraints =
          new PathConstraints(0.1, 3, 2 * Math.PI / 2, 4 * Math.PI); // TODO update
      // these
      // constants
      PathPlannerPath path =
          new PathPlannerPath(
              waypoints, pathConstraints, null, new GoalEndState(0, Rotation2d.fromDegrees(0)));
      path.preventFlipping = true;
      return AutoBuilder.pathfindThenFollowPath(path, pathConstraints);
    } catch (Exception e) {
      System.err.println("Error: "+ e);
      Logger.recordOutput("Error", e.toString());
      return null;
    }
  }
  public static Command goTo(Drive drive) {
    PathConstraints pathConstraints =
        new PathConstraints(.1, 3, 2 * Math.PI / 2, 4 * Math.PI); // TODO increase max
    // velocity later
    return AutoBuilder.pathfindToPose(new Pose2d(2.423, 4.42, Rotation2d.fromDegrees(0)), pathConstraints, 0.01);
  }
  public static Command testMethod(Drive drive){
    PathConstraints pathConstraints =
    new PathConstraints(.1, 3, 2 * Math.PI / 2, 4 * Math.PI); // TODO increase max
    return new InstantCommand(()-> AutoBuilder.pathfindToPose(new Pose2d(2.423, 4.42, Rotation2d.fromDegrees(0)), pathConstraints, 0.01).schedule());
  }
}
