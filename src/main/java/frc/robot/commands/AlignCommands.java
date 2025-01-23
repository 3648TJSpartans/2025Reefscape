package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class AlignCommands {
  public AlignCommands() {}

  public static Command goToPoint(Drive drive) {
    try {
      List<Waypoint> waypoints =
          PathPlannerPath.waypointsFromPoses(
              drive.getPose(), new Pose2d(7, .8, Rotation2d.fromDegrees(90)));
      PathConstraints pathConstraints =
          new PathConstraints(0.1, 3, 2 * Math.PI / 2, 4 * Math.PI); // TODO update
      // these
      // constants
      PathPlannerPath path =
          new PathPlannerPath(
              waypoints, pathConstraints, null, new GoalEndState(1, Rotation2d.fromDegrees(0)));
      path.preventFlipping = true;
      return AutoBuilder.pathfindThenFollowPath(path, pathConstraints);
    } catch (Exception e) {
      return null;
    }
  }

  // public static Command goTo(Drive drive) {
  // PathConstraints pathConstraints = new PathConstraints(3, 3, 2 * Math.PI, 4 *
  // Math.PI);
  // PathPlannerPath path = new PathPlannerPath(
  // AutoBuilder.pathfindToPose(new Pose2d(11.5, 7.2, Rotation2d.fromDegrees(90)),
  // pathConstraints, 0));
  // }
  public static Command goTo(Drive drive) {
    PathConstraints pathConstraints =
        new PathConstraints(1, 3, 2 * Math.PI, 4 * Math.PI); // TODO increase max
    // velocity later
    System.out.println("Auto Builder Configured: " + AutoBuilder.isConfigured());
    System.out.println("Auto Builder Is Holonomic: " + AutoBuilder.isHolonomic());
    System.out.println(
        "Auto Builder is Pathfinding Configured: " + AutoBuilder.isPathfindingConfigured());
    return AutoBuilder.pathfindToPose(
        new Pose2d(7, .8, Rotation2d.fromDegrees(90)), pathConstraints, 0);
  }
}
