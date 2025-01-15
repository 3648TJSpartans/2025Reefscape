package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import com.pathplanner.lib.path.PathPlannerPath;
import java.util.List;
import com.pathplanner.lib.path.Waypoint;

public class FollowTheLeader extends Command {
    private Drive drive;
    private PathPlannerPath path;

    public FollowTheLeader(Drive drive) {
        this.drive = drive;
        List<Waypoint> = PathPlannerPath.waypointsFromPoses(
            new Pose2d(2.5, 4.015, 0);
        );
        path = new PathPlannerPath();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // m_motorMMove.setMotorSpeed(MotorConstants.speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }
}
