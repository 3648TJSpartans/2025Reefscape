package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class GoToPointCmd extends Command{
    private final Pose2d point;
    private PathConstraints pathConstraints;
    public GoToPointCmd(Drive drive, Pose2d point){
        this.point = point;
        addRequirements(drive);
    }
    @Override
    public void initialize(){
        pathConstraints = new PathConstraints(.1, 3, 2 * Math.PI / 2, 4 * Math.PI);
        AutoBuilder.pathfindToPose(point, pathConstraints, 0.01).schedule();
    }
    @Override
    public void execute(){
        
    }
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
