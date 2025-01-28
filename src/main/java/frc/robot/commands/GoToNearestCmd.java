package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class GoToNearestCmd extends Command{
    private final Pose2d[] points = new Pose2d[]{
        new Pose2d(2.423, 3, Rotation2d.fromDegrees(0)),
        new Pose2d(2.423, 5, Rotation2d.fromDegrees(0))
    };
    private PathConstraints pathConstraints;
    private Drive drive;
    public  GoToNearestCmd(Drive drive){
        this.drive = drive;
        addRequirements(drive);
    }
    @Override
    public void initialize(){
        pathConstraints = new PathConstraints(.1, 3, 2 * Math.PI / 2, 4 * Math.PI);
        Pose2d nearestPoint = points[0];
        double minDistance = Double.MAX_VALUE;
        for(Pose2d point : points){
           double distance = drive.getPose().getTranslation().getDistance(point.getTranslation());
            if(distance < minDistance){
                minDistance = distance;
                nearestPoint = point;
            }
        }
        AutoBuilder.pathfindToPose(nearestPoint, pathConstraints, 0.01).schedule();
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
