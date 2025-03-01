package frc.robot.commands.autonCommands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.coralCommands.CoralCmd;
import frc.robot.commands.coralCommands.CoralOutCmd;
import frc.robot.commands.coralCommands.SlamCoralCmd;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants.AutonState;
import frc.robot.commands.goToCommands.AutonConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.commands.goToCommands.DriveToNearest2;
import frc.robot.commands.goToCommands.DriveToNearestObjective;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.objectiveTracking.ObjectiveTracker;

public class ObjectiveTrackingCmd extends SequentialCommandGroup {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final Drive m_drive;
    private final Command coralCommand;
    private static int level = AutonConstants.defaultLevel; // Defualt Level
    private static boolean exact = false;
    private static AutonState autonState = AutonState.RIGHTREEF;

    public ObjectiveTrackingCmd(Drive drive, CoralIntake coralIntake,
            Elevator elevator, ObjectiveTracker objectiveTracker) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_drive = drive;
        coralCommand = AutoBuildingBlocks.coralSmartLevelCommand(elevator, coralIntake, () -> CoralSequentialCmd.level);
        DriveToNearestObjective driveCommand = new DriveToNearestObjective(drive, objectiveTracker,
                () -> CoralSequentialCmd.level);
        // Command driveExactCommand = AutoBuildingBlocks.driveToNearest(m_drive, () ->
        // CoralSequentialCmd.poses());
        addCommands(
                new ParallelCommandGroup(
                        driveCommand,
                        coralCommand),
                // driveCloseCommand,
                // coralCommand,
                // driveExactCommand,
                // new WaitCommand(1),
                new CoralOutCmd(m_coralIntake),
                new InstantCommand(() -> driveCommand.setTrackerFilled()));
        // AutoBuildingBlocks.driveToPose(drive, PoseConstants.START));
    }
}