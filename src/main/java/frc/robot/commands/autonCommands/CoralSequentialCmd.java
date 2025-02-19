package frc.robot.commands.autonCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.coralCommands.CoralCmd;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants.AutonState;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class CoralSequentialCmd extends SequentialCommandGroup {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final Drive m_drive;
    private final Command coralCommand;
    private static int level;
    private static AutonState autonState;

    public CoralSequentialCmd(Drive drive, CoralIntake coralIntake,
            Elevator elevator, boolean slam) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_drive = drive;
        switch (level) {
            case 1:
                coralCommand = AutoBuildingBlocks.l1(coralIntake, elevator);
                break;
            case 2:
                coralCommand = AutoBuildingBlocks.l2(coralIntake, elevator);
                break;
            case 3:
                coralCommand = AutoBuildingBlocks.l3(coralIntake, elevator);
                break;
            case 4:
                coralCommand = AutoBuildingBlocks.l4(coralIntake, elevator);
                break;
            default:
                coralCommand = null;
        }
        Command driveCommand = AutoBuildingBlocks.driveToNearest(m_drive, () -> CoralSequentialCmd.poses());
        addCommands(
                new SequentialCommandGroup(
                        driveCommand,
                        coralCommand,
                        new WaitCommand(1),
                        slam ? new CoralCmd(m_coralIntake, .05, -.2) : null));
    }

    public static void setLevel(int level) {
        CoralSequentialCmd.level = level;
        Logger.recordOutput("CoralSequentialCommand/level", level);
    }

    public static Pose2d[] poses() {
        if (autonState == AutonState.RIGHTREEF) {
            return PoseConstants.rightReefPoints();
        } else if (autonState == AutonState.LEFTREEF) {
            return PoseConstants.leftReefPoints();
        } else {
            return null;
        }
    }

    public static void setAutonState(AutonState state) {
        CoralSequentialCmd.autonState = state;
        Logger.recordOutput("CoralSequentialCommand/AutonState", state);
    }
}