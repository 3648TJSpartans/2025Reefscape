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
import frc.robot.commands.goToCommands.DriveToNearest5;
import frc.robot.commands.goToCommands.DriveToNearest3;
import frc.robot.commands.goToCommands.DriveToNearest4;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class AlgaeRemovalCmd extends SequentialCommandGroup {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final Drive m_drive;
    private final Command coralCommand;
    private static int level = AutonConstants.defaultLevel; // Defualt Level
    private static boolean exact = false;
    private static AutonState autonState = AutonState.RIGHTREEF;

    public AlgaeRemovalCmd(Drive drive, CoralIntake coralIntake,
            Elevator elevator, Supplier<Integer> level) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_drive = drive;
        coralCommand = AutoBuildingBlocks.coralSmartLevelCommand(elevator, coralIntake, () -> level.get());
        DriveToNearest3 drive3Command = new DriveToNearest3(m_drive, () -> CoralSequentialCmd.poses(false));
        DriveToNearest4 drive4Command = new DriveToNearest4(m_drive, () -> CoralSequentialCmd.poses(true));
        DriveToNearest5 drive5Command = new DriveToNearest5(m_drive, () -> CoralSequentialCmd.poses(true));
        // Command driveExactCommand = AutoBuildingBlocks.driveToNearest(m_drive, () ->
        // CoralSequentialCmd.poses());
        addCommands(
                new SequentialCommandGroup(
                        // AutoBuildingBlocks.driveToPose(drive, PoseConstants.START),

                        new ParallelCommandGroup(
                                driveCommand,
                                coralCommand),
                        drive2Command,
                        // driveCloseCommand,
                        // coralCommand,
                        // driveExactCommand,
                        // new WaitCommand(1),
                        slam ? new CoralOutCmd(m_coralIntake) : null));
        // AutoBuildingBlocks.driveToPose(drive, PoseConstants.START));
    }

    public static void setLevel(int level) {
        CoralSequentialCmd.level = level;
        Logger.recordOutput("CoralSequentialCommand/level", level);
    }

    public static void setExact(boolean exact) {
        CoralSequentialCmd.exact = exact;
        Logger.recordOutput("CoralSequentialCommand/exact", exact);
    }

    public static Pose2d[] poses(boolean exact) {
        if (exact) {
            if (level == 3 || level == 4) {
                if (autonState == AutonState.RIGHTREEF) {
                    return PoseConstants.l4ExactRightReefPoints;
                } else if (autonState == AutonState.LEFTREEF) {
                    return PoseConstants.l4ExactLeftReefPoints;
                } else {
                    System.out.println("return Null");
                    System.out.println("Auton State: " + autonState.toString());
                    return null;

                }
            } else if (level == 2) {
                if (autonState == AutonState.RIGHTREEF) {
                    return PoseConstants.l2ExactRightReefPoints;
                } else if (autonState == AutonState.LEFTREEF) {

                    return PoseConstants.l2ExactLeftReefPoints;
                } else {
                    System.out.println("return Null");
                    System.out.println("Auton State: " + autonState.toString());
                    return null;

                }
            } else {
                if (autonState == AutonState.RIGHTREEF) {
                    return PoseConstants.l1ExactRightReefPoints;
                } else if (autonState == AutonState.LEFTREEF) {
                    return PoseConstants.l1ExactLeftReefPoints;
                } else {
                    System.out.println("return Null");
                    System.out.println("Auton State: " + autonState.toString());
                    return null;

                }
            }
        } else {
            if (level == 3 || level == 4) {
                if (autonState == AutonState.RIGHTREEF) {
                    return PoseConstants.l4CloseRightReefPoints;
                } else if (autonState == AutonState.LEFTREEF) {

                    return PoseConstants.l4CloseLeftReefPoints;
                } else {
                    System.out.println("return Null");
                    System.out.println("Auton State: " + autonState.toString());
                    return null;

                }
            } else if (level == 2) {
                if (autonState == AutonState.RIGHTREEF) {
                    return PoseConstants.l2CloseRightReefPoints;
                } else if (autonState == AutonState.LEFTREEF) {

                    return PoseConstants.l2CloseLeftReefPoints;
                } else {
                    System.out.println("return Null");
                    System.out.println("Auton State: " + autonState.toString());
                    return null;

                }
            } else {
                if (autonState == AutonState.RIGHTREEF) {
                    return PoseConstants.l1CloseRightReefPoints;
                } else if (autonState == AutonState.LEFTREEF) {
                    return PoseConstants.l1CloseLeftReefPoints;
                } else {
                    System.out.println("return Null");
                    System.out.println("Auton State: " + autonState.toString());
                    return null;

                }
            }
        }
    }

    public static void setAutonState(AutonState state) {
        CoralSequentialCmd.autonState = state;
        Logger.recordOutput("CoralSequentialCommand/AutonState", state.toString());
    }

    public int getLevel() {
        return level;
    }

}