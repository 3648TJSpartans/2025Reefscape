package frc.robot.commands.autonCommands;

import java.lang.Thread.State;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.coralCommands.CoralCmd;
import frc.robot.commands.coralCommands.CoralOutCmd;
import frc.robot.commands.coralCommands.CoralSmartLevelWristCmd;
import frc.robot.commands.coralCommands.SlamCoralCmd;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants.AutonState;
import frc.robot.commands.goToCommands.AutonConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.commands.goToCommands.DriveToNearest2;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.LedSubsystem;

public class CoralSequentialCmd extends SequentialCommandGroup {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final Drive m_drive;
    private final Command coralCommand;
    private final Command coral2Command;
    private static int level = AutonConstants.defaultLevel; // Defualt Level
    private static boolean exact = false;
    private static AutonState autonState = AutonState.RIGHTREEF;

    public CoralSequentialCmd(Drive drive, CoralIntake coralIntake,
            Elevator elevator, boolean slam) {
        Logger.recordOutput("CoralSequentialCommand/level", level);
        Logger.recordOutput("CoralSequentialCommand/AutonState", autonState.toString());
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_drive = drive;
        coralCommand = AutoBuildingBlocks.coralSmartLevelCommand(elevator, coralIntake, () -> getLevel(), true);
        coral2Command = AutoBuildingBlocks.coralSmartLevelCommand(elevator, coralIntake, () -> getLevel(), false);
        Command coral3Command = AutoBuildingBlocks.coralSmartLevelCommand(elevator, coralIntake, () -> getLevel(),
                false);
        Command outtake = new CoralSmartLevelWristCmd(coralIntake, elevator, () -> getLevel(),
                CoralIntakeConstants.outtakeSpeed);
        DriveToNearest driveCommand = new DriveToNearest(m_drive, () -> CoralSequentialCmd.poses(false));
        DriveToNearest2 drive2Command = new DriveToNearest2(m_drive, () -> CoralSequentialCmd.poses(true));
        // Command driveExactCommand = AutoBuildingBlocks.driveToNearest(m_drive, () ->
        // CoralSequentialCmd.poses());
        addCommands(
                new SequentialCommandGroup(

                        new ParallelDeadlineGroup(
                                driveCommand,
                                coralCommand.repeatedly()),

                        new ParallelDeadlineGroup(
                                drive2Command,
                                coral2Command).withTimeout(.75),
                        coral3Command.withTimeout(0.5),
                        slam ? outtake : null));
        ;
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

    public static int getLeftOrRIghtState() {
        if (CoralSequentialCmd.autonState == autonState.RIGHTREEF) {
            return 1;
        } else {
            return 0;
        }

    }

    public static int getLevel() {
        return level;
    }

}