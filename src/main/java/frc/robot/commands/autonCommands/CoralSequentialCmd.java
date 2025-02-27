package frc.robot.commands.autonCommands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.coralCommands.CoralCmd;
import frc.robot.commands.coralCommands.SlamCoralCmd;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants.AutonState;
import frc.robot.commands.goToCommands.AutonConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.LedConstants;
import frc.robot.subsystems.leds.LedSubsystem;

public class CoralSequentialCmd extends SequentialCommandGroup {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final Drive m_drive;
    private final Command coralCommand;
    private static int level = AutonConstants.defaultLevel; // Defualt Level
    private static AutonState autonState = AutonState.RIGHTREEF;

    public CoralSequentialCmd(Drive drive, CoralIntake coralIntake,
            Elevator elevator, boolean slam) {
        Logger.recordOutput("CoralSequentialCommand/level", level);
        Logger.recordOutput("CoralSequentialCommand/AutonState", autonState.toString());
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_drive = drive;
        coralCommand = AutoBuildingBlocks.coralSmartLevelCommand(elevator, coralIntake, () -> getLevel());
        Command driveCommand = AutoBuildingBlocks.driveToNearest(m_drive, () -> CoralSequentialCmd.poses());
        addCommands(
                new SequentialCommandGroup(
                        // AutoBuildingBlocks.driveToPose(drive, PoseConstants.START),
                        driveCommand,
                        coralCommand,
                        new WaitCommand(1),
                        slam ? new SlamCoralCmd(coralIntake) : null));
        // AutoBuildingBlocks.driveToPose(drive, PoseConstants.START));
    }

    public static void setLevel(int level) {
        CoralSequentialCmd.level = level;
        Logger.recordOutput("CoralSequentialCommand/level", level);
    }

    public static Pose2d[] poses() {
        if (autonState == AutonState.RIGHTREEF) {
            return PoseConstants.rightReefPoints;
        } else if (autonState == AutonState.LEFTREEF) {

            return PoseConstants.leftReefPoints;
        } else {
            System.out.println("return Null");
            System.out.println("Auton State: " + autonState.toString());
            return null;

        }
    }

    public static void setAutonState(AutonState state) {
        CoralSequentialCmd.autonState = state;
        Logger.recordOutput("CoralSequentialCommand/AutonState", state.toString());
    }

    public static int getLevel() {
        return level;
    }

}