package frc.robot.commands.autonCommands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.coralCommands.CoralCmd;
import frc.robot.commands.coralCommands.CoralElevatorIntegratedCmd;
import frc.robot.commands.coralCommands.CoralOutCmd;
import frc.robot.commands.coralCommands.ElevatorCmd;
import frc.robot.commands.coralCommands.SlamCoralCmd;
import frc.robot.commands.coralCommands.WristAnalogCmd;
import frc.robot.commands.coralCommands.WristCmd;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants.AutonState;
import frc.robot.commands.goToCommands.AutonConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.commands.goToCommands.DriveToNearest5;
import frc.robot.commands.goToCommands.DriveToNearest3;
import frc.robot.commands.goToCommands.DriveToNearest4;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class AlgaeRemovalCmd extends SequentialCommandGroup {
    private static int level = AutonConstants.defaultLevel; // Defualt Level

    public AlgaeRemovalCmd(Drive drive, CoralIntake coralIntake,
            Elevator elevator, BooleanSupplier high) {
        DriveToNearest3 driveFirstCommand = new DriveToNearest3(drive, () -> poses(1));
        DriveToNearest4 driveSecondCommand = new DriveToNearest4(drive, () -> poses(2));
        DriveToNearest5 driveThirdCommand = new DriveToNearest5(drive, () -> poses(3));
        Command highCommand = new ElevatorCmd(elevator, ElevatorConstants.algaeRemovalHeightHigh - 10);
        Command lowCommand = new ElevatorCmd(elevator, ElevatorConstants.algaeRemovalHeightLow);
        Command wristCommand = new WristAnalogCmd(coralIntake, () -> -.5);// out of 10
        Command coralInitialCommand = new CoralElevatorIntegratedCmd(coralIntake, elevator,
                ElevatorConstants.coralLeveL4, CoralIntakeConstants.L4Angle);
        Command coralCommand = new ConditionalCommand(highCommand, lowCommand, high);
        Command holdCoral = new CoralCmd(coralIntake, CoralIntakeConstants.algaeRemovalAngle, 0);
        // Command driveExactCommand = AutoBuildingBlocks.driveToNearest(m_drive, () ->
        // CoralSequentialCmd.poses());
        addCommands(
                driveFirstCommand.alongWith(coralInitialCommand),
                driveSecondCommand,
                coralCommand.alongWith(wristCommand).withTimeout(1.5),
                new ParallelCommandGroup(holdCoral, driveThirdCommand));
    }
    // AutoBuildingBlocks.driveToPose(drive, PoseConstants.START));

    public static Pose2d[] poses(int closeLevel) {
        switch (closeLevel) {
            case 1:
                return PoseConstants.farReefPoints;
            case 2:
                return PoseConstants.closeReefPoints;
            case 3:
                return PoseConstants.farrerReefPoints;
            default:
                System.out.println("F-ed up");
                return PoseConstants.farReefPoints;

        }
    }

    public int getLevel() {
        return level;
    }

}