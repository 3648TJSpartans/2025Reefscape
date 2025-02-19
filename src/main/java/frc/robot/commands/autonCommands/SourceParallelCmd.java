package frc.robot.commands.autonCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coralCommands.CoralCmd;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class SourceParallelCmd extends ParallelCommandGroup {
        private final CoralIntake m_coralIntake;
        private final Elevator m_elevator;
        private final Drive m_drive;
        private final Command sourceCommand;

        public SourceParallelCmd(Drive drive, CoralIntake coralIntake,
                        Elevator elevator) {
                m_coralIntake = coralIntake;
                m_elevator = elevator;
                m_drive = drive;
                sourceCommand = AutoBuildingBlocks.driveToPose(m_drive,
                                new Pose2d(1.5, 1.6, new Rotation2d(Math.PI / 3)));

                addRequirements(m_coralIntake);
                addRequirements(m_elevator);

                addCommands(
                                AutoBuildingBlocks.intakeSource(elevator, coralIntake),
                                sourceCommand);
        }

}