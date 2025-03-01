package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class UpFromIntakeCmd extends SequentialCommandGroup {

        public UpFromIntakeCmd(CoralIntake coralIntake,
                        Elevator elevator) {
                Command firstElevator = new ElevatorCmd(elevator, ElevatorConstants.preIntakePose);
                Command firstWrist = new WristCmd(coralIntake, CoralIntakeConstants.preIntakeAngle);
                Command spitCoral = new CoralCmd(coralIntake, CoralIntakeConstants.preIntakeAngle,
                                CoralIntakeConstants.intakeSpeed);
                addCommands(
                                firstElevator.andThen(firstWrist).withTimeout(1),
                                spitCoral.withTimeout(0.2));
        }
}