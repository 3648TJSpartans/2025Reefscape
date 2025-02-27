package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class DownToIntakeCmd extends SequentialCommandGroup {

    public DownToIntakeCmd(CoralIntake coralIntake,
            Elevator elevator) {
        Command firstElevator = new ElevatorCmd(elevator, ElevatorConstants.preIntakePose);
        Command firstWrist = new WristCmd(coralIntake, CoralIntakeConstants.IntakeAngle);
        Command secondElevator = new ElevatorCmd(elevator, ElevatorConstants.intakePose);
        Command coralIn = new CoralSmartInCmd(coralIntake, CoralIntakeConstants.IntakeAngle);
        addCommands(
                firstElevator.withTimeout(1),
                firstWrist.withTimeout(2),
                secondElevator.alongWith(coralIn));
    }
}