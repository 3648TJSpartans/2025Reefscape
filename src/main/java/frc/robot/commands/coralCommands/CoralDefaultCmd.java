package frc.robot.commands.coralCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.coralIntake.CoralIntake;

public class CoralDefaultCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;

    public CoralDefaultCmd(CoralIntake coralIntake, Elevator elevator) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        addRequirements(coralIntake, elevator);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Elevator/Command/Scheduled", "CoralDefault");
    }

    @Override
    public void execute() {
        m_coralIntake.rotateTo(CoralIntakeConstants.defaultAngle);
        m_elevator.elevateTo(ElevatorConstants.defaultPosition);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Elevator/Command/Scheduled", "Unscheduled");
    }

}