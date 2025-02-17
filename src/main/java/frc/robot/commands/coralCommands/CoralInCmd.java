package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class CoralInCmd extends Command {
    private final CoralIntake m_coralIntake;

    public CoralInCmd(CoralIntake coralIntake) {
        m_coralIntake = coralIntake;
        addRequirements(m_coralIntake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_coralIntake.setSpeed(CoralIntakeConstants.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_coralIntake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return m_coralIntake.getIR();
    }

}