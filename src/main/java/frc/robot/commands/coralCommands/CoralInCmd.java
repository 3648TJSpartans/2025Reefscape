package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralSubsystems.CoralConstants;
import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntake;

public class CoralInCmd extends Command {
    private final CoralIntake m_coralIntake;

    public CoralInCmd(CoralIntake coralIntake) {
        m_coralIntake = coralIntake;
        addRequirements(m_coralIntake);
    }

    @Override
    public void initialize() {
        m_coralIntake.setSpeed(CoralConstants.intakeSpeed);
    }

    @Override
    public void execute() {

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