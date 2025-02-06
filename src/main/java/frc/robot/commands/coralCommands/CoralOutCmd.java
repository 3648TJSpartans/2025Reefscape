package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralSubsystems.CoralConstants;
import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntake;
import edu.wpi.first.wpilibj.Timer;

public class CoralOutCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final Timer m_timer;
    private Boolean isFinished = false;

    public CoralOutCmd(CoralIntake coralIntake) {
        m_coralIntake = coralIntake;
        addRequirements(m_coralIntake);
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_timer.stop();
        m_timer.reset();
    }

    @Override
    public void execute() {
        boolean objectDetected = !m_coralIntake.getIR();
        if (objectDetected) {
            m_coralIntake.setSpeed(-CoralConstants.intakeSpeed);
            m_timer.reset();
        } else {
            if (!m_timer.hasElapsed(0.02)) {
                if (!m_timer.isRunning()) {
                    m_timer.start();
                }
                m_coralIntake.setSpeed(-CoralConstants.intakeSpeed);
            } else {
                isFinished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_coralIntake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}