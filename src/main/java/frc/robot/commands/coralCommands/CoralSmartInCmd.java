package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntake;

public class CoralSmartInCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final double angle;
    private final Timer m_timer;
    private Boolean isFinished = false;

    public CoralSmartInCmd(CoralIntake coralIntake, double angle) {
        m_coralIntake = coralIntake;
        this.angle = angle;
        m_timer = new Timer();
        addRequirements(m_coralIntake);
    }

    @Override
    public void initialize() {
        m_timer.stop();
        m_timer.reset();
    }

    @Override
    public void execute() {
        boolean objectDetected = !m_coralIntake.getIR();
        if (!objectDetected) {
            m_coralIntake.setSpeed(CoralIntakeConstants.intakeSpeed);
            m_timer.reset();
        } else {
            if (!m_timer.hasElapsed(0.1)) {
                if (!m_timer.isRunning()) {
                    m_timer.start();
                }
                m_coralIntake.setSpeed(CoralIntakeConstants.intakeSpeed);
            } else {
                isFinished = true;
            }
        }
        m_coralIntake.rotateTo(angle);
    }

    @Override
    public void end(boolean interrupted) {
        m_coralIntake.setSpeed(0);
        m_coralIntake.stopWristMotor();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}