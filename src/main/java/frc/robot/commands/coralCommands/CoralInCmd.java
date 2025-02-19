package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class CoralInCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final Timer m_timer;
    private Boolean isFinished = false;

    public CoralInCmd(CoralIntake coralIntake) {
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