package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;

public class CoralInCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final Timer m_timer;
    private Boolean isFinished = false;

    public CoralInCmd(CoralIntake coralIntake, Elevator elevator) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        addRequirements(m_coralIntake, m_elevator);
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_timer.stop();
        m_timer.reset();
    }

    @Override
    public void execute() {
        boolean coralInSFT = !m_coralIntake.getIR();
        boolean coralInArm = !m_coralIntake.getIR();
        if (coralInSFT) {
            m_coralIntake.setSpeed(CoralIntakeConstants.intakeSpeed);
            m_elevator.set
            if (!coralInArm) {
                m_timer.reset();

            }
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