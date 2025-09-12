package frc.robot.commands.coralCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntake;

public class CoralSmartInCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final double angle;

    private Timer m_timer;

    private Boolean isFinished = false;

    public CoralSmartInCmd(CoralIntake coralIntake, double angle) {
        m_coralIntake = coralIntake;
        this.angle = angle;
        m_timer = new Timer();
        addRequirements(coralIntake);

        Logger.recordOutput("Intake/smartInRunning", false);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Elevator/Command/Scheduled", "CoralSmartIn");
        isFinished = false;
        m_timer = new Timer();
        m_timer.start();

    }

    @Override
    public void execute() {
        // Logger.recordOutput("Intake/smartInRunning", true);
        boolean objectDetected = !m_coralIntake.getIR();
        // if (!objectDetected) {

        // m_timer.reset();
        // m_timer.start();
        // }
        // isFinished = objectDetected && m_timer.get() > 0.4;

        // Logger.recordOutput("Timer/time", m_timer.get());
        m_coralIntake.rotateTo(angle);
        m_coralIntake.setSpeed(CoralIntakeConstants.intakeSpeed);
        isFinished = objectDetected;

    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Elevator/Command/Scheduled", "Unscheduled");
        m_coralIntake.setSpeed(0);
        m_coralIntake.stopWristMotor();
        Logger.recordOutput("Intake/smartInRunning", false);

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}