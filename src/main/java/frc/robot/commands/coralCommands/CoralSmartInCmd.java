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
        addRequirements(m_coralIntake);
    }

    @Override
    public void initialize() {

        isFinished = false;
        m_timer = new Timer();
        m_timer.start();

    }

    @Override
    public void execute() {
        boolean objectDetected = !m_coralIntake.getIR();
        // if (!objectDetected) {

        // m_timer.reset();
        // m_timer.start();
        // }
        isFinished = objectDetected;// && m_timer.get() > 0.4;

        Logger.recordOutput("Timer/time", m_timer.get());
        m_coralIntake.rotateTo(angle);
        m_coralIntake.setSpeed(CoralIntakeConstants.intakeSpeed);

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