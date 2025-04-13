package frc.robot.commands.coralCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class CoralOutCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final Timer m_timer;
    private Boolean isFinished = false;

    public CoralOutCmd(CoralIntake m_coralIntake) {
        this.m_coralIntake = m_coralIntake;
        addRequirements(m_coralIntake);
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Elevator/Command/Scheduled", "CoralOut");
        m_timer.stop();
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        // boolean objectDetected = !m_coralIntake.getIR();
        // if (objectDetected) {
        // m_coralIntake.setSpeed(CoralIntakeConstants.outtakeSpeed);
        // m_timer.reset();
        // } else {
        // if (!m_timer.hasElapsed(0.1)) {
        // if (!m_timer.isRunning()) {
        // m_timer.start();
        // }
        // m_coralIntake.setSpeed(-CoralIntakeConstants.intakeSpeed);
        // } else {
        // isFinished = true;
        // }
        // }
        m_coralIntake.setSpeed(CoralIntakeConstants.outtakeSpeed);
        // isFinished = m_timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Elevator/Command/Scheduled", "Unscheduled");
        m_coralIntake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // return isFinished;
        return false;
    }

}