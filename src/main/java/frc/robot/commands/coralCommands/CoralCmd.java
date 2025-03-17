package frc.robot.commands.coralCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntake;

public class CoralCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final double intakeSpeed;
    private final double angle;

    public CoralCmd(CoralIntake coralIntake, double angle, double intakeSpeed) {
        m_coralIntake = coralIntake;
        this.intakeSpeed = intakeSpeed;
        this.angle = angle;
        addRequirements(coralIntake);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Elevator/Command/Scheduled", "CoralCmd");
    }

    @Override
    public void execute() {
        m_coralIntake.setSpeed(intakeSpeed);
        m_coralIntake.rotateTo(angle);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Elevator/Command/Scheduled", "Unscheduled");
        m_coralIntake.setSpeed(0);
        m_coralIntake.stopWristMotor();
    }

    @Override
    public boolean isFinished() {
        return /* m_coralIntake.getIR() && */ ((angle - CoralIntakeConstants.marginOfError) < m_coralIntake.getAngle()
                &&
                m_coralIntake.getAngle() < (angle + CoralIntakeConstants.marginOfError));
    }

}