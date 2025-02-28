package frc.robot.commands.sftCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.sft.Sft;
import frc.robot.subsystems.coralIntake.CoralIntake;

public class SftCmd extends Command {
    private final Sft m_sft;
    private final double angle;

    public SftCmd(Sft sft, double angle) {
        m_sft = sft;
        this.angle = angle;
        addRequirements(m_sft);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_sft.rotateTo(angle);
    }

    @Override
    public void end(boolean interrupted) {
        m_sft.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return /* m_coralIntake.getIR() && */ ((angle - CoralIntakeConstants.marginOfError) < m_sft.getAngle()
                &&
                m_sft.getAngle() < (angle + CoralIntakeConstants.marginOfError));
    }

}