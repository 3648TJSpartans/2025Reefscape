package frc.robot.commands.sftCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.sft.Sft;
import frc.robot.subsystems.coralIntake.CoralIntake;

public class SftAnalogCmd extends Command {
    private final Sft m_sft;
    private final Supplier<Double> speed;

    public SftAnalogCmd(Sft sft, Supplier<Double> speed) {
        m_sft = sft;
        this.speed = speed;
        addRequirements(m_sft);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_sft.setSpeed(speed.get());
    }

    @Override
    public void end(boolean interrupted) {
        m_sft.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}