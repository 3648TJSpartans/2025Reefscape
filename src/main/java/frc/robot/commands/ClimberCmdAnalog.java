package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

import java.util.function.DoubleSupplier;

public class ClimberCmdAnalog extends Command {
    private final ClimberSubsystem climberSubsystem;
    private DoubleSupplier rightStickSupplier;

    public ClimberCmdAnalog(ClimberSubsystem climberSubsystem, DoubleSupplier rightStickSupplier) {
        this.climberSubsystem = climberSubsystem;
        this.rightStickSupplier = rightStickSupplier;
        addRequirements(climberSubsystem);

    }

    @Override
    public void execute() {
        climberSubsystem.setSpeed(rightStickSupplier.getAsDouble() * 0.25);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setSpeed(0);
    }
}
