package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

import java.util.function.DoubleSupplier;

public class ClimberAnalogCmd extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final DoubleSupplier rightStickSupplier;

    public ClimberAnalogCmd(ClimberSubsystem climberSubsystem, DoubleSupplier rightStickSupplier) {
        this.climberSubsystem = climberSubsystem;
        this.rightStickSupplier = rightStickSupplier;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        if (rightStickSupplier.getAsDouble() > 0) {
            if (climberSubsystem.getPosition() < ClimberConstants.downPosition1) {
                climberSubsystem.setSpeed(rightStickSupplier.getAsDouble() * 0.25);
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setSpeed(0);
    }
}
