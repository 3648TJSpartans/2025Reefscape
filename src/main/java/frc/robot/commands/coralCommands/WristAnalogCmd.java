package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntake;

import java.util.function.DoubleSupplier;

public class WristAnalogCmd extends Command {
    private final CoralIntake coralIntake;
    private final DoubleSupplier rightStickSupplier;

    public WristAnalogCmd(CoralIntake coralIntake, DoubleSupplier rightStickSupplier) {
        this.coralIntake = coralIntake;
        this.rightStickSupplier = rightStickSupplier;
        addRequirements(coralIntake);

    }

    @Override
    public void execute() {
        coralIntake.setWristSpeed(rightStickSupplier.getAsDouble() * 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        coralIntake.setWristSpeed(0);
    }
}
