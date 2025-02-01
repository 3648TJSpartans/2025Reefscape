package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.coralSubsystems.elevator.Elevator;

import java.util.function.DoubleSupplier;

public class ElevatorAnalogCmd extends Command {
    private final Elevator elevator;
    private final DoubleSupplier rightStickSupplier;

    public ElevatorAnalogCmd(Elevator elevator, DoubleSupplier rightStickSupplier) {
        this.elevator = elevator;
        this.rightStickSupplier = rightStickSupplier;
        addRequirements(elevator);

    }

    @Override
    public void execute() {
        elevator.setSpeed(rightStickSupplier.getAsDouble() * 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setSpeed(0);
    }
}
