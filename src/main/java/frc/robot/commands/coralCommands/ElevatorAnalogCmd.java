package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class ElevatorAnalogCmd extends Command {
    private final Elevator elevator;
    private final DoubleSupplier rightStickSupplier;

    public ElevatorAnalogCmd(Elevator elevator, DoubleSupplier rightStickSupplier) {
        this.elevator = elevator;
        this.rightStickSupplier = rightStickSupplier;
        addRequirements(elevator);

    }

    @Override
    public void initialize() {
        Logger.recordOutput("Elevator/Command/Scheduled", "ElevatorAnalog");
    }

    @Override
    public void execute() {
        elevator.setSpeed(rightStickSupplier.getAsDouble() * 0.5);
    }

    @Override
    public void end(boolean interupted) {
        Logger.recordOutput("Elevator/Command/Scheduled", "Unscheduled");
        elevator.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
