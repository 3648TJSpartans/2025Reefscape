package frc.robot.commands.coralCommands;

import frc.robot.Constants.MotorConstants;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

import org.ejml.interfaces.decomposition.LUSparseDecomposition;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.coralSubsystems.elevator.Elevator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
