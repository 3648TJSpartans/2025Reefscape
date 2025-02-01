package frc.robot.commands;

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
import frc.robot.subsystems.climber.ClimberSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ClimberCmd extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final DoubleSupplier rightStickSupplier;

    public ClimberCmd(ClimberSubsystem climberSubsystem, DoubleSupplier rightStickSupplier) {
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
