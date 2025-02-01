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
import frc.robot.subsystems.algae.AlgaeSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AlgaeCmd extends Command {
    private final AlgaeSubsystem algaeSubsystem;
    private final DoubleSupplier leftStickSupplier;

    public AlgaeCmd(AlgaeSubsystem algaeSubsystem, DoubleSupplier leftStickSupplier) {
        this.algaeSubsystem = algaeSubsystem;
        this.leftStickSupplier = leftStickSupplier;
        // this.RTSupplier = RTSupplier;
        addRequirements(algaeSubsystem);

    }

    @Override
    public void execute() {
        algaeSubsystem.setLiftSpeed(0.5 * leftStickSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setLiftSpeed(0);
    }
}
