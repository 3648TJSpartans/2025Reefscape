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
import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntake;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
