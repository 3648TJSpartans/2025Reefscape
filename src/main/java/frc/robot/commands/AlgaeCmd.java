package frc.robot.commands;

import frc.robot.subsystems.algae.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

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
