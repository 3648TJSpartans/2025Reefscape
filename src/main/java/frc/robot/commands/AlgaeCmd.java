package frc.robot.commands;

import frc.robot.subsystems.algae.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeCmd extends Command {
    private final AlgaeSubsystem algaeSubsystem;
    private final double angle;

    public AlgaeCmd(AlgaeSubsystem algaeSubsystem, double angle) {
        this.algaeSubsystem = algaeSubsystem;
        this.angle = angle;
        // this.RTSupplier = RTSupplier;
        addRequirements(algaeSubsystem);

    }

    @Override
    public void execute() {
        algaeSubsystem.setLiftPosition(angle);
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setLiftSpeed(0);
    }
}
