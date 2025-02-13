package frc.robot.commands.algaeCommands;

import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class AlgaeDownCmd extends Command {
    private final AlgaeSubsystem algaeSubsystem;

    public AlgaeDownCmd(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
    }

    @Override
    public void execute() {
        algaeSubsystem.setLiftPosition(AlgaeConstants.liftIntakePosition);// lift down
        algaeSubsystem.setIntakeSpeed(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setLiftSpeed(0);
    }
}