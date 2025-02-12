package frc.robot.commands.algaeCommands;

import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class AlgaeDefaultCmd extends Command {
    private final AlgaeSubsystem algaeSubsystem;

    public AlgaeDefaultCmd(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
        addRequirements(algaeSubsystem);
    }

    @Override
    public void execute() {
        algaeSubsystem.setIntakeSpeed(0);
        algaeSubsystem.setLiftPosition(AlgaeConstants.liftUpWithBall);
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setLiftSpeed(0);
    }
}
