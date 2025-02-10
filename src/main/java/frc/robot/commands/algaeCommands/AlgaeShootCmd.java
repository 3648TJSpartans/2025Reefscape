package frc.robot.commands.algaeCommands;

import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeShootCmd extends Command {
    private final AlgaeSubsystem algaeSubsystem;

    public AlgaeShootCmd(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
    }

    @Override
    public void execute() {
        algaeSubsystem.setLiftPosition(AlgaeConstants.shoot);// lift down
        algaeSubsystem.setIntakeSpeed(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setLiftSpeed(0);
    }
}
