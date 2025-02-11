package frc.robot.commands.algaeCommands;

import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.lang.Math;

public class AlgaeShootCmd extends Command {
    private final AlgaeSubsystem algaeSubsystem;

    public AlgaeShootCmd(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;
    }

    @Override
    public void execute() {
        algaeSubsystem.setLiftPosition(AlgaeConstants.shoot);// lift down
        if (Math.floor(algaeSubsystem.getLiftPosition()*100) == 0.23) {
            algaeSubsystem.setIntakeSpeed(-0.5);
        } else {
            algaeSubsystem.setIntakeSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setLiftSpeed(0);
    }
}
