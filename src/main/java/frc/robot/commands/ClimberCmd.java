package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCmd extends Command {
    private final ClimberSubsystem climberSubsystem;

    public ClimberCmd(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.setPosition(ClimberConstants.climbLeftPosition, ClimberConstants.climbRightPosition);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setSpeed(0);
    }
}
