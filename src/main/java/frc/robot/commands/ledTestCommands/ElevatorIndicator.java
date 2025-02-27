// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledTestCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorIndicator extends Command {
    private final LedSubsystem m_leds;

    public ElevatorIndicator(LedSubsystem leds) {
        m_leds = leds;
        addRequirements(m_leds);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_leds.setLedPattern(LedConstants.elevatorHeight, m_leds.rightSideBuffer);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_leds.setLedPattern(LedConstants.noColor, m_leds.rightSideBuffer);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
