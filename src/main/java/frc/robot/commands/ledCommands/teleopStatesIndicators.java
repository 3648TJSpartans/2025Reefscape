// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ledCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeIOSparkMax;
import frc.robot.subsystems.coralIntake.CoralIntake;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.*;
import frc.robot.commands.autonCommands.CoralSequentialCmd;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class teleopStatesIndicators extends Command {
    private final LedSubsystem m_leds;
    private final CoralIntake m_corCoralIntake;

    public teleopStatesIndicators(LedSubsystem leds, CoralIntake coralIntake) {
        m_leds = leds;
        m_corCoralIntake = coralIntake;
        addRequirements(m_leds);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // right case
        if (CoralSequentialCmd.getLeftOrRIghtState() == 1) {
            if (m_corCoralIntake.getIR() == false) {
                m_leds.setLedPattern(LedConstants.teal, CoralSequentialCmd.getLevel());
            }
            if (m_corCoralIntake.getIR() == true) {
                m_leds.setLedPattern(LedConstants.blinkingteal, CoralSequentialCmd.getLevel());
            }
        }
        // left case
        if (CoralSequentialCmd.getLeftOrRIghtState() == 0) {
            if (m_corCoralIntake.getIR() == false) {
                m_leds.setLedPattern(LedConstants.blue, CoralSequentialCmd.getLevel());
            }
            if (m_corCoralIntake.getIR() == true) {
                m_leds.setLedPattern(LedConstants.blinkingBlue, CoralSequentialCmd.getLevel());
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_leds.turnLedsOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
