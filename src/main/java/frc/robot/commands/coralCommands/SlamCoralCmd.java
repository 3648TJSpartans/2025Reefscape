// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SlamCoralCmd extends Command {
    private final CoralIntake coralIntake;

    /** Creates a new WristCmd. */
    public SlamCoralCmd(CoralIntake coralIntake) {
        this.coralIntake = coralIntake;
        addRequirements(coralIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.recordOutput("Elevator/Command/Scheduled", "SlamCoral");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        coralIntake.rotateTo(CoralIntakeConstants.slamAngle);
        coralIntake.setSpeed(0.1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // coralIntake.stopIntakeMotor();
        Logger.recordOutput("Elevator/Command/Scheduled", "Unscheduled");
        coralIntake.stopWristMotor();
        coralIntake.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (CoralIntakeConstants.slamAngle - CoralIntakeConstants.marginOfError) < coralIntake.getAngle() &&
                coralIntake.getAngle() < (CoralIntakeConstants.slamAngle + CoralIntakeConstants.marginOfError)
                && !coralIntake.getIR();
    }
}
