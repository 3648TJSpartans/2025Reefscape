// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeElevatorCmd extends Command {
  // declare the motors and the substem
  private final Elevator elevatorSubsystem;
  private final CoralIntake coralIntake;

  /** Creates a new Elevator. */
  public HomeElevatorCmd(Elevator elevatorSubsystem, CoralIntake coralIntake) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralIntake = coralIntake;
    addRequirements(elevatorSubsystem, coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("Elevator/Command/Scheduled", "HomeElevator");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setSpeed(-.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Elevator/Command/Scheduled", "Unscheduled");
    elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.atBottom();
  }
}
