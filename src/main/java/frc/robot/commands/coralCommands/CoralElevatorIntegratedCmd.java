package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class CoralElevatorIntegratedCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final double height;
    private final double angle;

    public CoralElevatorIntegratedCmd(CoralIntake coralIntake, Elevator elevator, double height, double angle) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        this.height = height;
        this.angle = angle;
        addRequirements(m_coralIntake);
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.elevateTo(height);
        m_coralIntake.rotateTo(angle);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
        m_coralIntake.stopIntakeMotor();
        m_coralIntake.stopWristMotor();
    }

    @Override
    public boolean isFinished() {
        return (m_elevator.getHeight() - ElevatorConstants.marginOfError) < m_elevator.getHeight() &&
                m_elevator.getHeight() < (m_elevator.getHeight() + CoralIntakeConstants.marginOfError) &&
                (m_coralIntake.getAngle() - CoralIntakeConstants.marginOfError) < m_coralIntake.getAngle() &&
                m_coralIntake.getAngle() < (m_coralIntake.getAngle() + CoralIntakeConstants.marginOfError);
    }

}