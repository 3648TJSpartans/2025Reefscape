package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.goToCommands.AutonConstants;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.TunableNumber;

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
        if (m_elevator.getHeight() < AutonConstants.elevatorCutoff
                && m_coralIntake.getAngle() > AutonConstants.coralCutoff) {
            m_elevator.elevateTo(AutonConstants.elevatorCutoff + 1);
        } else {
            m_elevator.elevateTo(height);
            m_coralIntake.rotateTo(angle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
        m_coralIntake.stopIntakeMotor();
        m_coralIntake.stopWristMotor();
        System.out.println("000000000000elevator  finished000000000000000");
    }

    @Override
    public boolean isFinished() {
        double elevatorMargin = new TunableNumber("Elevator/MarginOfError", ElevatorConstants.marginOfError).get();
        double coralIntakeMargin = new TunableNumber("Wrist/MarginOfError", CoralIntakeConstants.marginOfError).get();
        return ((height - elevatorMargin) < m_elevator.getHeight()) &&
                (m_elevator.getHeight() < (height + elevatorMargin)) &&
                ((angle - coralIntakeMargin) < m_coralIntake.getAngle()) &&
                (m_coralIntake.getAngle() < (angle + coralIntakeMargin));
    }

}