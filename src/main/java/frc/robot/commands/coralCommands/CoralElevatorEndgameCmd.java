package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.goToCommands.AutonConstants;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.TunableNumber;

public class CoralElevatorEndgameCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;

    public CoralElevatorEndgameCmd(CoralIntake coralIntake, Elevator elevator) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
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
            m_elevator.elevateTo(0);
            m_coralIntake.rotateTo(CoralIntakeConstants.endgamePose);
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
        return false;
    }

}