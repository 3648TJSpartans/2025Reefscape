package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class CoralSequentialCmd extends SequentialCommandGroup {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final Command DriveToNearestPoint;
    private final Drive m_drive;
    private final double height;
    private final double angle;

    public CoralSequentialCmd(Drive drive, Command driveToNearestPointCommand, CoralIntake coralIntake,
            Elevator elevator,
            double height, double angle) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_drive = drive;
        DriveToNearestPoint = new DriveToNearest(m_drive, () -> PoseConstants.criticalPoints);
        this.height = height;
        this.angle = angle;
        addRequirements(m_coralIntake);
        addRequirements(m_elevator);

        addCommands(
                new SequentialCommandGroup(DriveToNearestPoint, CoralInCmd, ElevatorCmd));
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        m_elevator.elevateTo(height);
        m_coralIntake.rotateTo(angle);
        DriveToNearestPoint;

    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
        m_coralIntake.stopIntakeMotor();
        m_coralIntake.stopWristMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}