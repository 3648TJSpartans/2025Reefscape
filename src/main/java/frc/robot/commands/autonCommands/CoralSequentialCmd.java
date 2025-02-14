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
    private final Drive m_drive;
    private final double height;
    private final double angle;
    private final boolean right;
    private final Command coralCommand;

    public CoralSequentialCmd(Drive drive, CoralIntake coralIntake,
            Elevator elevator, boolean right,
            double height, double angle, int level) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_drive = drive;
        this.height = height;
        this.angle = angle;
        this.right = right;
        switch (level) {
            case 1:
                coralCommand = AutoBuildingBlocks.l1(coralIntake, elevator);
                break;
            case 2:
                coralCommand = AutoBuildingBlocks.l1(coralIntake, elevator);
                break;
            case 3:
                coralCommand = AutoBuildingBlocks.l1(coralIntake, elevator);
                break;
            case 4:
                coralCommand = AutoBuildingBlocks.l1(coralIntake, elevator);
                break;
            default:
                coralCommand = null;
        }
        addRequirements(m_coralIntake);
        addRequirements(m_elevator);

        addCommands(
                new SequentialCommandGroup(
                        right ? AutoBuildingBlocks.driveToNearest(m_drive, () -> PoseConstants.rightReefPoints())
                                : AutoBuildingBlocks.driveToNearest(m_drive,
                                        () -> PoseConstants.leftReefPoints()),
                        coralCommand));
    }

}