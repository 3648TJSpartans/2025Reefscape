package frc.robot.commands.autonCommands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coralCommands.CoralCmd;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class RightReefCmd extends SequentialCommandGroup {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final Drive m_drive;
    private final Command coralCommand;

    public RightReefCmd(Drive drive, CoralIntake coralIntake,
            Elevator elevator, int level, boolean slam) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_drive = drive;
        switch (level) {
            case 1:
                coralCommand = AutoBuildingBlocks.l1(coralIntake, elevator);
                break;
            case 2:
                coralCommand = AutoBuildingBlocks.l2(coralIntake, elevator);
                break;
            case 3:
                coralCommand = AutoBuildingBlocks.l3(coralIntake, elevator);
                break;
            case 4:
                coralCommand = AutoBuildingBlocks.l4(coralIntake, elevator);
                break;
            default:
                coralCommand = null;
        }
        addRequirements(m_coralIntake);
        addRequirements(m_elevator);
        addCommands(
                new SequentialCommandGroup(
                        AutoBuildingBlocks.driveToNearest(m_drive, () -> PoseConstants.rightReefPoints()),
                        coralCommand, new WaitCommand(1), slam ? new CoralCmd(m_coralIntake, .05, -.2) : null));
    }

}