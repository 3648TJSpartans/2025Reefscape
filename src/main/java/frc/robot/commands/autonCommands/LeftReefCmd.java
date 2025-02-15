package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coralCommands.CoralCmd;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class LeftReefCmd extends SequentialCommandGroup {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private final Drive m_drive;
    private final boolean right;
    private final Command coralCommand;
    private final boolean slam;

    public LeftReefCmd(Drive drive, CoralIntake coralIntake,
            Elevator elevator, int level, boolean slam) {
        super();
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        m_drive = drive;
<<<<<<< Updated upstream:src/main/java/frc/robot/commands/autonCommands/CoralSequentialCmd.java
        this.right = right;
        this.slam = slam;
=======

>>>>>>> Stashed changes:src/main/java/frc/robot/commands/autonCommands/LeftReefCmd.java
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
<<<<<<< Updated upstream:src/main/java/frc/robot/commands/autonCommands/CoralSequentialCmd.java

        addCommands(
                new SequentialCommandGroup(
                        right ? AutoBuildingBlocks.driveToNearest(m_drive, () -> PoseConstants.rightReefPoints())
                                : AutoBuildingBlocks.driveToNearest(m_drive,
                                        () -> PoseConstants.leftReefPoints()),
                        coralCommand, new WaitCommand(0.5), slam ? new CoralCmd(m_coralIntake, .05, -.2) : null));
=======
        addCommands(
                new SequentialCommandGroup(
                        AutoBuildingBlocks.driveToNearest(m_drive, () -> PoseConstants.leftReefPoints()),
                        coralCommand, new WaitCommand(1), slam ? new CoralCmd(m_coralIntake, .05, -.2) : null));
>>>>>>> Stashed changes:src/main/java/frc/robot/commands/autonCommands/LeftReefCmd.java
    }
}