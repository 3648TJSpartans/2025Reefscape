package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.OnTheFlyAutons.AutonConstants;
import frc.robot.commands.OnTheFlyAutons.AutonConstants.PoseConstants;
import frc.robot.commands.coralCommands.CoralElevatorIntegratedCmd;
import frc.robot.commands.coralCommands.CoralInCmd;
import frc.robot.commands.coralCommands.CoralOutCmd;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.TunableNumber;

public class AutoBuildingBlocks {
    public static Command intake(CoralIntake m_coralIntake) {
        return new CoralInCmd(m_coralIntake);

    }

    public static Command outtake(CoralIntake m_coralIntake) {
        return new CoralOutCmd(m_coralIntake);
    }

    public static Command l1(CoralIntake m_coral, Elevator m_elevator) {
        return new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                new TunableNumber("Elevator/Height/L1", ElevatorConstants.coralLeveL1).get(),
                new TunableNumber("Elevator/Angle/L1", CoralIntakeConstants.L1Angle).get());
    }

    public static Command l2(CoralIntake m_coral, Elevator m_elevator) {
        return new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                new TunableNumber("Elevator/Height/L2", ElevatorConstants.coralLeveL2).get(),
                new TunableNumber("Elevator/Angle/L2", CoralIntakeConstants.L2Angle).get());
    }

    public static Command l3(CoralIntake m_coral, Elevator m_elevator) {
        return new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                new TunableNumber("Elevator/Height/L3", ElevatorConstants.coralLeveL3).get(),
                new TunableNumber("Elevator/Angle/L3", CoralIntakeConstants.L3Angle).get());
    }

    public static Command l4(CoralIntake m_coral, Elevator m_elevator) {
        return new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                new TunableNumber("Elevator/Height/L4", ElevatorConstants.coralLeveL4).get(),
                new TunableNumber("Elevator/Angle/L4", CoralIntakeConstants.L4Angle).get());
    }

    public static Command driveToNearestRight(Drive m_drive){
        return new DriveToNearest(m_drive, ()-> PoseConstants.)
    }
}
