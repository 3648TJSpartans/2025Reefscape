package frc.robot.commands.autonCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.goToCommands.DriveToPose;
import frc.robot.commands.coralCommands.CoralElevatorIntegratedCmd;
import frc.robot.commands.coralCommands.CoralInCmd;
import frc.robot.commands.coralCommands.CoralOutCmd;
import frc.robot.commands.coralCommands.CoralSmartLevelCmd;
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

    public static Command driveToNearest(Drive m_drive, Supplier<Pose2d[]> points) {
        return new DriveToNearest(m_drive, points);
    }

    public static Command driveToPose(Drive m_drive, Pose2d point) {
        return new DriveToPose(m_drive, () -> point);
    }

    public static Command setCoralPose(Elevator m_elevator, CoralIntake m_coral, double height, double angle) {
        return new CoralElevatorIntegratedCmd(m_coral, m_elevator, height, angle);
    }

    public static Command intakeSource(Elevator m_elevator, CoralIntake m_coral) {
        return new CoralElevatorIntegratedCmd(m_coral, m_elevator, ElevatorConstants.intakePose,
                CoralIntakeConstants.IntakeAngle);
    }

    public static Command coralSmartLevelCommand(Elevator elevator, CoralIntake intake, Supplier<Integer> level) {
        return new CoralSmartLevelCmd(intake, elevator, level);
    }
}
