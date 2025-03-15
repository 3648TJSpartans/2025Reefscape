package frc.robot.commands.coralCommands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonCommands.AutoBuildingBlocks;
import frc.robot.commands.goToCommands.AutonConstants;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.TunableNumber;

public class CoralSmartLevelCmd extends Command {
    private final CoralIntake m_coralIntake;
    private final Elevator m_elevator;
    private double height;
    private double angle;
    private Supplier<Integer> level;
    private final boolean up;

    public CoralSmartLevelCmd(CoralIntake coralIntake, Elevator elevator, Supplier<Integer> level, boolean up) {
        m_coralIntake = coralIntake;
        m_elevator = elevator;
        this.level = level;
        this.up = up;
        addRequirements(m_coralIntake);
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        updateHandA();
    }

    @Override
    public void execute() {
        updateHandA();
        if (m_elevator.getHeight() < AutonConstants.elevatorCutoff
                && m_coralIntake.getAngle() > AutonConstants.coralCutoff) {
            m_elevator.elevateTo(AutonConstants.elevatorCutoff + 1);
        } else {
            m_elevator.elevateTo(height);
            m_coralIntake.rotateTo(angle);
        }
        Logger.recordOutput("Commands/CoralSmartCommand/setHeight", height);
        Logger.recordOutput("Commands/CoralSmartCommand/setAngle", angle);
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

    public void updateHandA() {
        switch (level.get()) {
            case 1:
                angle = CoralIntakeConstants.L1Angle;
                height = ElevatorConstants.coralLeveL1;
                break;
            case 2:
                angle = CoralIntakeConstants.L2Angle;
                height = ElevatorConstants.coralLeveL2;
                break;
            case 3:
                angle = up ? CoralIntakeConstants.upAngle : CoralIntakeConstants.L3Angle;
                height = ElevatorConstants.coralLeveL3;
                break;
            case 4:
                angle = up ? CoralIntakeConstants.upAngle : CoralIntakeConstants.L4Angle;
                height = ElevatorConstants.coralLeveL4;
                break;
            default:
                // SEE HERE

                System.out.println("IF THIS IS RUNNING< YOU %#$! UP!, Go To Coral Smart Command");
                System.out.println("Level Inputed: " + level.get());
        }
    }
}