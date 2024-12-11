package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.ejml.interfaces.decomposition.LUSparseDecomposition;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.MotorMMove;

public class MotorMoveCmd extends Command {
    private final MotorMMove m_motorMMove;

    public MoveMotorCmd(MotorMMove motorMMove){
        m_motorMMove = motorMMove;
        addRequirements(m_motorMMove);
    }

    
    @Override
    public void initialize() {
        m_ledsSubsystem.setColor(LedConstants.intakeRunningRGB, LedConstants.topBarLedStart,
                LedConstants.topBarLedStop);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setBeltSpeed(ShooterConstants.beltAmpSpeed);
        m_intakeSubsystem
                .setIntakeSpeed(IntakeConstants.IntakeSpeed);

    }

    public boolean isFinished() {
        if (!m_IRSensor.get()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ledsSubsystem.setIntakeColor(m_IRSensor);
        m_intakeSubsystem.setIntakeSpeed(Constants.IntakeConstants.DefaultSpeed);
        m_shooterSubsystem.setBeltSpeed(ShooterConstants.DefaultSpeed);
        m_shooterSubsystem.setShooterVelocity(ShooterConstants.DefaultSpeed, ShooterConstants.DefaultSpeed);
    }

}