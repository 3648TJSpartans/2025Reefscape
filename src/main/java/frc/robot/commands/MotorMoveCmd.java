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

    public MotorMoveCmd(MotorMMove motorMMove){
        this.m_motorMMove = motorMMove;
        addRequirements(m_motorMMove);
    }

    
    @Override
    public void initialize() {
       // m_motorMMove.setMotorSpeed(MotorConstants.speed);
    }

    @Override
    public void execute() {
        m_motorMMove.setMotorSpeed(MotorConstants.speed);

    }

    @Override
    public void end(boolean interrupted) {
        m_motorMMove.setMotorSpeed(0);
        
    }

}