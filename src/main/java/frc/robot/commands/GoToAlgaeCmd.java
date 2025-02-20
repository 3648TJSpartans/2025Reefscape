package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.NeuralVision;


public class GoToAlgaeCmd extends Command {
    private final NeuralVision neuralVision;
    private int oldPipeline; 

    public GoToAlgaeCmd(NeuralVision neuralVision) {
        this.neuralVision = neuralVision;
    }

    @Override
    public void initialize() {
        oldPipeline = neuralVision.getCurrentPipeline(); // Saves the current pipeline being used
        neuralVision.switchPipeline(1); // Switches to pipeline 1 (Neural network)
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        neuralVision.switchPipeline(oldPipeline); // Switches back to pipeline being used before the command
    }
}