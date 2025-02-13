package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NeuralVision {

    private final NetworkTable table;
    private double ty = 0.0;

    public NeuralVision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void adjustValues() {
        ty = table.getEntry("ty").getDouble(0.0); // Get tx, default to 0.0 if not present
    }

    public double getTx() {
        return ty;
    }

    public int getCurrentPipeline() {
        return (int) table.getEntry("pipeline").getDouble(0);
    }

    public void switchPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
}