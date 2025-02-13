package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

public class NeuralVision {

    private final double Kp = -0.1; // Proportional control constant
    private final NetworkTable table;
    private double tx = 0.0;

    public NeuralVision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void adjustValues() {
        tx = table.getEntry("tx").getDouble(0.0); // Get tx, default to 0.0 if not present
    }

    public double getTx() {
        return tx;
    }

    public int getCurrentPipeline() {
        return (int) table.getEntry("pipeline").getDouble(0);
    }

    public void switchPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
}