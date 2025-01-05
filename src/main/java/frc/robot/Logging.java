package frc.robot;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Logging {

    private static Logging singleInstance = null;

    StringLogEntry values;

    private Logging() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLog log = DataLogManager.getLog();
        values = new StringLogEntry(log, "/my/string");
    }

    public void log(String input, String value) {
        values.append(input + ", " + value);
    }

    public static synchronized Logging getInstance() {
        if (singleInstance == null)
            singleInstance = new Logging();
        return singleInstance;
    }
}