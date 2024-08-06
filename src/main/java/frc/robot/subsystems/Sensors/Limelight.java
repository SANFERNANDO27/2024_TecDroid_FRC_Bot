package frc.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public double getX() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0.0);
    }

    public double getY() {
        NetworkTableEntry ty = table.getEntry("ty");
        return ty.getDouble(0.0);
    }

    public double getArea() {
        NetworkTableEntry ta = table.getEntry("ta");
        return ta.getDouble(0.0);
    }
    
}
