package frc.excalib.slam.mapper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.*;

public class AuroraClient {
    private final NetworkTableInstance instance;
    private final NetworkTable table;
    private final NetworkTableEntry xEntry, yEntry, zEntry, rollEntry, pitchEntry, yawEntry;

    public AuroraClient(NetworkTableInstance instance) {
        this.instance = instance;
        table = instance.getTable("Aurora/Localization");
        xEntry = table.getEntry("X");
        yEntry = table.getEntry("Y");
        zEntry = table.getEntry("Z");
        rollEntry = table.getEntry("Roll");
        pitchEntry = table.getEntry("Pitch");
        yawEntry = table.getEntry("Yaw");
    }


    public Pose3d getPose() {
        Pose3d pose = new Pose3d(
                xEntry.getDouble(-1),
                yEntry.getDouble(-1),
                zEntry.getDouble(-1),
                new Rotation3d(
                        rollEntry.getDouble(100),
                        pitchEntry.getDouble(100),
                        yawEntry.getDouble(100)));

        if (pose.getX() == -1 ||
                pose.getY() == -1 ||
                pose.getZ() == -1 ||
                pose.getRotation().getX() == 100 ||
                pose.getRotation().getY() == 100 ||
                pose.getRotation().getZ() == 100) return null;
        return pose;
    }

    public double getX () {
        if (xEntry==null) System.out.println("yehuda");
        return xEntry.getDouble(-1);
    }

    public double getY () {
        return yEntry.getDouble(-1);
    }

    public double getZ () {
        return zEntry.getDouble(-1);
    }
    public double getRoll () {
        return rollEntry.getDouble(100);
    }

    public double getPitch () {
        return pitchEntry.getDouble(100);
    }

    public double getYaw () {
        return yawEntry.getDouble(100);
    }
}
