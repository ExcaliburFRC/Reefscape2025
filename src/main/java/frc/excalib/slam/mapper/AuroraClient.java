package frc.excalib.slam.mapper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.*;

public class AuroraClient {
    private final NetworkTableInstance m_instance;
    private final NetworkTable m_table;
    private final NetworkTableEntry m_xEntry, m_yEntry, m_zEntry, m_rollEntry, m_pitchEntry, m_yawEntry;

    public AuroraClient(NetworkTableInstance instance) {
        m_instance = instance;
        m_table = m_instance.getTable("Aurora/Localization");
        m_xEntry = m_table.getEntry("X");
        m_yEntry = m_table.getEntry("Y");
        m_zEntry = m_table.getEntry("Z");
        m_rollEntry = m_table.getEntry("Roll");
        m_pitchEntry = m_table.getEntry("Pitch");
        m_yawEntry = m_table.getEntry("Yaw");
    }


    public Pose3d getPose() {
        Pose3d pose = new Pose3d(
                getX(), getY(), getZ(),
                new Rotation3d(getRoll(), getPitch(), getYaw())
        );
        if (pose.getX() == -1 ||
                pose.getY() == -1 ||
                pose.getZ() == -1 ||
                pose.getRotation().getX() == 100 ||
                pose.getRotation().getY() == 100 ||
                pose.getRotation().getZ() == 100) return null;
        return pose;
    }

    public double getX() {
        return m_xEntry.getDouble(-1);
    }

    public double getY() {
        return m_yEntry.getDouble(-1);
    }

    public double getZ() {
        return m_zEntry.getDouble(-1);
    }

    public double getRoll() {
        return m_rollEntry.getDouble(100);
    }

    public double getPitch() {
        return m_pitchEntry.getDouble(100);
    }

    public double getYaw() {
        return m_yawEntry.getDouble(100);
    }
}
