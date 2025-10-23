package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeRangeSubsystem extends SubsystemBase {
    private final CANrange sensor = new CANrange(Configs.CAN.IntakeDetect);

    public double getDistanceMeters() {
        return sensor.getDistance().getValueAsDouble(); // meters
    }
    
    public boolean isWithinRange(double minMeters, double maxMeters) {
        // auto-fix swapped bounds
        double lo = Math.min(minMeters, maxMeters);
        double hi = Math.max(minMeters, maxMeters);
        double d = getDistanceMeters();
        return d >= lo && d <= hi;
    }
}
