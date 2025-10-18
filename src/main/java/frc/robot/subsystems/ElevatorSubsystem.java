package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ElevatorSubsystem extends SubsystemBase {

    // ======= mech + control constants (tune these) =======
    // If you know your gearing and drum, set the ratio so "mechanism rotations" map cleanly.
    // Example: 10.0 means 10 motor revs = 1 mechanism rev.
    // private static final double SENSOR_TO_MECH_RATIO = 10.0; // TODO tune or comment out if you’d rather stay in motor rot units

    // PIDF (regular Motion Magic, not Expo)
    private static final double kS = 0.25;   // V, static
    private static final double kV = 0.12;   // V per rps
    private static final double kA = 0.01;   // V per rps^2
    private static final double kP = 4.8;
    private static final double kI = 0.0;
    private static final double kD = 0.10;

    // Gravity feedforward so it holds position without fighting with P
    private static final double kG = 0.5;    // V to hold. Start ~0.3–0.8, tune on-robot

    // Motion Magic constraints
    private static final double CRUISE_VEL_RPS = 80.0;   // rot/s
    private static final double ACCEL_RPS2    = 160.0;   // rot/s^2
    private static final double JERK_RPS3     = 800.0;   // rot/s^3 start conservative; try 1200–1600 if you want snappier ramps

    // Position presets (in rotations of the configured unit)
    public static final double Reset  = 0.0;
    public static final double Intake = 0.9;
    public static final double LEVEL1 = 12.0;
    public static final double LEVEL2 = 20.0;
    public static final double LEVEL3 = 29.0;
    public static final double LEVEL4 = 49.0;
    public static final double Algae1 = 12.0;
    public static final double Algae2 = 22.0;

    public static final double POSITION_TOLERANCE = 0.20;

    // ======= hardware =======
    private final TalonFX leader = new TalonFX(Configs.CAN.ElevatorRight);
    private final TalonFX follower = new TalonFX(Configs.CAN.ElevatorLeft);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    public ElevatorSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Motor output + brake for crisp holding
        MotorOutputConfigs mo = cfg.MotorOutput;
        mo.Inverted = InvertedValue.Clockwise_Positive;     // flip if needed
        mo.NeutralMode = NeutralModeValue.Brake;

        // Current limits so it doesn’t cook on jams
        CurrentLimitsConfigs cl = cfg.CurrentLimits;
        cl.SupplyCurrentLimitEnable = true;
        cl.SupplyCurrentLimit = 40.0;      // amps
        cl.SupplyCurrentLowerLimit = 30.0; // amps to drop back to
        cl.SupplyCurrentLowerTime = 0.5;   // seconds over-limit before lowering
        

        // Optional voltage comp for consistent feel
        cfg.Voltage.PeakForwardVoltage = 12.0;
        cfg.Voltage.PeakReverseVoltage = -12.0;

        // Sensor units mapping to mechanism rotations (optional but nice)
        FeedbackConfigs fb = cfg.Feedback;
        // fb.SensorToMechanismRatio = SENSOR_TO_MECH_RATIO; // comment out if you want raw motor rotations

        // Slot 0 gains
        Slot0Configs s0 = cfg.Slot0;
        s0.kS = kS;
        s0.kV = kV;
        s0.kA = kA;
        s0.kP = kP;
        s0.kI = kI;
        s0.kD = kD;
        s0.kG = kG;
        s0.GravityType = GravityTypeValue.Elevator_Static;

        // Motion Magic constraints
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = CRUISE_VEL_RPS;
        mm.MotionMagicAcceleration   = ACCEL_RPS2;
        mm.MotionMagicJerk           = JERK_RPS3;

        // Apply to leader
        leader.getConfigurator().apply(cfg);

        // Zero position on boot (use a homing routine if you have a hard bottom switch)
        leader.setPosition(0);

        // Follower. Flip "true" if the motors face opposite directions mechanically.
        follower.setControl(new Follower(leader.getDeviceID(), true));
        follower.getConfigurator().apply(cfg);
    }

    /** Move elevator to target position (in configured rotations) */
    public void setPosition(double rotations) {
        leader.setControl(motionMagic.withPosition(rotations));
    }

    /** Stop right now */
    public void stop() {
        leader.stopMotor();
    }

    /** Force encoder to zero (use after homing) */
    public void setZero() {
        leader.setPosition(0);
    }

    /** Current position in rotations */
    public double getPosition() {
        return leader.getRotorPosition().getValue().in(Rotations);
    }

    /** Within tolerance of target */
    public boolean atTarget(double target) {
        return Math.abs(getPosition() - target) <= POSITION_TOLERANCE;
    }

    /** At bottom based on position. Swap to a limit switch if you have one */
    public boolean atBottom() {
        return getPosition() <= Reset + POSITION_TOLERANCE;
    }

    // ======= optional: quick homing helper if you add a limit switch =======
    // Call this with a small negative percent output until the bottom switch hits,
    // then setZero(). Don’t “stall to bottom” on purpose, that’s pain.
    // public void homeToBottom(Supplier<Boolean> bottomLimit) {
    //     if (!bottomLimit.get()) {
    //         leader.set(-0.1); // gentle down
    //     } else {
    //         leader.stopMotor();
    //         setZero();
    //     }
    // }
}
