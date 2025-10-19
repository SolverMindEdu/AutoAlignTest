package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(Configs.CAN.ElevatorRight); // main motor
    private final TalonFX follower = new TalonFX(Configs.CAN.ElevatorLeft); // follows leader

    // Motion Magic request object
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    // Positions in rotations
    public static final double Reset  = 0.0;
    public static final double LEVEL1 = 14.5;
    public static final double LEVEL2 = 19.0;
    public static final double LEVEL3 = 31.0;
    public static final double LEVEL4 = 48.0;
    public static final double Algae1 = 15.0;
    public static final double Algae2 = 22;
    public static final double Barge = 51.0;
    public static final double AlgaeGround = 5;

    // limits + tolerance 
    public static final double POSITION_TOLERANCE = 0.2;
    private static final double MIN_POS = Reset;      // floor clamp
    private static final double MAX_POS = Barge;     // top clamp
    private static final double FLOOR_MARGIN = 0.00;  // safety snap zone

    // track last target for graphs
    private double lastSetpointRot = Reset;

    public ElevatorSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // output settings
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID + FF (start simple, tune on-robot)
        Slot0Configs slot0 = configs.Slot0;
        slot0.kP = 2.0;
        slot0.kI = 0.0;        // set I to 0 first; add later only if small steady error remains
        slot0.kD = 0.05;
        slot0.kV = 0.01;        // add if you need better cruise tracking
        slot0.kS = 0.25;
        slot0.kG = 0.45;        // holding voltage; tune so it hovers without drifting
        slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Motion Magic constraints
        configs.MotionMagic.MotionMagicCruiseVelocity = 39;  // rot/s
        configs.MotionMagic.MotionMagicAcceleration   = 35;  // rot/s^2
        configs.MotionMagic.MotionMagicJerk           = 800; // rot/s^3, shapes start/stop smoothness

        // reverse soft limit so it can’t command below the floor
        SoftwareLimitSwitchConfigs soft = configs.SoftwareLimitSwitch;
        // enable reverse soft limit (bottom)
        soft.ReverseSoftLimitEnable = true;
        soft.ReverseSoftLimitThreshold = Reset; // this is the new name ✅
        
        // optional: forward/top limit
        // soft.ForwardSoftLimitEnable = true;
        // soft.ForwardSoftLimitThreshold = MAX_POS;
        
        // optional: cap the very top too
        // soft.ForwardSoftLimitEnable = true;
        // soft.ForwardSoftLimit = MAX_POS;

        leader.getConfigurator().apply(configs);

        // follower mirrors leader (flip true only if mounted opposite)
        follower.setControl(new com.ctre.phoenix6.controls.Follower(Configs.CAN.ElevatorRight, true));

        // reset encoder after config
        leader.setPosition(Reset);
        lastSetpointRot = Reset;
    }

    /** Move elevator to given target rotations (clamped to safe range) */
    public void setPosition(double rotations) {
        double clamped = MathUtil.clamp(rotations, MIN_POS, MAX_POS);
        lastSetpointRot = clamped;
        leader.setControl(motionMagic.withPosition(clamped));
    }

    /** Stop elevator immediately */
    public void stop() {
        leader.stopMotor();
    }

    /** Force reset encoder position to zero (use after homing) */
    public void setZero() {
        leader.setPosition(Reset);
        lastSetpointRot = Reset;
    }

    /** Read current position in rotations */
    public double getPosition() {
        return leader.getRotorPosition().getValue().in(Rotations);
    }

    /** Returns true if elevator is within tolerance of target */
    public boolean atTarget(double target) {
        return Math.abs(getPosition() - target) <= POSITION_TOLERANCE;
    }

    /** Returns true if at physical bottom (safe to zero) */
    public boolean atBottom() {
        return getPosition() <= Reset + POSITION_TOLERANCE;
    }

    /** Push live data, guard floor */
    @Override
    public void periodic() {
        // last-ditch guard: if sensor ever dips below floor, freeze and snap back
        double pos = getPosition();
        if (pos < MIN_POS - FLOOR_MARGIN) {
            leader.stopMotor();
            leader.setPosition(MIN_POS);
        }

        // live telemetry for tuning
        SmartDashboard.putNumber("Elevator/Pos (rot)", pos);
        SmartDashboard.putNumber("Elevator/Setpoint (rot)", lastSetpointRot);
        SmartDashboard.putNumber("Elevator/Error (rot)", lastSetpointRot - pos);
        SmartDashboard.putBoolean("Elevator/At Target", atTarget(lastSetpointRot));
        SmartDashboard.putNumber("Elevator/Vel (rps)",
            leader.getRotorVelocity().getValue().in(RotationsPerSecond));
        SmartDashboard.putNumber("Elevator/Supply Current (A)",
            leader.getSupplyCurrent().getValueAsDouble());
    }
}
