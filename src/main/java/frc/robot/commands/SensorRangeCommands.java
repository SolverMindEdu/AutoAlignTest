package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeRangeSubsystem;
import frc.robot.subsystems.EECoralRangeSubsystem;
import frc.robot.subsystems.AlgaeDetectRangeSubsystem;

public class SensorRangeCommands {
    /** existing */
    public static Command waitForCoral(EECoralRangeSubsystem sensor, double min, double max) {
        return new WaitUntilCommand(() -> sensor.isWithinRange(min, max));
    }
    /** existing */
    public static Command waitForAlgae(AlgaeDetectRangeSubsystem sensor, double min, double max) {
        return new WaitUntilCommand(() -> sensor.isWithinRange(min, max));
    }
    /** existing */
    public static Command waitForIntake(IntakeRangeSubsystem sensor, double min, double max) {
        return new WaitUntilCommand(() -> sensor.isWithinRange(min, max));
    }

    public static Command waitForIntakeFartherThan(IntakeRangeSubsystem sensor, double thresholdMeters) {
        return new edu.wpi.first.wpilibj2.command.WaitUntilCommand(
            () -> sensor.getDistanceMeters() >= thresholdMeters
        );
    }
    
    // (optional) if you ever need the opposite
    public static Command waitForIntakeCloserThan(IntakeRangeSubsystem sensor, double thresholdMeters) {
        return new WaitUntilCommand(() -> sensor.getDistanceMeters() <= thresholdMeters);
    }
}
