package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommands {

    // Reset elevator to bottom and zero encoder
    public static Command reset(ElevatorSubsystem elevator) {
        return Commands.sequence(
            new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.Reset), elevator),
            new InstantCommand(() -> {
                if (elevator.atBottom()) {
                    elevator.setZero(); // snap encoder to 0
                }
            }, elevator)
        );
    }

    public static Command goToLevel1(ElevatorSubsystem elevator) {
        return new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.LEVEL1), elevator);
    }

    public static Command goToLevel2(ElevatorSubsystem elevator) {
        return new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.LEVEL2), elevator);
    }

    public static Command goToLevel3(ElevatorSubsystem elevator) {
        return new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.LEVEL3), elevator);
    }

    public static Command goToLevel4(ElevatorSubsystem elevator) {
        return new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.LEVEL4), elevator);
    }

    public static Command Algae1(ElevatorSubsystem elevator) {
        return new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.Algae1), elevator);
    }

    public static Command Algae2(ElevatorSubsystem elevator) {
        return new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.Algae2), elevator);
    }

    public static Command AlgaeGround(ElevatorSubsystem elevator) {
        return new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.AlgaeGround), elevator);
    }

    public static Command goBarge(ElevatorSubsystem elevator) {
        return new InstantCommand(() -> elevator.setPosition(ElevatorSubsystem.Barge), elevator);
    }

    public static InstantCommand stop(ElevatorSubsystem elevator) {
        return new InstantCommand(elevator::stop, elevator);
    }
}
