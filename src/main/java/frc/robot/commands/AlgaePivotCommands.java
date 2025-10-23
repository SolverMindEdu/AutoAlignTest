package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaePivotSubsystem;

public class AlgaePivotCommands {
    public static InstantCommand stow(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.RESET), algae);
    }

    public static InstantCommand runEE(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.RunEE), algae);
    }

    public static InstantCommand level1(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.level1), algae);
    }

    public static InstantCommand algae(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.Algae), algae);
    }

    public static InstantCommand algaeground(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.algaeground), algae);
    }

    public static InstantCommand AlgaeEErun(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.AlgaeEERun), algae);
    }

    public static InstantCommand l2Algae(AlgaePivotSubsystem algae) {
        return new InstantCommand(() -> algae.setPosition(AlgaePivotSubsystem.l2algae), algae);
    }

    public static InstantCommand stop(AlgaePivotSubsystem algae) {
        return new InstantCommand(algae::stop, algae);
    }
}
