package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDColor extends InstantCommand {
    public SetLEDColor(LEDSubsystem leds, Color color) {
        super(
            () -> {
                leds.setSolid(color);
                System.out.println("[LED] Set color: " + color.toString());
            },
            leds
        );
    }
}
