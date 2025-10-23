package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;

public class BlinkLED extends Command {
    private final LEDSubsystem leds;
    private final Color color;
    private final double interval; // seconds per toggle

    private boolean isOn = false;
    private double lastToggleTime = 0;

    public BlinkLED(LEDSubsystem leds, Color color, double interval) {
        this.leds = leds;
        this.color = color;
        this.interval = interval;
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        isOn = false;
        lastToggleTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        leds.off();
    }

    @Override
    public void execute() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (now - lastToggleTime >= interval) {
            isOn = !isOn;
            if (isOn)
                leds.setSolid(color);
            else
                leds.off();
            lastToggleTime = now;
        }
    }

    @Override
    public void end(boolean interrupted) {
        leds.off();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
