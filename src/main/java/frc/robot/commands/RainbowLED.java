package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class RainbowLED extends Command {
    private final LEDSubsystem leds;
    private int hue = 0;
    private final int deltaHue = 2;   // change per pixel
    private final int stepSpeed = 3;  // how fast hue shifts (higher = faster)

    public RainbowLED(LEDSubsystem leds) {
        this.leds = leds;
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        hue = 0; // reset hue on start
    }

    @Override
    public void execute() {
        leds.rainbowStep(hue, deltaHue);
        hue = (hue + stepSpeed) % 180; // cycle hue smoothly
    }

    @Override
    public void end(boolean interrupted) {
        // leave the final rainbow instead of turning off
        leds.rainbowStep(hue, deltaHue);
    }

    @Override
    public boolean isFinished() {
        return false; // continuous until interrupted
    }
}
