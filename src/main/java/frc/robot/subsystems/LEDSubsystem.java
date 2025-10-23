package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 0;
  private static final int kLength = 31;

  private final AddressableLED led = new AddressableLED(kPort);
  private final AddressableLEDBuffer buf = new AddressableLEDBuffer(kLength);

  private enum Mode { SOLID, BLINK, RAINBOW }
  private Mode mode = Mode.RAINBOW;

  // SOLID
  private Color solidColor = Color.kPurple; // default boot color

  // BLINK
  private boolean blinkOn = false;
  private Color blinkColor = Color.kRed;
  private double blinkIntervalSec = 0.7;
  private double lastToggle = 0.0;

  // RAINBOW
  private int rainbowHue = 0;
  private int rainbowDelta = 3; // band spacing
  private int rainbowSpeed = 3; // hue advance per tick

  public LEDSubsystem() {
    led.setLength(buf.getLength());
    fillSolid(solidColor);
    led.setData(buf);
    led.start();
  }

  /* ---------- public API your commands already use ---------- */

  public void setSolid(Color color) {
    mode = Mode.SOLID;
    solidColor = color;
    fillSolid(solidColor);
    led.setData(buf);
  }

  public void off() {
    setSolid(Color.kBlack);
  }

  /** helper used by your RainbowLED command */
  public void rainbowStep(int startHue, int deltaHue) {
    mode = Mode.RAINBOW;
    rainbowHue = startHue % 180;
    rainbowDelta = Math.max(1, deltaHue);
    // draw once immediately
    for (int i = 0; i < buf.getLength(); i++) {
      int hue = (rainbowHue + i * rainbowDelta) % 180;
      buf.setHSV(i, hue, 255, 128);
    }
    led.setData(buf);
  }

  /* ---------- new background modes you can flip on/off in a sequence ---------- */

  public void enableBlink(Color color, double intervalSec) {
    mode = Mode.BLINK;
    blinkColor = color;
    blinkIntervalSec = Math.max(0.02, intervalSec);
    blinkOn = false;
    lastToggle = Timer.getFPGATimestamp();
    fillSolid(Color.kBlack);
    led.setData(buf);
  }

  public void disableBlink() {
    // keep whatever was last drawn, but set mode to SOLID
    mode = Mode.SOLID;
    // if you want it to revert to a specific color, uncomment:
    // setSolid(Color.kPurple);
  }

  public void enableRainbow(int speed, int delta) {
    mode = Mode.RAINBOW;
    rainbowSpeed = Math.max(1, speed);
    rainbowDelta = Math.max(1, delta);
  }

  /* -------------------- periodic engine -------------------- */

  @Override
  public void periodic() {
    switch (mode) {
      case SOLID:
        // nothing to do, already drawn
        break;

      case BLINK: {
        double now = Timer.getFPGATimestamp();
        if (now - lastToggle >= blinkIntervalSec) {
          blinkOn = !blinkOn;
          lastToggle = now;
          fillSolid(blinkOn ? blinkColor : Color.kBlack);
          led.setData(buf);
        }
        break;
      }

      case RAINBOW: {
        for (int i = 0; i < buf.getLength(); i++) {
          int hue = (rainbowHue + i * rainbowDelta) % 180;
          buf.setHSV(i, hue, 255, 128);
        }
        led.setData(buf);
        rainbowHue = (rainbowHue + rainbowSpeed) % 180;
        break;
      }
    }
  }

  /* -------------------- private helpers -------------------- */

  private void fillSolid(Color color) {
    for (int i = 0; i < buf.getLength(); i++) {
      buf.setLED(i, color);
    }
  }
}
