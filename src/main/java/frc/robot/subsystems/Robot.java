package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Robot extends TimedRobot {
  private AddressableLED led;
  private AddressableLEDBuffer buf;
  private int hue = 0;

  @Override
  public void robotInit() {
    led = new AddressableLED(0);          // try 1 if you move wires
    buf = new AddressableLEDBuffer(28);   // your count
    led.setLength(buf.getLength());

    // solid purple on boot
    for (int i = 0; i < buf.getLength(); i++) buf.setRGB(i, 128, 0, 128);
    led.setData(buf);
    led.start();
  }

  @Override
  public void robotPeriodic() {
    // simple rainbow to prove updates
    for (int i = 0; i < buf.getLength(); i++) {
      int h = (hue + i * 3) % 180;
      buf.setHSV(i, h, 255, 128);
    }
    led.setData(buf);
    hue = (hue + 1) % 180;
  }
}
