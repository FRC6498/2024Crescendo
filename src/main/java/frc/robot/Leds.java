package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  AddressableLED leds;
  AddressableLEDBuffer ledBuffer;
  double phase, ledSpeed;
  public Leds() {
    leds = new AddressableLED(0);
    leds.setLength(Constants.LedConstants.LED_LENGTH);
    ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.LED_LENGTH);
    leds.start();
  }
  public Command StopLeds() {
    return this.runOnce(()->leds.stop());
  }
  public Command StartLeds() {
    return this.runOnce(()->leds.start());
  }
  @Override
  public void periodic() {
    phase += ledSpeed;
    if (phase >= Math.PI*2){
      phase -= Math.PI*2;
    }
    for (int i = 0; i < 14; i++) {
      double iphase = i*Math.PI/7;
      ledBuffer.setRGB(i, (int)Math.round(128*Math.sin(phase + iphase)), 0, (int)Math.round(128*Math.sin(phase+Math.PI + iphase)));
    }
    leds.setData(ledBuffer);
  }
}
