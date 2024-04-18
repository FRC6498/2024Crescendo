package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  // LEDS
  // ====================
  // symmetrical on robot
  // 33 per side panel
  // 14 per eye
  // on DIO port 0
  // ====================

  // declares a new Object to control LEDs
  private final AddressableLED leds;
  // declares a buffer to represent the data that should be written to the leds
  private final AddressableLEDBuffer ledBuffer;

  private double phase; // controls the color shift in the rainbow
  private double ledSpeed; // speed the LEDs should cycle at while in rainbow mode 

  public Leds() {
    leds = new AddressableLED(0); // instantiates the LED object
    leds.setLength(Constants.LedConstants.LED_LENGTH);  // sets the length of the led strip 
    ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.LED_LENGTH); // sets the number of leds that should be controlled by the buffer
    leds.setData(ledBuffer); // writes data from the buffer to the LEDS
    leds.start(); // runs the leds with the data from the buffer constantly
  }
  /**
   * Stops the leds
   * @return 
   * Command to Stop the Leds
   */
  public Command StopLeds() {
    return this.runOnce(()->leds.stop());
  }
  /**
   * Starts the Leds
   * @return 
   * Command to Start the leds
   */
  public Command StartLeds() {
    return this.runOnce(()->leds.start());
  }

  // this method runs once every robot loop so things in here will be repeated
  @Override  
  public void periodic() {
    // creates a rainbow effect on the leds using sin curves to control the rgb values

    phase += ledSpeed; // increment the phase based on the speed of the leds
    if (phase >= Math.PI*2){
      phase -= Math.PI*2; // 2 pi rads = 0 rads
                          // avoids large numbers 
    }

    //loop over all leds in the buffer and write data to them
    for (int i = 0; i < 14; i++) {
      double iphase = i*Math.PI/7; // constant was determined experimentally
      ledBuffer.setRGB(i /* led to write data to */, (int)Math.round(128*Math.sin(phase + iphase)) /* red value */, 0 /* blue value */, (int)Math.round(128*Math.sin(phase+Math.PI + iphase) /* green value */));
    }
    leds.setData(ledBuffer);
  }
}
