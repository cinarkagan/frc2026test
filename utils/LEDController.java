package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDController {


    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    public LEDController(int port, int length) {
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);

        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    public void setAll(int r, int g, int b) { //sets all leds to the given rgb color
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }
    public void setOneLed(double percentage, int r, int g, int b) { //sets only one led in the specified percentage spot on the strip
        int i = (int)Math.round(percentage*buffer.getLength());
        buffer.setRGB(i,r,g,b);
    }
    public void setOneLedIndex(int index, int r, int g, int b){ //sets only one led in the specified index
        buffer.setRGB(index,r,g,b);
    }
    public void setMultipleFromZero(double percentage, int r, int g, int b) { //sets a range of rgb leds to the specified color starting from the first led to the given percentage
        int max = (int)Math.round(percentage*buffer.getLength());
        for (int i = 0; i < max; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }
    public void setMultipleWithRange(double pmin,double pmax, int r, int g, int b) {  //sets a range of rgb leds to the specified color starting from the minimum given percentage to the maximum given percentage
        int max = (int)Math.round(pmax*buffer.getLength());
        int min = (int)Math.round(pmin*buffer.getLength());
        for (int i = min; i < max; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }
    public void zeroAll() { //makes all the leds in the strip 0, or not powered
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0,0);
        }
    }
    public void pushData() { //pushes the led data to the strip, also resets the buffer so this should only be called afer the entirety of the operations meant for a action are done
        leds.setData(buffer); 
    }
}
