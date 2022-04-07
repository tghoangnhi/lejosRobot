
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.utility.Delay;

public class HelloGraphics
{
    public static void main(String[] args)
    {
        GraphicsLCD lcd = BrickFinder.getDefault().getGraphicsLCD();
        final int screenWidth = lcd.getWidth();
        final int screenHeight = lcd.getHeight();
        Button.LEDPattern(4);
        Sound.beepSequenceUp();
        
        lcd.setFont(Font.getLargeFont());
        lcd.drawString("I'm Big!", screenWidth/2, screenHeight/2, 
        				GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
        lcd.drawLine(0, screenHeight/2+10, screenWidth, screenHeight/2-10);
        Button.LEDPattern(3);
        Delay.msDelay(4000);
        Button.LEDPattern(5);
        lcd.clear();
        lcd.refresh();
        Sound.beepSequence();
        Delay.msDelay(500);
        Button.LEDPattern(0);
    }
}
