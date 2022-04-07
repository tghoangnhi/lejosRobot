

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ColorTest {
	
	public static String getColorName(int colorId) {
		// LeJOS treats colors as integer codes
		switch (colorId) {
			case Color.NONE: return "NONE";
			case Color.RED: return "RED";
			case Color.GREEN: return "GREEN";
			case Color.BLUE: return "BLUE";
			case Color.YELLOW: return "YELLOW";
			case Color.MAGENTA: return "MAGENTA";
			case Color.ORANGE: return "ORANGE";
			case Color.WHITE: return "WHITE";
			case Color.BLACK: return "BLACK";
			case Color.PINK: return "PINK";
			case Color.GRAY: return "GRAY";
			case Color.LIGHT_GRAY: return "LIGHT_GRAY";
			case Color.DARK_GRAY: return "DARK_GRAY";
			case Color.BROWN: return "BROWN";
		}
		return "OTHER?";
	}
	public static void main(String[] args) {
		EV3ColorSensor sensor = new EV3ColorSensor(SensorPort.S4);
		
		while(!Button.ESCAPE.isDown()){
			LCD.clear();
			int colorId = sensor.getColorID();
			String colorName = getColorName(colorId);
			System.out.println("Color Id: " + colorName);
			Delay.msDelay(500); // 1/2 second
		}
	}

}
