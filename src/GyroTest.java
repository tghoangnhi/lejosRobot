


import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3GyroSensor;

/**
 * For this demo to work, you'll need to install a 
 * Gyro Sensor on your robot, attached to sensor port S2.
 * 
 * For more info about Gyro Sensor, see: 
 *   http://www.lejos.org/ev3/docs/lejos/hardware/sensor/EV3GyroSensor.html
 * 
 * The Gyro Sensor might (or might not) be helpful in correcting
 * your robot's motion if (when) it doesn't move/turn accurately enough.  
 * 
 * (Alternatively, you can try to use the distance sensor repeatedly 
 * measuring wall distances to the side of the robot, or some other technique,
 * to try to help keep the robot more centered in each grid cell.)
 *
 */
public class GyroTest {

	static EV3GyroSensor gyroSensor;
	static SampleProvider gyroSampleProvider;
	
	public static void main(String[] args){
		gyroSensor = new EV3GyroSensor(SensorPort.S3);
		gyroSampleProvider = gyroSensor.getAngleMode();
		
		System.out.println("Twist robot to");
		System.out.println(" see gyro values");
		System.out.println("ENTER btn resets");
		Button.waitForAnyPress();
		run();
	}
	
	public static void run() {
		while(Button.ESCAPE.isUp()) {
			System.out.println(getGyroMeasurement());
			if (Button.ENTER.isDown()) {
				System.out.println("GYRO RESET");
				gyroSensor.reset(); 
			}
	        Delay.msDelay(500);			
		}
	}
		
	/**
	 * @return the 
	 */
	public static float getGyroMeasurement(){
		float[] gyroSample = new float[gyroSampleProvider.sampleSize()];
		gyroSampleProvider.fetchSample(gyroSample, 0);
		
		return gyroSample[0];
	}

}
