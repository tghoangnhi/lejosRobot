

import java.util.Random;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.MovePilot;
import lejos.hardware.sensor.EV3TouchSensor;

public class BumpTest {

	static MovePilot robot;
	static RegulatedMotor leftMotor = Motor.C;
	static RegulatedMotor rightMotor = Motor.B;
	static EV3TouchSensor bumpSensor;
	
	public static void main(String[] args){
		Wheel leftWheel = WheeledChassis.modelWheel(leftMotor, 5.65).offset(7.25);
		Wheel rightWheel = WheeledChassis.modelWheel(rightMotor, 5.65).offset(-7.25);
		Chassis myChassis = new WheeledChassis( new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		robot= new MovePilot(myChassis);
		robot.setLinearSpeed(30);  // these are customizable, and choices
		robot.setAngularSpeed(60); // may lead to more/less accurate moving/turning? 
		
		bumpSensor = new EV3TouchSensor(SensorPort.S2);
		
		run();
	}
	
	public static void run(){
		SampleProvider bumpSampleProvider = bumpSensor.getTouchMode();
		float[] bumpSensorData = new float[bumpSampleProvider.sampleSize()];
		bumpSampleProvider.fetchSample(bumpSensorData, 0);
		
		robot.backward(); // moves robot forward until robot.stop() is called
		
		while(!Button.ESCAPE.isDown()){
			if(bumpSensorData[0] == 1){
				robot.stop();
				robot.travel(-10); // back up
				robot.rotate(90); // in degrees
				robot.backward();
			}
			bumpSampleProvider.fetchSample(bumpSensorData, 0);
		}
	}

}
