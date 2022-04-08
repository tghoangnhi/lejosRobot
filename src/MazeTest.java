
import lejos.hardware.Button; 
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

import java.util.*;


public class MazeTest {
	static MovePilot robotPilot;
	static RegulatedMotor frontNeckMotor = Motor.A;
	static EV3UltrasonicSensor distanceSensor;
	static EV3TouchSensor bumpSensor; 
	static EV3ColorSensor colorSensor;
	static SampleProvider bumpSampleProvider;
	static SampleProvider distanceSampleProvider;
	static SampleProvider colorSampleProvider;

	static long startTimeMillis;
	static char face = 'E';
	static int x = 0, y = 0;
	public static final HashMap<Character, Character> oppositeDir = new HashMap<Character, Character>() {{ put('N', 'S'); put('S', 'N'); put('W', 'E'); put('E', 'W'); }};

	public static void main(String[] args) {
		// Sets up pilot and sensors for robot
		// Note: If your robot *consistently* rotates too far / too little,
		//       you can try playing with the wheel diameter & offset values.
		//        Specifying too big of wheel diameter #s will make the robot under-rotate 
		//         whereas small wheel diameter #s will cause overturning.
		//        Different right/left wheel diameters will cause the robot to
		//        tend to arc slowly left or right while moving forward
		//        (The robots wheels should be symmetrical, or very close.)        
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.C, 5.5).offset(7.25);
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.B, 5.5).offset(-7.25);
		Chassis myChassis = new WheeledChassis( new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		robotPilot = new MovePilot(myChassis);
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S1);
		bumpSensor = new EV3TouchSensor(SensorPort.S2);
		colorSensor = new EV3ColorSensor(SensorPort.S4);
		distanceSampleProvider = distanceSensor.getDistanceMode();

		// feel free to play with these numbers...
		robotPilot.setLinearSpeed(30);
		robotPilot.setLinearAcceleration(60);
		robotPilot.setAngularSpeed(30);

		LCD.clear();
		LCD.drawString("ENTER to run!", 0,0);
		Button.waitForAnyPress();
		LCD.clear();
		
		run();
	}

	public static void run(){
		bumpSampleProvider = bumpSensor.getTouchMode();
		float[] bumpSample = new float[bumpSampleProvider.sampleSize()];

		bumpSampleProvider.fetchSample(bumpSample, 0);
		
		boolean[][] visited = new boolean[5][5];
		
		for (int i = 0; i < visited.length; i++) {
			for (int j = 0; j < visited[0].length; j++) {
				visited[i][j] = false;
			}
		}
		
		boolean solveMaze = helper(visited, x, y);
		
		//looking for white
//		while(colorSensor.getColorID() != Color.WHITE && !Button.ESCAPE.isDown()){
//			frontNeckMotor.rotate(-90);
//			float distRight = getDistanceMeasurement();
//			frontNeckMotor.rotate(90);
//			float distStraight = getDistanceMeasurement();
//			frontNeckMotor.rotate(90);
//			float distLeft = getDistanceMeasurement();
//			System.out.println("-------------");
//			System.out.printf("d-right: %.2f\n", distRight);
//			System.out.printf("d-straight: %.2f\n", distStraight);
//			System.out.printf("d-left: %.2f\n", distLeft);
//			frontNeckMotor.rotate(-90);
//
//			// let's say that more than 50 cm in front means open (if sensor isn't lying)
//			if (distStraight > 0.5) { 
//				robotPilot.travel(-40, true); // travel 40 CENTIMETERS forward.
//				// the "true" argument means that this call returns immediately (instead of 
//				// waiting/blocking), and the code below runs while the robot travels
//			} else {
//				robotPilot.rotate(-90); // turn right!
//			}
//			while (robotPilot.isMoving()) {
//				bumpSampleProvider.fetchSample(bumpSample, 0);
//				if(bumpSample[0] == 1){ // is the touch sensor currently pushed in?
//					robotPilot.stop();
//					robotPilot.travel(20);
//				}
//				if (colorSensor.getColorID() == Color.WHITE || Button.ESCAPE.isDown()){ 
//					break; // found the GOAL (or human aborted with ESC button)
//				}
//			}
//		}
		robotPilot.stop();
		Button.LEDPattern(4); // victory celebration!
		Sound.beepSequenceUp();
		Sound.beepSequence();
		Button.waitForAnyPress();
	}

	/** Convenience method to get ONE distance measurement from the robot.
	 * 
	 * @return the distance reading (in meters) in the direction the EV3's ultra-sonic sensor is currently facing.
	 */
	public static float getDistanceMeasurement(){
		float[] distanceSample = new float[distanceSampleProvider.sampleSize()];
		distanceSampleProvider.fetchSample(distanceSample, 0);

		return distanceSample[0];
	}
	
	public static float getBumpSample() {
		bumpSampleProvider = bumpSensor.getTouchMode();
		float[] bumpSample = new float[bumpSampleProvider.sampleSize()];
		
		bumpSampleProvider.fetchSample(bumpSample, 0);
		
		return bumpSample[0];
	}
	
	public static List<Character> getOpenDirs() {
		List<Character> openDirs = new ArrayList<>();
		
		if (face == 'N') {
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('N');
			}
			frontNeckMotor.rotate(-90);
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('W');
			}
			frontNeckMotor.rotate(180);
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('E');
			}
			frontNeckMotor.rotate(-90);
		}
		else if (face == 'E') {
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('E');
			}
			frontNeckMotor.rotate(-90);
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('N');
			}
			frontNeckMotor.rotate(180);
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('S');
			}
			frontNeckMotor.rotate(-90);
		}
		else if (face == 'S') {
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('S');
			}
			frontNeckMotor.rotate(-90);
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('E');
			}
			frontNeckMotor.rotate(180);
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('W');
			}
			frontNeckMotor.rotate(-90);
		}
		else if (face == 'W') {
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('W');
			}
			frontNeckMotor.rotate(-90);
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('S');
			}
			frontNeckMotor.rotate(180);
			if (getDistanceMeasurement() > 0.35) {
				openDirs.add('N');
			}
			frontNeckMotor.rotate(-90);
		}
		
		return openDirs;
	}

	public static boolean helper(boolean[][] visited, int x, int y) {
		
		System.out.printf("x = %d, y = %d", x, y);
		
		if (colorSensor.getColorID() == Color.BLACK) {
			System.out.println("FOUND GOAL!!!");
			return true;
		}
		
		System.out.println(visited[x][y]);
		if (visited[x][y])
			return false;
		
		visited[x][y] = true;
		List<Character> openDirs = getOpenDirs();
		
		for (char dir: openDirs)
		{
			System.out.println(dir);
			boolean move = moveBot(dir);
			if (move == true) {
				return true;
			}
			int[] temp = tempCoord(dir);
			if (helper(visited, temp[0], temp[1])) {
				return true;
			}
			moveBot(oppositeDir.get(dir));
		}
		
		visited[x][y] = false;
		return false;
		
	}
	
	public static int[] tempCoord(char dir) {
		if (dir == 'N') {
			y++;
		} else if (dir == 'S') {
			y--;
		} else if (dir == 'W') {
			x--;
		} else { // dir == 'E'
			x++;
		}
		return new int[] {x, y};
	}
	
	public static boolean moveBot(char dir) {
		if (face == 'N') {
			if (dir == 'E') {
				robotPilot.rotate(140);
			}
			else if (dir == 'W') {
				robotPilot.rotate(-140);
			}
			else if (dir == 'S') {
				robotPilot.rotate(270);
			}
		}
		else if (face == 'E') {
			if (dir == 'N') {
				robotPilot.rotate(-140);
			}
			else if (dir == 'S') {
				robotPilot.rotate(140);
			}
			else if (dir == 'W') {
				robotPilot.rotate(270);
			}
		}
		else if (face == 'S') {
			if (dir == 'W') {
				robotPilot.rotate(140);
			}
			else if (dir == 'E') {
				robotPilot.rotate(-140);
			}
			else if (dir == 'N') {
				robotPilot.rotate(270);
			}
		}
		else if (face == 'W') {
			if (dir == 'N') {
				robotPilot.rotate(140);
			}
			else if (dir == 'S') {
				robotPilot.rotate(-140);
			}
			else if (dir == 'E') {
				robotPilot.rotate(270);
			}
		}
		
		face = dir;
		robotPilot.travel(-40, true);
		
		while (robotPilot.isMoving()) {
			if (getBumpSample() == 1) {
				System.out.println("BUMPPPPPPPPP");
				robotPilot.stop();
				robotPilot.travel(10, true);
			}
			System.out.println(colorSensor.getColorID());
			if (colorSensor.getColorID() == Color.BLACK || Button.ESCAPE.isDown()){ 
				System.out.println("FOUND GOAL WHILE MOVING!!!");
				return true; // found the GOAL (or human aborted with ESC button)
			}
		}
		return false;	
	}
	

}
