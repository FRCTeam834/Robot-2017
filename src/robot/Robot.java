package robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Scanner;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;
import com.ctre.CANTalon;

import basicCommand.DelayCommand;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import visualrobot.ChooseAuton;
import visualrobot.Command;
import visualrobot.MoveStraightCommand;
import visualrobot.VisualRobot;
import edu.wpi.first.wpilibj.Solenoid;

public class Robot extends VisualRobot {
	

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;

	
	/*
	 * 0-2 Right Drive
	 * 3-5 Left Drive
	 * 6 Lift
	 * 7 Fuel: Encoder Value 0- 70000
	 */
	CANTalon[] motors = new CANTalon[8];
	
	Servo door = new Servo(0);
	
	DigitalInput gearSensor = new DigitalInput(4);
	
	Encoder lEncoder = new Encoder(0, 1) {
		public double getDistance() {
			return 0;
			
		}		
	};
			
	Encoder rEncoder = new Encoder(2, 3);
	
	Encoder fuelEncoder = new Encoder(8, 9) {
		public double getDistance() {
			return (double)motors[7].getEncPosition();
		}
		public void reset() {
			motors[7].setEncPosition(0);
		}
	};

	
	ADXRS450_Gyro robotGyro = new ADXRS450_Gyro();
	AnalogGyro gyro = new AnalogGyro(1);

	Solenoid ringLED = new Solenoid(1,4);
	Solenoid frontLED = new Solenoid(1,1);
	Solenoid rightLED = new Solenoid(1,2);
	Solenoid leftLED = new Solenoid(1,5);
	Solenoid backLED = new Solenoid(1,0);
	
	Joystick leftJoystick = new Joystick(0);
	Joystick rightJoystick = new Joystick(1);
	Joystick xbox = new Joystick(2);

	Toggler LED = new Toggler(xbox, 8); //Start button
	Toggler fieldCentric = new Toggler(rightJoystick, 2);
	Toggler alternateDrive = new Toggler(rightJoystick, 3);

	int fieldCentricStage = 0;	
	double turnAngle = 0;
	double dAngle = 0;
	
	int bShakeCount = 0;
	int LEDFlashCount = 0;
	
	Toggler autoClimbing = new Toggler(xbox, 7);
	
	Toggler manual = new Toggler(xbox, 5);
	
	final double CURRENT_THRESHOLD = 20.0;
	final double TIME_TO_CLIMB = 2.25;
	
	private VisionThread visionThread;
	private Object imgLock = new Object();;
	private double centerX;
	private double size;

	ArrayList<ArrayList<Double>> data = new ArrayList<ArrayList<Double>>();
	
	public void robotInit() {
		
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    
//		lEncoder.setDistancePerPulse(6.0*Math.PI/256.0*2.0); //*2 is temporary since one encdoer.
		rEncoder.setDistancePerPulse(-6.0*Math.PI/256.0*2.0);
		
		super.sensors.put("leftEncoder", lEncoder);
		super.sensors.put("rightEncoder", rEncoder);
		super.sensors.put("gyro", robotGyro);
		super.sensors.put("fuelEncoder", fuelEncoder);
		super.sensors.put("gear", gearSensor);
		
		robotGyro.calibrate();
		gyro.calibrate();
		gyro.reset();
		
		motors[0] = new CANTalon(5);
		motors[1] = new CANTalon(9);
		motors[2] = new CANTalon(8);
	
		motors[3] = new CANTalon(6);
		motors[4] = new CANTalon(3);
		motors[5] = new CANTalon(2);
		
		motors[6] = new CANTalon(7);
		motors[7] = new CANTalon(4);

		super.motors.put("fuel", motors[7]);
		
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()) {

	        	ArrayList<MatOfPoint> contours = pipeline.filterContoursOutput();
	        	int[] centers = new int[contours.size()];
	        	
	        	for(int i = 0; i < contours.size(); i++) {
	        		Rect r = Imgproc.boundingRect(contours.get(i));
	        		centers[i] = r.x + r.width/2 ;      		
	        	}
	        
	        	for (int i = 0; i < contours.size(); i++) {
        			int minj = 0;
        			int min = -1;

	        		for (int j = i+1; j < contours.size(); j++) {
	        			if(min == -1 || Math.abs(centers[j] - 160) < min) {
	        				minj = j;
		        			min = Math.abs(centers[j] - 160);
	        			}
	        		}
	        		
	        		MatOfPoint temp = contours.get(minj);
	        		contours.set(minj, contours.get(i));
	        		centers[minj] = centers[i];
	        		contours.set(i, temp);
	        		centers[i] = min;
	        	}
	        	
	        	int result = 0;
	        	if(centers.length >=2) {
	        		result = (centers[0] + centers[1]) /2;
	        	}
	        	else{
	        		result = -1;
	        	}
	            synchronized (imgLock) {
	                centerX = result;
	                this.size = contours.size();

	            }
	        }
	    });
	    visionThread.start();



		
	}
	
	public void autonomous() {
		try {
			String program = SmartDashboard.getString("DB/String 0", "default");
			boolean vision = SmartDashboard.getString("DB/String 1", "").equals( "enablevision");
			boolean recording = SmartDashboard.getString("DB/String 1", "").equals( "userecording");
			boolean flop = SmartDashboard.getString("DB/String 2", "").equals( "floppy");
	
			if(recording) {
				useRecording();
				return;
			}
			
	
			ChooseAuton c = new ChooseAuton(this);
			c.chooseAuton(program);
			ArrayList<Command> t = c.cmdSet.getMain();
	
			for(Command com : t) {
				if(com.getClass().equals(visualrobot.MoveStraightCommand.class)) {
					((MoveStraightCommand) com).distance -=  6;
				}
			}
			
			
			if(program.contains("Goal"))		
				door.setAngle(0);
	
			
			MoveStraightCommand mvc = null;
			if(vision) {
				ringLED.set(true);
				if(program.contains("Center")) {
					mvc = (MoveStraightCommand) t.remove(1);
				}

			}
			else {
				if(program.contains("Center")) {
					t.set(1, new TimedStraightCommand((MoveStraightCommand) t.get(1), 3));
					t.get(1).setRobot(this);
				}
	
			}
			
			if(flop) {
				MoveStraightCommand floppy1 =  new MoveStraightCommand(10, .6);
				MoveStraightCommand floppy2 =  new MoveStraightCommand(5, -.6);
	
				floppy1.setRobot(this);
				floppy2.setRobot(this);
	
				DelayCommand delay = new DelayCommand(0.5);
				delay.setRobot(this);
				
				floppy1.execute();
				floppy2.execute();
				delay.execute(); 
			}
			if(vision) {
				
				c.run();
				
				lEncoder.reset();	
				
				
				ringLED.set(true);
				robotGyro.reset();
				while (Math.abs(lEncoder.getDistance()) < mvc.distance && this.isAutonomous()) {
					goTowardsPeg(.2, mvc.distance-(rEncoder.getDistance() + lEncoder.getDistance())+10);
				}
				this.setRightSide(0.0);
				this.setLeftSide(0.0);
				
				ringLED.set(false);
				
				MoveStraightCommand toPeg = new MoveStraightCommand(10, .3);
				toPeg.setRobot(this);
				toPeg.execute();
			}
			else {
				c.run();
			}
		}
		catch(Exception e) {
			System.out.println(e.getMessage());
		}

	}

	public void setLeftSide(double speed) {
		speed = speed > 1 ? 1 : speed;
		speed = speed < -1 ? -1 : speed;
		
		for(int i = 3; i <= 5; i++) {
			motors[i].set(-speed);
		}
	}

	public void setRightSide(double speed) {
		speed = speed > 1 ? 1 : speed;
		speed = speed < -1 ? -1 : speed;
		
		for(int i = 0; i <= 2; i++) {
			motors[i].set(speed);
		}
	}

	public void teleOpInit() {
		if(SmartDashboard.getString("DB/String 1", "").equals("Record")) {
			data.add(new ArrayList<Double>());
			data.add(new ArrayList<Double>());
			data.add(new ArrayList<Double>());		
			
			while(this.isEnabled() && this.isOperatorControl()) {
				
				long startTime = System.currentTimeMillis();
				double right = -rightJoystick.getY();
				double left = -leftJoystick.getY();
				setRightSide(right);
				setLeftSide(left); 
				data.get(0).add(left);
				data.get(1).add(right);
				Timer.delay(.05);
				data.get(2).add((double) (System.currentTimeMillis()-startTime));
				

			}
		}
	}

	public void disabled() {
		if(!data.isEmpty())
			saveRecording();
	}
	
	public void saveRecording() {
		PrintWriter pw;
		try {
			pw = new PrintWriter(new BufferedWriter(new FileWriter("/home/lvuser/recorded.txt")));
			for(int i = 0; i < data.get(0).size(); i++) {
				pw.println(data.get(0).get(i) + " " + data.get(1).get(i) + " " + data.get(2).get(i));
			}
			pw.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		data.clear();
	}
	public void useRecording() {
		try {
			Scanner in = new Scanner(new File("/home/lvuser/recorded.txt"));
			
			ArrayList<Double> lSpeeds = new ArrayList<Double>();
			ArrayList<Double> rSpeeds = new ArrayList<Double>();
			ArrayList<Double> times = new ArrayList<Double>();

			while(in.hasNextLine()) {
				String[] input =  in.nextLine().split(" ");
				lSpeeds.add(Double.parseDouble(input[0]));
				rSpeeds.add(Double.parseDouble(input[1]));
				times.add(Double.parseDouble(input[2]));
			}
			
			int i = 0;
			while( i < lSpeeds.size() && this.isEnabled()) {
				this.setLeftSide(lSpeeds.get(i));
				this.setRightSide(rSpeeds.get(i));
				Timer.delay(times.get(i)/1000.0);
				i++;
			}
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
	
	@Override
	public void teleOpPeriodic() {

		manual.check();
//		alternateDrive.check();
//		
//		
//		if(xbox.getRawButton(6)) {
//			fuelEncoder.reset();
//		}
//		
//		if(leftJoystick.getRawButton(2)){
//			this.turnTowardsPeg(.2, 70);
//		}
		
		if(rightJoystick.getRawButton(1)) {
			setRightSide(1.0 * Math.signum(-rightJoystick.getY()));
			setLeftSide(1.0 * Math.signum(-rightJoystick.getY()));
		}
		else {				
//			if(fieldCentric.enabled) {
//				if(fieldCentricStage == 0) {
//					
//						fieldCentricStage = 1;
//					
//				}
//				else if(fieldCentricStage == 1) {
//					double lspeed = -rightJoystick.getY(); //* Math.abs(leftJoystick.getY());
//					double rspeed = -rightJoystick.getY();
//					
//					double turnAngle = targetAngle % 360;
//					if(turnAngle < 0) 
//						turnAngle+=360;
//						
//					if(turnAngle >= 90) {
//						lspeed *= -1;
//						rspeed *= -1;
//					}
//					if(gyro.getAngle() - targetAngle < 0){
//						rspeed -= Math.abs((gyro.getAngle()- targetAngle) * rspeed/25.0);
//					}
//					else if(gyro.getAngle()-targetAngle > 0) {
//						lspeed -= Math.abs((gyro.getAngle()- targetAngle) * lspeed/25.0);
//					}
//					this.setLeftSide(lspeed);
//					this.setRightSide(rspeed);
//
//				}
//			}
//			else if (alternateDrive.enabled) {
//				double magnitude = -leftJoystick.getY();
//				double rotation = -.8 * rightJoystick.getX()* Math.abs(rightJoystick.getX());
//				
//				setLeftSide(magnitude - rotation);
//				setRightSide(magnitude +  rotation);
//			}
//			else{
				setRightSide(-rightJoystick.getY());
				setLeftSide(-leftJoystick.getY()); 

//			}

		}
		
//		if(fieldCentric.getButton() && !fieldCentric.toggle ) {
//			 dAngle = (gyro.getAngle() % 180);
//			 if(dAngle < 0)
//				 dAngle += 180;
//			 System.out.println(dAngle);
//			 dAngle =  Math.abs(dAngle) > 90 ? -(180 - dAngle) : dAngle;
//			 turnAngle = gyro.getAngle() - dAngle;
//			 fieldCentricStage = 0;
//		}
//		fieldCentric.check();		

		//Control structure for LED toggle (start button)
		ringLED.set(LED.enabled);
		LED.check();
		
		if(gearSensor.get()) {
			this.setLEDs(false);
		}
		else {
			this.setLEDs(true);
		}
		
		//Various outputs
		SmartDashboard.putString("DB/String 3", manual.enabled ? "manual enabled" : "manual disabled"); 
//		SmartDashboard.putString("DB/String 4", "Field Centric: " + Boolean.toString(fieldCentric.enabled) + " " + turnAngle);
//		SmartDashboard.putString("DB/String 5", !gearSensor.get()? "IN IN IN IN" : "no gear");
		SmartDashboard.putString("DB/String 6", Double.toString(lEncoder.getDistance()));
		SmartDashboard.putString("DB/String 7", Double.toString(rEncoder.getDistance()));
//		SmartDashboard.putString("DB/String 8", Double.toString(fuelEncoder.getDistance()));
//		SmartDashboard.putString("DB/String 9", Double.toString(gyro.getAngle()));

		//Auto Climbing
//		if(autoClimbing.getButton() && !autoClimbing.toggle ) {
//			if(autoClimbing.enabled ){
//				motors[6].set(1.0);
//				Timer.delay(.5);
//				while(autoClimbing.enabled && motors[6].getOutputCurrent() < this.CURRENT_THRESHOLD) {}
//				System.out.println("Current Threshold Reached");
//				double startTime = Timer.getFPGATimestamp();
//				while(autoClimbing.enabled && motors[6].getOutputCurrent() > this.CURRENT_THRESHOLD && Timer.getFPGATimestamp()-startTime < this.TIME_TO_CLIMB) {}
//				motors[6].set(0.0);
//				autoClimbing.enabled = false;
//			}
//
//		}
//		autoClimbing.check();

				
		//Manual Control of Lift
		if(xbox.getRawButton(1)) {
			this.setLEDs(false);
			motors[6].set(1.0);
		}
		else if (xbox.getRawButton(2) && manual.enabled) {
			motors[6].set(-.3);
		}
		else if(!autoClimbing.enabled){
			motors[6].set(0);
		}

		//Control of balls and door
		if(xbox.getRawButton(4)){
			if(manual.enabled || fuelEncoder.getDistance() < 70000) {
				motors[7].set(.8);
			}
			else {
				bShakeCount = bShakeCount > 100 ? 0 : bShakeCount + 1;
				if(bShakeCount > 50 && fuelEncoder.getDistance() < 70000) {
					motors[7].set(.4);
				}
				else {
					motors[7].set(-.4);
				}
				
			}
			door.setAngle(0);
		}
		else if(xbox.getRawButton(3)){// && (manual.enabled || fuelEncoder.getDistance() > 0)) {
			motors[7].set(-.3);
			door.setAngle(180);

		}
		else  {
			motors[7].set(0);
		}

		
		
		Timer.delay(.05);

	}
	
	double diff;
	double lastDiff;
	double targetAngle = 0;

	public void goTowardsPeg(double speed, double distance) {
					
		double centerX, size;
		synchronized (imgLock) {
			centerX = this.centerX;
			size = this.size;
		}

		double lspeed = speed, rspeed = speed;
		if(centerX != -1) {
			lastDiff = diff;
			diff = (centerX - 160) * 2*distance/320;
			
			if(lastDiff != diff) {
				targetAngle = Math.atan(diff/distance) * 180.0 / Math.PI;
				gyro.reset();
				if(distance < 15) 
					targetAngle = 0;
				if(centerX == 0)
					targetAngle = 0;
				System.out.println("changing to " + (robotGyro.getAngle()- targetAngle) + " (" + centerX + ")");
			}
			

			if(targetAngle != 0) {
				if(robotGyro.getAngle() < targetAngle){
					rspeed -= speed * Math.abs(robotGyro.getAngle() - targetAngle)/25.0 ;
				}
				else if(robotGyro.getAngle() > targetAngle) {
					lspeed -= speed *  Math.abs(robotGyro.getAngle() - targetAngle)/25.0;
				}
			}

		}
	
		rspeed = rspeed < 0 ? 0 : rspeed;
		lspeed = lspeed < 0 ? 0 : lspeed;
		if(lastDiff != diff) 
			System.out.println(lspeed + " " + rspeed);

		this.setRightSide(rspeed);
		this.setLeftSide(lspeed);
		Timer.delay(.05);
		

	}
	
	public void turnTowardsPeg(double speed, double distance) {
		
		double centerX, size;
		synchronized (imgLock) {
			centerX = this.centerX;
			size = this.size;
		}

		double lspeed = speed, rspeed = speed;
		if(centerX != -1) {
			lastDiff = diff;
			diff = (centerX - 160) * 2*distance/320;
			
			if(lastDiff != diff) {
				targetAngle = Math.atan(diff/distance) * 180.0 / Math.PI;
				gyro.reset();
				if(distance < 15) 
					targetAngle = 0;
				if(centerX == 0)
					targetAngle = 0;
				System.out.println("changing to " + (robotGyro.getAngle()- targetAngle) + " (" + centerX + ")");
			}
			

			while(Math.abs(gyro.getAngle()-targetAngle) < 2 && this.isEnabled()){
				setRightSide(Math.signum(gyro.getAngle()-targetAngle) * speed);
				setLeftSide(-Math.signum(gyro.getAngle()-targetAngle) * speed);

			}

		}	

	}
	
	public void setLEDs(boolean on) {
		frontLED.set(on);
		backLED.set(on);
		rightLED.set(on);
		leftLED.set(on);

	}
	
	public static class Toggler {
		public boolean enabled;
		public boolean toggle;
		private Joystick joystick;
		private int button;
		
		public Toggler(Joystick j, int b) {
			joystick = j;
			button = b;
		}
		
		public void check() {
			if(getButton()) {
				if(!toggle) {
					enabled = !enabled;
				}
				toggle = true;
			}
			else {
				toggle = false;
			}
		} 
		
		public boolean getButton() {
			return joystick.getRawButton(button);
		}
	}
}