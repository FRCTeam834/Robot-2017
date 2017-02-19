package robot;

import java.util.ArrayList;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;
import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
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
	 * 7 Fuel
	 */
	CANTalon[] motors = new CANTalon[8];
	
	Servo door = new Servo(0);
	
	Encoder lEncoder = new Encoder(0, 1);
	Encoder rEncoder = new Encoder(2, 3) {
		public double getDistance() {
			return 0;
			
		}
	};
	
	AnalogInput ultrasonic = new AnalogInput(0);
	
	ADXRS450_Gyro robotGyro = new ADXRS450_Gyro();
	
	Solenoid ringLED = new Solenoid(1,0);
	
	Joystick leftJoystick = new Joystick(0);
	Joystick rightJoystick = new Joystick(1);
	Joystick xbox = new Joystick(2);

	boolean LEDOn = false;
	boolean toggleLED = false;
	
	boolean straightEnabled = false;
	
	private VisionThread visionThread;
	private Object imgLock = new Object();;
	private double centerX;
	private double centerY;
	private double size;

	
	public void robotInit() {
		
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

	    
		lEncoder.setDistancePerPulse(6.0*Math.PI/256.0 * 2); //*2 is temporary since one encdoer.
//		rEncoder.setDistancePerPulse(-6.0*Math.PI/4.0);

		
		super.sensors.put("leftEncoder", lEncoder);
		super.sensors.put("rightEncoder", rEncoder);
		super.sensors.put("gyro", robotGyro);
		
		motors[0] = new CANTalon(5);
		motors[1] = new CANTalon(9);
		motors[2] = new CANTalon(8);
	
		motors[3] = new CANTalon(6);
		motors[4] = new CANTalon(3);
		motors[5] = new CANTalon(2);
		
		motors[6] = new CANTalon(7);
		motors[7] = new CANTalon(4);

		 
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	        if (!pipeline.filterContoursOutput().isEmpty()) {

	        	ArrayList<MatOfPoint> contours = pipeline.filterContoursOutput();
	            int size = contours.size();

            	int avgX = 0;
            	int avgY = 0;
            	if(size == 1) {
            		Rect r = Imgproc.boundingRect(contours.get(0));
            		avgX += r.x + r.width/2;
            		avgY += r.y + r.height/2;
            	}
            	else if(size == 2) {
	            	for(MatOfPoint c : contours) {
	            		Rect r = Imgproc.boundingRect(c);
	            		avgX += r.x + r.width/2;
	            		avgY += r.y + r.height/2;
	            	}
	            	avgX /= 2;
	            	avgY /= 2;
	            }
	            else if (size == 3) {
	            	
	            	double xs[] = new double[3];
	            	double ys[] = new double[3];
	            	for(int i = 0; i < 3; i++) {
	            		MatOfPoint c =  contours.get(i);
	            		Rect r = Imgproc.boundingRect(c);
	            		xs[i] = r.x + r.width/2;
	            		ys[i] = r.y + r.height/2;
	            	}
	            	double maxDiff = 0;
	            	double weightX = 0;
	            	double weightY = 0;
	            	
	            	for (int i = 0; i < 3; i++) {

	            		double otherX1 = xs[(i+1)%3];
	            		double otherX2 = xs[(i+2)%3];
	            		double distToOthers = Math.abs(xs[i] - otherX1) + Math.abs(xs[i] - otherX2);
	            		if(distToOthers > maxDiff) {
	            			maxDiff = distToOthers;
	            			weightX = xs[i];
	            			weightY = ys[i];
	            		}
	            		avgX += xs[i];
	            		avgY += ys[i];
	            	}
	            	
	            	avgX -= weightX;
	            	avgY -= weightY;

	            	avgX /= 2;
	            	avgY /= 2;
	            }
	            
	            synchronized (imgLock) {
	                centerX = avgX;
	                centerY = avgY;
	                this.size = size;

	            }
	        }
	    });
	    visionThread.start();



		
	}
	
	public void autonomous() {

		ringLED.set(true);

		while (this.isAutonomous() && !this.isDisabled()) {
			this.goTowardsPeg(.3, 24);
		}
//		ChooseAuton c = new ChooseAuton(this);
//		c.chooseAuton("fieldLap");
//
//		ArrayList<Command> t = c.cmdSet.getMain();
//		
//		MoveStraightCommand mvc = (MoveStraightCommand) t.remove(t.size()-1);
//		
////		c.run();
//		lEncoder.reset();	
//		
//		ringLED.set(true);
//
//		while (Math.abs(lEncoder.getDistance()) < mvc.distance-20 && this.isAutonomous()) {
//			goTowardsPeg(.2, mvc.distance-lEncoder.getDistance());
//		}
//		this.setRightSide(0.0);
//		this.setLeftSide(0.0);
//		
//		ringLED.set(false);
//		
//		MoveStraightCommand toPeg = new MoveStraightCommand(20, .3);
//		toPeg.setRobot(this);
//		toPeg.execute();
		


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

	}

	@Override
	public void teleOpPeriodic() {
		if(rightJoystick.getRawButton(1)) {
			if(!straightEnabled) {
				robotGyro.reset();
			}
			
			setRightSide(1.0);
			setLeftSide(1.0);
			Timer.delay(.05);
		}
		else if (leftJoystick.getRawButton(2)){
			this.goTowardsPeg(.2, 80);
		}
		else {	
			setRightSide(-rightJoystick.getY() * Math.abs(rightJoystick.getY()));
			setLeftSide(-leftJoystick.getY() * Math.abs(leftJoystick.getY()));
		}
		ringLED.set(LEDOn);
		
		if(xbox.getRawButton(8)) {
			if (!toggleLED){
				LEDOn = !LEDOn;
			}
			toggleLED = true;
		}
		else {
			toggleLED = false;
		}
		
		SmartDashboard.putString("DB/String 0", Double.toString(lEncoder.getDistance()));
//		SmartDashboard.putString("DB/String 1", Double.toString(rEncoder.getDistance()));

		SmartDashboard.putString("DB/String 2", Double.toString(ultrasonic.getVoltage()/(5.0/1024.0)*5.0/10.0  *2.54));
		


		if(xbox.getRawButton(1)) {
			motors[6].set(-1.0);
		}
		else if (xbox.getRawButton(2)) {
			motors[6].set(.3);
		}
		else {
			motors[6].set(0);
		}
		
		if(xbox.getRawButton(3)) {
			motors[7].set(.3);
		}
		else if(xbox.getRawButton(4)) {
			motors[7].set(-.3);
		}
		else {
			motors[7].set(0);
		}
		
		if(xbox.getRawButton(5)) {
			door.set(0);
		}
		else if(xbox.getRawButton(6)) {
			door.set(1);
		}
		

		
	}
	
	double diff;
	double lastDiff;
	double targetAngle = 0;
	public void goTowardsPeg(double speed, double distance) {
					
		double centerX, centerY, size;
		synchronized (imgLock) {
			centerX = this.centerX;
			centerY = this.centerY;
			size = this.size;
		}

		lastDiff = diff;
		diff = (centerX - 160) * 2*distance/320;
		
		
		if(lastDiff != diff) {
			targetAngle = Math.atan(diff/distance) * 180.0 / Math.PI;
			System.out.println("changing to " + targetAngle);
		}
		
		double lspeed = speed, rspeed = speed;
	
		if(robotGyro.getAngle() < targetAngle){
			rspeed -= speed * Math.abs(robotGyro.getAngle() - targetAngle)/50.0 ;
		}
		else if(robotGyro.getAngle() > targetAngle) {
			lspeed -= speed *  Math.abs(robotGyro.getAngle() - targetAngle)/50.0;
		}
	
		rspeed = rspeed < 0 ? 0 : rspeed;
		lspeed = lspeed < 0 ? 0 : lspeed;
		if(lastDiff != diff) 
			System.out.println(lspeed + " " + rspeed);

		this.setRightSide(rspeed);
		this.setLeftSide(lspeed);
		Timer.delay(.05);
		

	}
}