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
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
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
	 * 7 Fuel
	 */
	CANTalon[] motors = new CANTalon[8];
	
	Servo door = new Servo(0);
	
	DigitalInput gearSensor = new DigitalInput(4);
	
	Encoder lEncoder = new Encoder(0, 1);
	Encoder rEncoder = new Encoder(2, 3) {
		public double getDistance() {
			return 0;
			
		}
	};
	
	private static class AUltrasonic extends AnalogInput {
		
		private final double VOLTAGE = 5.0;
	
		public AUltrasonic(int channel) {
			super(channel);		
		}

		public double getDistance() {
			return this.getVoltage()/(VOLTAGE/1024.0)*5.0*100/2.54/21000*24;
		}
	}
	AUltrasonic ultrasonic = new AUltrasonic(0);


	
	ADXRS450_Gyro robotGyro = new ADXRS450_Gyro();
	AnalogGyro gyro = new AnalogGyro(1);

	Solenoid ringLED = new Solenoid(1,0);
	
	Joystick leftJoystick = new Joystick(0);
	Joystick rightJoystick = new Joystick(1);
	Joystick xbox = new Joystick(2);

	Toggler LED = new Toggler(xbox, 8); //Start button
	Toggler fieldCentric = new Toggler(rightJoystick, 2);
	int fieldCentricStage = 0;	
	double turnAngle = 0;
	double dAngle = 0;
	
	Toggler autoClimbing = new Toggler(xbox, 7);
	
	final double CURRENT_THRESHOLD = 20.0;
	final double TIME_TO_CLIMB = 2.25;
	
	private VisionThread visionThread;
	private Object imgLock = new Object();;
	private double centerX;
	private double size;

	
	public void robotInit() {
		
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

	    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    
		lEncoder.setDistancePerPulse(6.0*Math.PI/256.0 * 2); //*2 is temporary since one encdoer.
//		rEncoder.setDistancePerPulse(-6.0*Math.PI/4.0);
		
		super.sensors.put("leftEncoder", lEncoder);
		super.sensors.put("rightEncoder", rEncoder);
		super.sensors.put("gyro", robotGyro);
		
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
	        			if(minj == -1 || Math.abs(centers[j] - 160) < min) {
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
	        		result = 0;
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

		ringLED.set(true);

		ChooseAuton c = new ChooseAuton(this);
		c.chooseAuton("fieldLap");

		ArrayList<Command> t = c.cmdSet.getMain();
		
		MoveStraightCommand mvc = (MoveStraightCommand) t.remove(t.size()-1);
		
		c.run();
		
		lEncoder.reset();	
		
		ringLED.set(true);
		robotGyro.reset();
		while (Math.abs(lEncoder.getDistance()) < mvc.distance && this.isAutonomous()) {
			goTowardsPeg(.2, mvc.distance-lEncoder.getDistance());
		}
		this.setRightSide(0.0);
		this.setLeftSide(0.0);
		
		ringLED.set(false);
		
		MoveStraightCommand toPeg = new MoveStraightCommand(20, .3);
		toPeg.setRobot(this);
		toPeg.execute();
		


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
			setRightSide(1.0 * Math.signum(-rightJoystick.getY()));
			setLeftSide(1.0 * Math.signum(-rightJoystick.getY()));
		}
		else if (leftJoystick.getRawButton(1)){
			this.goTowardsPeg(.2, ultrasonic.getDistance());
		}
		else {				
			if(fieldCentric.enabled) {
				if(fieldCentricStage == 0) {
					
					if (Math.signum(dAngle) * gyro.getAngle() <=  Math.signum(dAngle) *turnAngle) {
						fieldCentricStage = 1;

					}
				}
				else if(fieldCentricStage == 1) {
					double lspeed = -rightJoystick.getY(); //* Math.abs(leftJoystick.getY());
					double rspeed = -rightJoystick.getY();
					
					if(targetAngle % 360 >= 90) {
						lspeed *= -1;
						rspeed *= -1;
					}
					if(gyro.getAngle() - targetAngle < 0){
						rspeed -= Math.abs(gyro.getAngle() * rspeed/25.0);
					}
					else if(gyro.getAngle() > 0) {
						lspeed -= Math.abs(gyro.getAngle() * lspeed/25.0);
					}
					this.setLeftSide(lspeed);
					this.setRightSide(rspeed);

				}
			}
			else if(!fieldCentric.enabled){
				setRightSide(-rightJoystick.getY());
				setLeftSide(-leftJoystick.getY()); 

			}

		}
		
		fieldCentric.check();		
		if(fieldCentric.getButton() && !fieldCentric.toggle ) {
			 dAngle = (gyro.getAngle() % 180);
			 dAngle =  Math.abs(dAngle) > 90 ? -(180 - dAngle) : dAngle;
			 turnAngle = gyro.getAngle() - dAngle;

		}
		//Control structure for LED toggle (start button)
		ringLED.set(LED.enabled);
		LED.check();
		
		//Various outputs
		SmartDashboard.putString("DB/String 0", Double.toString(lEncoder.getDistance()));
//		SmartDashboard.putString("DB/String 1", Double.toString(rEncoder.getDistance()));
		SmartDashboard.putString("DB/String 2", Double.toString(ultrasonic.getDistance()));		
		SmartDashboard.putString("DB/String 3", Double.toString(motors[6].getOutputCurrent()));
		SmartDashboard.putString("DB/String 4", Boolean.toString(fieldCentric.enabled) +" " + this.fieldCentricStage);
		SmartDashboard.putString("DB/String 5", Boolean.toString(gearSensor.get()));

		//Auto Climbing
		if(autoClimbing.getButton() && !autoClimbing.toggle ) {
			if(autoClimbing.enabled ){
				motors[6].set(-1.0);
				Timer.delay(.5);
				while(autoClimbing.enabled && motors[6].getOutputCurrent() < this.CURRENT_THRESHOLD) {}
				System.out.println("Current Threshold Reached");
				double startTime = Timer.getFPGATimestamp();
				while(autoClimbing.enabled && motors[6].getOutputCurrent() > this.CURRENT_THRESHOLD && Timer.getFPGATimestamp()-startTime < this.TIME_TO_CLIMB) {}
				motors[6].set(0.0);
				autoClimbing.enabled = false;
			}

		}
		autoClimbing.check();

				
		//Manual Control of Lift
		if(xbox.getRawButton(1)) {
			
			motors[6].set(-1.0);
		}
		else if (xbox.getRawButton(2)) {
			motors[6].set(.3);
		}
		else if(!autoClimbing.enabled){
			motors[6].set(0);
		}

		//Control of balls
		if(xbox.getRawButton(3)) {
			motors[7].set(.3);
		}
		else if(xbox.getRawButton(4)) {
			motors[7].set(-.3);
		}
		else  {
			motors[7].set(0);
		}
		
		//Control Door
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
			size = this.size;
		}

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
		
		double lspeed = speed, rspeed = speed;
	
		if(robotGyro.getAngle() < targetAngle){
			rspeed -= speed * Math.abs(robotGyro.getAngle() - targetAngle)/25.0 ;
		}
		else if(robotGyro.getAngle() > targetAngle) {
			lspeed -= speed *  Math.abs(robotGyro.getAngle() - targetAngle)/25.0;
		}
	
		rspeed = rspeed < 0 ? 0 : rspeed;
		lspeed = lspeed < 0 ? 0 : lspeed;
		if(lastDiff != diff) 
			System.out.println(lspeed + " " + rspeed);

		this.setRightSide(rspeed);
		this.setLeftSide(lspeed);
		Timer.delay(.05);
		

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