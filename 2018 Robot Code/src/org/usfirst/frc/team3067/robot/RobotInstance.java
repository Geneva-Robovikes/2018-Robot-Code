package org.usfirst.frc.team3067.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

public class RobotInstance { // Declares robot components
	RobotStick stickoboyo;
	Talon talLF, talRF, talLB, talRB, talGrabber, talLiftA, talLiftB; // front left, front right, back left, back right, grabber belt, lift A, lift B
    Solenoid solGrabber1;
    Solenoid solGrabber2;
	DigitalInput limTopA, limBottomA, limTopB, limBottomB;
	Encoder LeftEnc, RightEnc;
	ADXRS450_Gyro gyro;
	SmartDashboard dashBoard;
	double outputA;
	double outputB;
	final double autoSpeed;
	double teleSpeed;
	double turnSpeed;
	double driveSmoothing;
	final double liftSpeed;
	final double grabberSpeed;
	double gyroAngle;
	String switchScale;
	boolean smoothSteering;
	
	public RobotInstance() { // Instantiate joystick, talons, encoders, limit switches, etc...
		stickoboyo = new RobotStick(5);
		
		talLF      = new Talon(0); //0
		talLB      = new Talon(1); //1
		talRF      = new Talon(2); //2
		talRB      = new Talon(3); //3
		talLiftA   = new Talon(4); //5
		talLiftB   = new Talon(5); //4
		talGrabber = new Talon(6); //6
		
		solGrabber1 = new Solenoid(0);
		solGrabber2 = new Solenoid(1);
		
		limTopA    = new DigitalInput(0);
		limBottomA = new DigitalInput(1);
		limTopB    = new DigitalInput(2);
		limBottomB = new DigitalInput(3);
		
		LeftEnc  = new Encoder(4,5,true,EncodingType.k4X); 		
		RightEnc = new Encoder(6,7,false,EncodingType.k4X);
		
		gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
		
		autoSpeed = 0.2; // SET AUTONOMOUS SPEED HERE
		teleSpeed = 0.5; // SET TELEOP SPEED HERE
		turnSpeed = 0.8; // SET TURN SPEED HERE
		liftSpeed = 1; // SET LIFT SPEED HERE
		grabberSpeed  = 0.2; // SET GRABBER SPEED HERE
		
		driveSmoothing = 0.3;
		smoothSteering = true;
		
		gyroAngle = 0;
		
		
		switchScale = DriverStation.getInstance().getGameSpecificMessage();
		
		LeftEnc.reset(); // Reset encoders when code is built
		RightEnc.reset();
		SmartDashUpdate();
	}
	
	public void AutonomousPos1() { //If we are staring in position 1 (far left)
		double distanceVal = (8 * Math.PI)/360;
		double Ldistance; // Left encoder distance accumulator
		double Rdistance;
		LeftEnc.reset(); // Reset encoders when method runs
		RightEnc.reset();
		LeftEnc.setDistancePerPulse(distanceVal); 
		RightEnc.setDistancePerPulse(distanceVal);
		if (switchScale.charAt(0) == 'R') { // If switch is on right
			while (LeftEnc.getDistance() > -154 && RightEnc.getDistance() < 154) {//Move forward across base line
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
		}
		else if (switchScale.charAt(0) == 'L') {//If switch is on left
			while (LeftEnc.getDistance() > -154 && RightEnc.getDistance() < 154) {//Move forward until in line with switch
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			while (!limTopB.get()) { //Raise lift
				talLiftB.set(liftSpeed);
			}
			talLiftB.set(0);
			while (gyroAngle < 90) { //Rotate right
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while ((LeftEnc.getDistance() - Ldistance) > -35 && (RightEnc.getDistance() - Rdistance) < 35) {//Move forward until directly next to switch
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			talGrabber.set(liftSpeed); //Drop cube
			solGrabber1.set(false);
			Timer.delay(1);
			talGrabber.set(0);
		}
	}
	
	public void AutonomousPos2() { //If we are staring in position 2 (middle)
		
		int rotateValue = 50;
		double distanceVal = (8 * Math.PI)/360; // 1 rotation = 8pi inches
		double Ldistance; // Left encoder distance accumulator
		double Rdistance; // Right encoder distance accumulator
		
		LeftEnc.reset(); // Reset encoders when method runs
		RightEnc.reset();
		LeftEnc.setDistancePerPulse(distanceVal); 
		RightEnc.setDistancePerPulse(distanceVal);
		
		while (!limTopA.get()) {
			talLiftB.set(liftSpeed);
		}
		talLiftB.set(0);
		if(switchScale.charAt(0) == 'L') {//If switch is on left
			// Go to left side
			while(LeftEnc.getDistance() < 12 && RightEnc.getDistance() < 12) { // Move off alliance wall
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while(getGyroAngle() < rotateValue) { // Rotate left
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while((LeftEnc.getDistance() - Ldistance) < 60 && (RightEnc.getDistance() - Rdistance) < 60) { // Move until even with switch
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while(getGyroAngle() > -rotateValue) { // Rotate right
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while((LeftEnc.getDistance() - Ldistance) < 80 && (RightEnc.getDistance() - Rdistance) < 80) { // Move until at switch
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			talGrabber.set(liftSpeed); // Drop cube
			//solGrabber1.set(false);
			Timer.delay(1);
			talGrabber.set(0); // End auto
		}
		
		else if (switchScale.charAt(0) == 'R') {//If switch is on right
			// Go to right side
			while(LeftEnc.getDistance() < 12 && RightEnc.getDistance() < 12) { // Move off alliance wall
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);		
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while(getGyroAngle() > -rotateValue) {// Rotate right
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while((LeftEnc.getDistance() - Ldistance) < 60 && (RightEnc.getDistance() - Rdistance) < 60) { // Move until even with switch
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);		
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while(getGyroAngle() < rotateValue) { // Rotate left
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while((LeftEnc.getDistance() - Ldistance) < 80 && (RightEnc.getDistance() - Rdistance) < 80) { // Move until at switch
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			talGrabber.set(liftSpeed); // Drop cube
			solGrabber1.set(false); 
			Timer.delay(1);
			talGrabber.set(0); // End auto
		}
	} 
	
	public void AutonomousPos3() { //If we are staring in position 3 (far right)
		double distanceVal = (8 * Math.PI)/360;
		double Ldistance; // Left encoder distance accumulator
		double Rdistance;
		LeftEnc.reset(); // Reset encoders when method runs
		RightEnc.reset();
		LeftEnc.setDistancePerPulse(distanceVal); 
		RightEnc.setDistancePerPulse(distanceVal);
		if (switchScale.charAt(0) == 'L') { //If switch is on left
			while (LeftEnc.getDistance() < 154 && RightEnc.getDistance() < 154) { //Move forward across base line
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
		}
		else if (switchScale.charAt(0) == 'R') { //If switch is on right
			while (LeftEnc.getDistance() < 154 && RightEnc.getDistance() < 154) { //Move forward until in line with switch
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			while (!limTopB.get()) { //Raise lift
				talLiftB.set(liftSpeed);
			}
			talLiftB.set(0);
			while (gyroAngle < 90) {//Rotate left
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while ((LeftEnc.getDistance() - Ldistance) < 35 && (RightEnc.getDistance() - Rdistance) < 35) { //Move forward until directly next to switch
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			talGrabber.set(liftSpeed); //Drop cube
			solGrabber1.set(false);
			Timer.delay(1);
			talGrabber.set(0);
		}
	}
	
	public void update() { // What runs in teleopPeriodic
		//teleSpeed = SmartDashboard.getNumber("teleSpeed", 0.5);
		//turnSpeed = SmartDashboard.getNumber("turnSpeed", 0.5);
		//driveSmoothing = SmartDashboard.getNumber("smoothing", driveSmoothing);
		//smoothSteering = SmartDashboard.getBoolean("smoothSteering", true);
		//if (smoothSteering) {
			//setMotorSmooth(); // Sets drive motors
	//	} else {
			//setMotorStandard();
	//	}
		//grabberGrab(); // Sets grabber motors
		//grabberSol(); // Sets grabber solenoid
		lift(); // Sets lift motors
		//SmartDashUpdate();
		
		
	}
	
	public void resetValues() { // Clears gyro angle
		gyroAngle = 0;
		
	}
	
	public double getGyroAngle() { // Gets and sets gyro angle simultaneously -- !!ONLY RUN ONCE PER CYCLE!!
		Timer.delay(.001); // Sets a uniform delay for calculation
		if (gyro.getRate() >= 1 || gyro.getRate() <= -1) { // Deadzone; prevents slight input
			gyroAngle += (gyro.getRate() * .001) * 20;
			//SmartDashboard.putNumber("gyroAngle", gyroAngle);
			//System.out.println(gyroAngle + "    " + SmartDashboard.getNumber("gyroAngle", 699));// Increments gyroAngle by rate * time
		}
		
		return gyroAngle; // Returns the current gyro angle
	}
	
	public double scaleMotor(double stickInput, Talon talon) {		
		double newMotorValue = talon.get() * (1 - driveSmoothing) + stickInput * driveSmoothing; 
		if (newMotorValue < .02 && newMotorValue > -.02)
			return 0;
		else
			return newMotorValue;
	}	
	
	public void setMotorSmooth() { // Arcade drive
		talLF.set(scaleMotor(teleSpeed * -stickoboyo.getDY() + turnSpeed * stickoboyo.getDZ(), talLF));
		talLB.set(scaleMotor(teleSpeed * -stickoboyo.getDY() + turnSpeed * stickoboyo.getDZ(), talLB));
		talRF.set(scaleMotor(teleSpeed * stickoboyo.getDY() + turnSpeed * stickoboyo.getDZ(),  talRF));
		talRB.set(scaleMotor(teleSpeed * stickoboyo.getDY() + turnSpeed * stickoboyo.getDZ(),  talRB));		
	}
	
	public void setMotorStandard() {
		talLF.set(teleSpeed * -stickoboyo.getDY() + turnSpeed * stickoboyo.getDZ());
		talLB.set(teleSpeed * -stickoboyo.getDY() + turnSpeed * stickoboyo.getDZ());
		talRF.set(teleSpeed * stickoboyo.getDY() + turnSpeed * stickoboyo.getDZ());
		talRB.set(teleSpeed * stickoboyo.getDY() + turnSpeed * stickoboyo.getDZ());		
	}
	
	public void grabberGrab() { // Spins belt in/out
		talGrabber.set(0);
		if(stickoboyo.getButton(2) && !stickoboyo.getButton(1)) {
			talGrabber.set(grabberSpeed);
		}
		if(!stickoboyo.getButton(2) && stickoboyo.getButton(1)) {
			talGrabber.set(-grabberSpeed);
		}
	}
	
	public void grabberSol() { // Grabber pneumatics
		if(stickoboyo.getButtonPress(3)) {
			boolean val = stickoboyo.getButtonToggle(3);
			solGrabber1.set(val);
			solGrabber2.set(!val);
		}
		
	}
	
	public void lift() {
		outputA = 0; // How fast A motor spins
		outputB = 0; // How fast B motor spins
		// A section
		if(stickoboyo.getButton(4) && !stickoboyo.getButton(6)/* && !limBottomA.get()*/) {
			outputA = -liftSpeed; // down
		} else if(!stickoboyo.getButton(4) && stickoboyo.getButton(6)/* && !limTopA.get()*/) {
			outputA = liftSpeed; // up
		}
		
		// B section
		if(stickoboyo.getButton(4) && !stickoboyo.getButton(6)/* && !limBottomB.get()*/) {
			outputB = -liftSpeed; // down
		} else if(!stickoboyo.getButton(4) && stickoboyo.getButton(6)/* && !limTopB.get()*/) {
			outputB = liftSpeed; // up
		}
		// Set motor speed at end
		talLiftA.set(outputA);
		talLiftB.set(outputB);
	}
	

	public void SmartDashUpdate() {
		/*SmartDashboard.putNumber("talLF", talLF.get());
		SmartDashboard.putNumber("talLB", talLB.get());
		SmartDashboard.putNumber("talRF", talRF.get());
		SmartDashboard.putNumber("talRB", talRB.get());
		SmartDashboard.putNumber("gyroAngle", getGyroAngle());
		SmartDashboard.putNumber("teleSpeed", teleSpeed);
		SmartDashboard.putNumber("turnSpeed", turnSpeed);
		SmartDashboard.putNumber("smoothing", driveSmoothing);
		SmartDashboard.putBoolean("smoothSteering", smoothSteering);
		SmartDashboard.updateValues();*/
		
	}
}