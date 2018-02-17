package org.usfirst.frc.team3067.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
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
	//Solenoid solGrabber; 
	DigitalInput limTopA, limBottomA, limTopB, limBottomB;
	Encoder LeftEnc, RightEnc;
	ADXRS450_Gyro gyro;
	SmartDashboard dashBoard;
	double outputA;
	double outputB;
	double autoSpeed;
	double teleSpeed;
	double gyroAngle;
		
	public RobotInstance() { // Instantiate joystick, talons, encoders, limit switches, etc...
		stickoboyo = new RobotStick(5);
		
		talLF      = new Talon(0);
		talLB      = new Talon(1);
		talRF      = new Talon(2);
		talRB      = new Talon(3);
		talGrabber = new Talon(4);
		talLiftA   = new Talon(5);
		talLiftB   = new Talon(6);
		
		//solGrabber = new Solenoid(0);
		
		limTopA    = new DigitalInput(0);
		limBottomA = new DigitalInput(1);
		limTopB    = new DigitalInput(2);
		limBottomB = new DigitalInput(3);
		
		LeftEnc  = new Encoder(4,5,false,EncodingType.k4X); 		
		RightEnc = new Encoder(6,7,true,EncodingType.k4X);
		
		gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
		
		autoSpeed = -.2; // SET AUTONOMOUS SPEED HERE
		teleSpeed = 1; // SET TELEOP SPEED HERE
		gyroAngle = 0;
		
		LeftEnc.reset(); // Reset encoders when code is built
		RightEnc.reset();
		
	}
	
	public void AutonomousDrive() { // Move forward, if starting on right or left
		double distanceVal = (8 * Math.PI)/360;
		LeftEnc.reset(); // Reset encoders when method runs
		RightEnc.reset();
		LeftEnc.setDistancePerPulse(distanceVal); 
		RightEnc.setDistancePerPulse(distanceVal);
		while (LeftEnc.getDistance() < 140 && RightEnc.getDistance() < 140) {
			talLF.set(autoSpeed);
			talLB.set(autoSpeed);
			talRF.set(-autoSpeed);
			talRB.set(-autoSpeed);
		}
		
		talLF.set(0);
		talLB.set(0);
		talRF.set(0);
		talRB.set(0);
	}
	
	public void AutonomousSwitch() { // Drop crate on switch, if starting in middle
		
		int rotateValue = 50;
		double distanceVal = (8 * Math.PI)/360; // 1 rotation = 8pi inches
		double Ldistance; // Left encoder distance accumulator
		double Rdistance; // Right encoder distance accumulator
		
		LeftEnc.reset(); // Reset encoders when method runs
		RightEnc.reset();
		LeftEnc.setDistancePerPulse(distanceVal); 
		RightEnc.setDistancePerPulse(distanceVal);
		
		String switchScale = DriverStation.getInstance().getGameSpecificMessage(); // Returns which side of switch is yours
		if(switchScale.charAt(0) == 'L') {
			// Go to left side
			while(LeftEnc.getDistance() < 12 && RightEnc.getDistance() < 12) { // Move off alliance wall
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while(getGyroAngle() > -rotateValue) { // Rotate left
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while((LeftEnc.getDistance() - Ldistance) < 60 && (RightEnc.getDistance() - Rdistance) < 60) { // Move until even with switch
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while(getGyroAngle() < rotateValue) { // Rotate right
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while((LeftEnc.getDistance() - Ldistance) < 104 && (RightEnc.getDistance() - Rdistance) < 104) { // Move until at switch
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
				System.out.println(LeftEnc.getDistance());
			}
			talLF.set(0);
			talLB.set(0);
			talRF.set(0);
			talRB.set(0);
			talGrabber.set(.5); // Drop cube
			//solGrabber.set(false);
			Timer.delay(1);
			talGrabber.set(0); // End auto
		}
		
		else if (switchScale.charAt(0) == 'R') {
			// Go to right side
			while(LeftEnc.getDistance() < 12 && RightEnc.getDistance() < 12) { // Move off alliance wall
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);		
			}
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while(getGyroAngle() < rotateValue) {// Rotate right
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(autoSpeed);
				talRB.set(autoSpeed);
			}
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while((LeftEnc.getDistance() - Ldistance) < 60 && (RightEnc.getDistance() - Rdistance) < 60) { // Move until even with switch
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);		
			}
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while(getGyroAngle() > -rotateValue) { // Rotate left
				talLF.set(-autoSpeed);
				talLB.set(-autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
			}
			Ldistance = LeftEnc.getDistance();
			Rdistance = RightEnc.getDistance();
			while((LeftEnc.getDistance() - Ldistance) < 104 && (RightEnc.getDistance() - Rdistance) < 104) { // Move until at switch
				talLF.set(autoSpeed);
				talLB.set(autoSpeed);
				talRF.set(-autoSpeed);
				talRB.set(-autoSpeed);
			}
			talGrabber.set(.5); // Drop cube
			//solGrabber.set(false); 
			Timer.delay(1);
			talGrabber.set(0); // End auto
		}
	} 
	
	public void update() { // What runs in teleopPeriodic

		//setMotor(); // Sets drive motors
		//grabberGrab(); // Sets grabber motors
		//grabbersolGrabber(); // Sets grabber solenoid
		//lift(); // Sets lift motors
		getGyroAngle();
		SmartDashboard.updateValues();
		
		
	}
	
	public void clearGyroAngle() { // Clears gyro angle
		gyroAngle = 0;
	}
	
	public double getGyroAngle() { // Gets and sets gyro angle simultaneously -- !!ONLY RUN ONCE PER CYCLE!!
		Timer.delay(.001); // Sets a uniform delay for calculation
		if (gyro.getRate() >= 1 || gyro.getRate() <= -1) { // Deadzone; prevents slight input
			gyroAngle += (gyro.getRate() * .001) * 20;
			SmartDashboard.putNumber("gyroAngle", gyroAngle);
			System.out.println(gyroAngle + "    " + SmartDashboard.getNumber("gyroAngle", 699));// Increments gyroAngle by rate * time
		}
		
		return gyroAngle; // Returns the current gyro angle
	}
	
	public void setMotor() { // Arcade drive
		talLF.set(teleSpeed * (stickoboyo.getDY() + .8 * stickoboyo.getDZ()));
		talLB.set(teleSpeed * (stickoboyo.getDY() + .8 * stickoboyo.getDZ()));
		talRF.set(teleSpeed * (-stickoboyo.getDY() + .8 * stickoboyo.getDZ()));
		talRB.set(teleSpeed * (-stickoboyo.getDY() + .8 * stickoboyo.getDZ()));
		
	}
	
	public void grabberGrab() { // Spins belt in/out
		talGrabber.set(0);
		if(stickoboyo.getButton(1) && !stickoboyo.getButton(2)) {
			talGrabber.set(.2);
		}
		if(!stickoboyo.getButton(1) && stickoboyo.getButton(2)) {
			talGrabber.set(-.2);
		}
	}
	
	public void grabbersolGrabber() { // Grabber pneumatics
		if(stickoboyo.getButtonDown(3)) {
			//solGrabber.set(stickoboyo.getButtonToggle(3));
		}
	}
	
	public void lift() {
		outputA = 0; // How fast A motor spins
		outputB = 0; // How fast B motor spins
		// A section
		if(stickoboyo.getButton(4) && !stickoboyo.getButton(6) && !limBottomA.get()) {
			outputA = -0.5; // down
		} else if(!stickoboyo.getButton(4) && stickoboyo.getButton(6) && !limTopA.get()) {
			outputA = 0.5; // up
		}
		
		// B section
		if(stickoboyo.getButton(4) && !stickoboyo.getButton(6) && !limBottomB.get()) {
			outputB = -0.5; // down
		} else if(!stickoboyo.getButton(4) && stickoboyo.getButton(6) && !limTopB.get()) {
			outputB = 0.5; // up
		}
		// Set motor speed at end
		talLiftA.set(outputA);
		talLiftB.set(outputB);
	}
		
}
