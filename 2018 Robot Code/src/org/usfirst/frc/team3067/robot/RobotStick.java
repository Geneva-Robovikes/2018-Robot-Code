package org.usfirst.frc.team3067.robot;

import edu.wpi.first.wpilibj.*; 
import edu.wpi.first.wpilibj.SPI.Port;
//import java.math.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotStick extends Joystick { // Defines the joystick
	double dead;
	boolean buttonState[];
	SmartDashboard Dash;
	

	
	public RobotStick(int port) { // Declares the buttons of the joystick --> array used in getButtonToggle()
		super(port);
		buttonState = new boolean[12];
		
		for(int i=0; i < buttonState.length; i++) {
			buttonState[i] = false;
		}
		
	}
	
	public double deadZone(double input) { // Deadzone for all joystick input
		dead = .15;
		if (input < dead && input > -dead) {
			return 0;
		}
		else {
			return input;
		}
	}
	
	public double getDX() { // Gets joystick X input (left/right)
		return deadZone(this.getRawAxis(0));
	}
	
	public double getDY() { // Gets joystick Y input (forward/backward)
		return deadZone(this.getRawAxis(1));
	}
	
	public double getDZ() { // Gets joystick Z input (twist)
		return deadZone(this.getRawAxis(2));
	}
	
	public boolean getButton(int button) { // Continuous input while button is pressed
		return this.getRawButton(button);
	}
	
	public int getNub() { // Input for POV stick
		return this.getPOV();
	}
	
	public boolean getButtonToggle(int button) { // Toggle input on press
		if (this.getRawButtonPressed(button)) {
			buttonState[button] = !buttonState[button];
		}
		return buttonState[button];
	}
	
	public boolean getButtonDown(int button) { // Executes once when button is pressed
		return this.getRawButtonPressed(button);
	}
	
}