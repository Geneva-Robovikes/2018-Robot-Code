package org.usfirst.frc.team3067.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.networktables.*;

public class Robot extends IterativeRobot { //class of methods called by FRC, names are fairly self explanatory
	RobotInstance dozer;
	//NetworkTableInstance dashTable;
	SendableChooser autoChooser;
	
	
	public void robotInit() { 
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(640, 480);
		System.out.println(CameraServer.kBasePort);
		
		dozer = new RobotInstance();
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Position 1", 1);
		autoChooser.addObject("Position 2", 2);
		autoChooser.addObject("Position 3", 3);
		SmartDashboard.putData("Autonomous mode chooser", autoChooser);
		SmartDashboard.putNumber("Camera port", CameraServer.kBasePort);
		//dashTable = NetworkTableInstance.create();
		//SmartDashboard.putString("DB/String 0", "no");
	}
	public void autonomousInit() {
		
		switch((int)autoChooser.getSelected()) {
		case(1):
			dozer.AutonomousPos1();
			break;
		case(2):
			dozer.AutonomousPos2();
			break;
		default:
					
		}
	}
	public void autonomousPeriodic() {
		
		
		
	
	}
	public void teleopInit() {
		dozer.clearGyroAngle();

	}
	public void teleopPeriodic() {
		dozer.update();
	}
}