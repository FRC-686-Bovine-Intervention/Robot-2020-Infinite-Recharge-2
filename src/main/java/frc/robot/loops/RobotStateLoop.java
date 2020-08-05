package frc.robot.loops;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.lib.sensors.Pigeon;
import frc.robot.subsystems.Drive;

/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope orientation, and velocity, among
 * various other factors. Similar to a car's odometer.
 */
public class RobotStateLoop implements Loop 
{
 	// singleton class
	 private static RobotStateLoop instance = null;
	 public static RobotStateLoop getInstance() 
	 { 
		 if (instance == null) {
			 instance = new RobotStateLoop();
		 }
		 return instance;
	 }

    RobotState robotState;
    DriveState drive;
    Pigeon pigeon;
    
    RobotStateLoop() 
    {
        robotState = RobotState.getInstance();
        drive = DriveState.getInstance();
        pigeon = Pigeon.getInstance();
    }
    


    @Override
    public void onStart() 
    {
    	robotState.setPrevEncoderDistance(drive.getLeftDistanceInches(), drive.getLeftDistanceInches());
    }

    @Override
    public void onLoop() 
    {
    	// the following DriveState elements are set during DriveLoop, called just previous to RobotStateLoop,
    	// and in the same LoopController thread
    	
        double time      = Timer.getFPGATimestamp();
        double lDistance = drive.getLeftDistanceInches();
        double rDistance = drive.getRightDistanceInches();
        double lSpeed    = drive.getLeftSpeedInchesPerSec();
        double rSpeed    = drive.getRightSpeedInchesPerSec();
        double gyroAngle = Math.toRadians(pigeon.getHeadingDeg()+90); //90 to handle internal errors

        robotState.generateOdometryFromSensors(time, lDistance, rDistance, lSpeed, rSpeed, gyroAngle);
    }

    @Override
    public void onStop(){}
}
