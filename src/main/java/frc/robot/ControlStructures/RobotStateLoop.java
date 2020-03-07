package frc.robot.ControlStructures;



import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.sensors.Pigeon;

/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope orientation, and velocity, among
 * various other factors. Similar to a car's odometer.
 */
public class RobotStateLoop extends Subsystem 
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
    Drivetrain drivetrain;
    Pigeon pigeon;
    
    RobotStateLoop() 
    {
        robotState = RobotState.getInstance();
        drivetrain = Drivetrain.getInstance();
        pigeon = Pigeon.getInstance();
    }
    


    @Override
    public void init() 
    {
    	robotState.setPrevEncoderDistance(drivetrain.getSensedInchesLeft(), drivetrain.getSensedInchesRight());
    }

    @Override
    public void run() 
    {
    	// the following DriveState elements are set during DriveLoop, called just previous to RobotStateLoop,
    	// and in the same LoopController thread
    	
        double time      = Timer.getFPGATimestamp();
        double lDistance = drivetrain.getSensedInchesLeft();
        double rDistance = drivetrain.getSensedInchesRight();
        double lSpeed    = drivetrain.getSensedIPS().leftSpeed;
        double rSpeed    = drivetrain.getSensedIPS().rightSpeed;
        double gyroAngle = Math.toRadians(pigeon.getHeadingDeg()+90); //90 to handle internal errors

        robotState.generateOdometryFromSensors(time, lDistance, rDistance, lSpeed, rSpeed, gyroAngle);
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub

    }

    @Override
    public void updateSmartDashboard() {
        // TODO Auto-generated method stub

    }
}
