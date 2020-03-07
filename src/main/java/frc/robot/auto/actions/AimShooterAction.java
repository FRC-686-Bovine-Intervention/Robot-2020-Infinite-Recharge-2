package frc.robot.auto.actions;

import frc.robot.Subsystems.Shooter.Limelight;
import frc.robot.Subsystems.Shooter.Turret;
import frc.robot.util.DataLogger;
import frc.robot.util.Vector2d;

public class AimShooterAction implements Action {
    //This action is used to line the turret up with the shooter
    //

    
    private boolean finished = false;

    private Turret turret = Turret.getInstance();
    private Limelight limelight = Limelight.getInstance();

    private double backupAngle; //This is used in the event that a target can not be found
    private int sensingAttempts = 0; //Tracks attempts
    private static int maxAttempts = 20;

    private static double toleranceRads = 0.075; //Allowable error in radians

    
    public AimShooterAction(double suggestedDeg) {
        backupAngle = suggestedDeg;
    }
    public AimShooterAction(){}



    @Override
    public void start() {
        //Somewhat outdated....
        if(backupAngle != 0){
            turret.setPosition(backupAngle); //Start by looking in the general direction
        } else {
            turret.setPosition(turret.getSensedPosition());
        }
    }

    @Override
    public void update() {
        if(!limelight.getIsTargetFound()){
            sensingAttempts++;
            if(sensingAttempts >= maxAttempts){
                finished = true; //Exit action
                return;
            }
        } else {
            Vector2d targetPos = shooter.getTargetDisplacement();
            lastTargetPos = targetPos; //Updating this for external functions
            //First see if we are close enough:
            double cTurretAngle = turret.getSensedPosition()
            double errorRad = limelight.getTargetHorizontalAngleRad(); //Should output positive if CCW correction is needed
            if(Math.abs(errorRad) <= toleranceRads){
                finished = true;
            }
            //Otherwise...
            double correction = errorRad/2; //Assumes positive means leftwards correction
            shooter.setTurretDeg(Math.toDegrees(cTurretAngle+correction)); //Adjusting turret
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {

    }

    public Vector2d getSensedTargetPos(){
        return lastTargetPos; //In the event that we want to reuse the 
    }



    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
	@Override
	public DataLogger getLogger() { return logger; }

}