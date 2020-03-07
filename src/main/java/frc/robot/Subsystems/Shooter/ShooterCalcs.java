package frc.robot.Subsystems.Shooter;

import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.LinearAngularSpeed;
import frc.robot.util.Pose;
import frc.robot.util.Vector2d;

public class ShooterCalcs {
    //General Units: inches, radians

    //Distance (in), rpm, hoodPos (deg)
    public static double[][] dataTable = {
        {24,3600,0},
        {49,2750,22},
        {105,3000,35},
        {189,4000,45},
        {265,4750,47},
    };

    //Physical Variables =====================================
    private static final double targetHeight = 99;
    private static final double cameraHeight = 41;
    private static final double cameraAngleElevation = Math.toRadians(23);
    public static final Vector2d shooterPosFromRobot = new Vector2d(-12.0, 0);
    public static final Vector2d shooterPosFromCam = new Vector2d(0.0, 0);
    public static final double targetSmoothing = (1.0/3.0);

    public static final double shooterWheelRadius = 3;


    /**
     * Ay what up
     * @param lastTargetPos
     * @param leftDeltaPos
     * @param rightDeltaPos
     * @return The target displacement vector relative to the robot's front
     */

    public static Vector2d getNewTargetPos(Vector2d lastTargetPos, double leftDeltaPos, double rightDeltaPos){
        Pose newRobotPose = getNewRobotPose(leftDeltaPos, rightDeltaPos);
        Vector2d newTargetPos = lastTargetPos.sub(newRobotPose.getPosition());
        newTargetPos = newTargetPos.rotate(-newRobotPose.getHeading());
        return newTargetPos;
    }

    public static Pose getNewRobotPose(double leftDeltaPos, double rightDeltaPos){
        Arc arc = getArc(leftDeltaPos, rightDeltaPos);
        double robotDispMag = 2*arc.radius*Math.sin(arc.angle/2.0);
        Vector2d robotDisplacement = new Vector2d(-robotDispMag*Math.sin(arc.angle/2.0),robotDispMag*Math.cos(arc.angle/2.0));
        Pose robotPose = new Pose(robotDisplacement, arc.angle);
        return robotPose;
    }

    public static Arc getArc(double leftLastPos, double rightLastPos){
        double arcAngle = (rightLastPos-leftLastPos)/Constants.kDrivetrainWidth;
        double arcLength = (rightLastPos+leftLastPos)/2.0;
        double arcRadius = arcLength/arcAngle;
        return new Arc(arcAngle, arcRadius);
    }


    /**
     * 
     * @param targetDistance
     * @return Necessary hood position for the targetDistance
     */
    public static double calcHoodPosition(double targetDistance){
        int keyL = getLinear(targetDistance, dataTable);
        double hoodPosition = handleLinear(targetDistance, dataTable[keyL][0], dataTable[keyL+1][0], dataTable[keyL][2], dataTable[keyL+1][2]);
        hoodPosition /= 57.2958; //Converting to radians
        return hoodPosition;
    }

    public static double calcShooterVelocity(double distance){
        int keyL = getLinear(distance, dataTable);
        double nominalSpeed = handleLinear(distance, dataTable[keyL][0], dataTable[keyL+1][0], dataTable[keyL][1], dataTable[keyL+1][1]);
        nominalSpeed /= 9.5493; //RPM to rps
        return nominalSpeed; //Determine the balls' velocity
    }


    //===========================
    //Leading Shots:
    //===========================

    /**
     * 
     * @param targetDisplacement
     * @param driveLASpeed
     * @return The lead velocity vector, in RPS
     */

    public static Vector2d calcShooterLeadVelocity(Vector2d targetDisplacement, LinearAngularSpeed driveLASpeed){
        double targetBallVelocityMag = calcShooterVelocity(targetDisplacement.length())*shooterWheelRadius*0.5;
        Vector2d targetVelocityBall = new Vector2d(targetBallVelocityMag, 0); //Used to maintain magnitude
        targetVelocityBall = targetVelocityBall.rotate(targetDisplacement.angle()); //Rotating it back to the correct rotation 

        Vector2d turretVelocity = getTurretVelocity(driveLASpeed); //Get the velocity of the shooter on the robot

        Vector2d leadVelocityBall = targetVelocityBall.sub(turretVelocity); //Determine the necessary velocity of the ball being shot
        Vector2d leadVelocityShooter = Vector2d.magnitudeAngle(leadVelocityBall.length()*(2/shooterWheelRadius), leadVelocityBall.angle());

        return leadVelocityShooter;
    }


    public static Vector2d getTurretVelocity(LinearAngularSpeed driveLASpeed){
        Vector2d shooterVelocity;
        if(driveLASpeed.angularSpeed > Math.PI/6.0){
            //Only use if angular speed is of some significant amount
            double motionRadius = Math.abs(driveLASpeed.linearSpeed/driveLASpeed.angularSpeed);
            Vector2d robotRadius = driveLASpeed.angularSpeed > 0 ? new Vector2d(0,motionRadius) : new Vector2d(0,-motionRadius);
            Vector2d shooterRadius = shooterPosFromRobot.sub(robotRadius);
            double shooterVelMag = Math.abs(shooterRadius.length()*driveLASpeed.angularSpeed);
            shooterVelocity = new Vector2d(shooterVelMag,0);
            double shooterVelAng = robotRadius.angle()+shooterRadius.angle();
            shooterVelocity = driveLASpeed.linearSpeed>0 ? (shooterVelocity.rotate(shooterVelAng)) : (shooterVelocity.rotate(shooterVelAng+Math.PI));
        } else {
            shooterVelocity = new Vector2d(driveLASpeed.linearSpeed, 0);
        }
        return shooterVelocity;
    }

    public static Vector2d getTargetDisplacement(Vector2d runningTargetPos, double verticalRads, double horizontalRads, double turretRads){
        //Target Displacement angle is on the interval [-pi,pi]
        double targetY = (targetHeight-cameraHeight)/Math.tan(verticalRads+cameraAngleElevation);
        double targetX = targetY*Math.tan(horizontalRads); //Negative is to ensure that left of camera is positive from top-view
        Vector2d detectedTargetPos = new Vector2d(targetX, targetY);
        detectedTargetPos = detectedTargetPos.rotate(-Math.PI/2.0); //Forward is considered at 90 degrees when sensed but should be 0.
        detectedTargetPos = detectedTargetPos.sub(shooterPosFromCam); //Map the detected vector onto the shooter's center
        detectedTargetPos = detectedTargetPos.rotate(turretRads); //This is used to rotate it back to the robot's perspective which is used to ground our measurements

        //Averaging:
        if(runningTargetPos == null){
            //First time through
            runningTargetPos = detectedTargetPos;
        } else {
            runningTargetPos = runningTargetPos.expAverage(detectedTargetPos, targetSmoothing);
        }
        return runningTargetPos;
    }

    //Simpler version of getTargetDisplacement();
    // public double getTargetDisplacement(){
    //     double targetY = (targetHeight-cameraHeight)/Math.tan(limelight.getTargetVerticalAngleRad()+Math.toRadians(cameraAngleElevation));
    //     double targetX = targetY*Math.tan(-limelight.getTargetHorizontalAngleRad()); //Negative is to ensure that left of camera is positive from top-view
    //     Vector2d detectedTargetPos = new Vector2d(targetX, targetY);
    //     return detectedTargetPos.length();
    // }

    public static int getLinear (double d, double table[][])
    {
        double distance = Math.max(Math.min(d, table[table.length-1][0]), table[0][0]);
        int k;
        for (k=0;k<table.length;k++)
        {
            if (distance <= table[k][0])
            {
                break;
            }
        }
        return Math.max(k-1, 0);
    }


    public static double handleLinear (double d, double dL, double dH, double sL, double sH)
    {
        return (sH-sL)*Math.min((d-dL)/(dH-dL),1)+sL;
    }

    





    static class Arc {
        double angle, length, radius;
        public Arc(double angle, double radius){
            this.angle = angle;
            this.radius = radius;
            this.length = angle*radius;
        }
    }
}