package frc.robot.Subsystems.Shooter;

import frc.robot.Constants;
import frc.robot.util.Vector2d;

public class ShooterCalcs {
    //Units: inches, radians


    public static final Vector2d shooterPosFromRobot = new Vector2d(-12.0, 0);


    public static Vector2d getRobotDisplacement(double leftLastPos, double rightLastPos){
        Arc arc = getArc(leftLastPos, rightLastPos);
        double robotDispMag = 2*arc.radius*Math.sin(arc.angle/2.0);
        Vector2d robotDisplacement = new Vector2d(-robotDispMag*Math.sin(arc.angle/2.0),robotDispMag*Math.cos(arc.angle/2.0));
        return robotDisplacement;
    }

    public static Arc getArc(double leftLastPos, double rightLastPos){
        double arcAngle = (rightLastPos-leftLastPos)/Constants.kDrivetrainWidth;
        double arcLength = (rightLastPos+leftLastPos)/2.0;
        double arcRadius = arcLength/arcAngle;
        return new Arc(arcAngle, arcRadius);
    }

    


    // public static Vector2d getShooterVelocity(double leftWheelSpeed, double rightWheelSpeed){
    //     double leftSpeed = leftWheelSpeed;
    //     double rightSpeed = rightWheelSpeed;
    //     LinearAngularSpeed robotSpeed = Kinematics.forwardKinematics(leftSpeed, rightSpeed);
    //     Vector2d shooterVelocity;
    //     if(robotSpeed.angularSpeed > Math.PI/6.0){
    //         //Only use if angular speed is of some significant amount
    //         double motionRadius = Math.abs(robotSpeed.linearSpeed/robotSpeed.angularSpeed);
    //         Vector2d robotRadius = robotSpeed.angularSpeed > 0 ? new Vector2d(0,motionRadius) : new Vector2d(0,-motionRadius);
    //         Vector2d shooterRadius = shooterPosFromRobot.sub(robotRadius);
    //         double shooterVelMag = Math.abs(shooterRadius.length()*robotSpeed.angularSpeed);
    //         shooterVelocity = new Vector2d(shooterVelMag,0);
    //         double shooterVelAng = robotRadius.angle()+shooterRadius.angle();
    //         shooterVelocity = robotSpeed.linearSpeed>0 ? (shooterVelocity.rotate(shooterVelAng)) : (shooterVelocity.rotate(shooterVelAng+Math.PI));
    //     } else {
    //         shooterVelocity = new Vector2d(robotSpeed.linearSpeed, 0);
    //     }
    //     return shooterVelocity;
    // }




    static class Arc {
        double angle, length, radius;
        public Arc(double angle, double radius){
            this.angle = angle;
            this.radius = radius;
            this.length = angle*radius;
        }
    }




}