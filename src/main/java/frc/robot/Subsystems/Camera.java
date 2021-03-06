package frc.robot.Subsystems;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.ControlStructures.Subsystem;

public class Camera extends Subsystem{
    private static Camera instance = null;
    public static Camera getInstance(){
        if(instance == null){
            instance = new Camera();
        }
        return instance;
    }

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(); 
    CvSink cvSink = CameraServer.getInstance().getVideo();
    CvSource outputStream = CameraServer.getInstance().putVideo("ClimbCam", 640, 480);

    private Servo servo;
    public Lift lift = Lift.getInstance();
    
    public Camera(){
        servo = new Servo(0);
    }

    @Override
    public void init(){}

    @Override
    public void run(){
        switch(lift.getPTOState())
        {
            case LIFT_ENABLED:              servo.setAngle(90);
            default:case DRIVE_ENABLED:     servo.setAngle(0);
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void updateSmartDashboard() {

    }
}