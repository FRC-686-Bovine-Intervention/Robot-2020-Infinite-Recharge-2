package frc.robot.subsystems;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.loops.Loop;

public class Camera implements Loop {
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
    public void onStart() {}

    @Override
    public void onLoop() {
        switch(lift.getPTOState())
        {
            case LIFT_ENABLED:              servo.setAngle(90); break;
            default:                        servo.setAngle(0);
        }
    }

    @Override
    public void onStop() {}
}