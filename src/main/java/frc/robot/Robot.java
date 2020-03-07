package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControlStructures.SubsystemController;
import frc.robot.Subsystems.ConveyorBelt;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Kickers;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter.Limelight;
import frc.robot.Subsystems.Shooter.Limelight.LedMode;
import frc.robot.sensors.GyroBase;
import frc.robot.sensors.Pigeon;
import frc.robot.Subsystems.Shooter.ShooterMaster;


public class Robot extends TimedRobot {
  SubsystemController subsystemController;
  UsbCamera ClimbCam = CameraServer.getInstance().startAutomaticCapture(); 
  CvSink cvSink = CameraServer.getInstance().getVideo();
  CvSource outputStream = CameraServer.getInstance().putVideo("ClimbCam", 640, 480);

  Pigeon pigeon;
  

  @Override
  public void disabledInit() {
    Limelight.getInstance().setLEDMode(LedMode.kOff);
  }

  @Override
  public void robotInit() {
    pigeon = Pigeon.getInstance();

    subsystemController = new SubsystemController();
    //subsystemController.addSubsystem(Drivetrain.getInstance());
    //subsystemController.addSubsystem(ShooterMaster.getInstance());
    subsystemController.addSubsystem(ConveyorBelt.getInstance());
    //subsystemController.addSubsystem(Intake.getInstance());
    //subsystemController.addSubsystem(Kickers.getInstance());
    //subsystemController.addSubsystem(Lift.getInstance());

    subsystemController.init();


    SmartDashboard.putBoolean("Pigeon/Zero", false);
    SmartDashboard.putBoolean("Pigeon/LockInit", false);
    SmartDashboard.putNumber("Pigeon/SetInit", 0);
  }

  @Override
  public void robotPeriodic() {   
  }

  @Override
  public void autonomousInit() {
    subsystemController.resetForCalibration();
  }

  @Override
  public void autonomousPeriodic() {
    subsystemController.updateSmartDashboard(); 

    if(!subsystemController.calibrationComplete()){
      subsystemController.runCalibration();
    } else {

    }
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    subsystemController.onTeleLoop();
    subsystemController.updateSmartDashboard(); 

    if(SmartDashboard.getBoolean("Pigeon/Zero", false)){
      pigeon.zeroSensor();
    }
    if(SmartDashboard.getBoolean("Pigeon/LockInit", false)){
      pigeon.zeroHeading(SmartDashboard.getNumber("Pigeon/SetInit", 0));
    }

    System.out.println(pigeon.getHeadingDeg());
  }

  @Override
  public void testPeriodic() {
    
  }
}