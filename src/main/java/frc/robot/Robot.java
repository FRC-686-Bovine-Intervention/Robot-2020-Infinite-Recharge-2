package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.ControlStructures.RobotState;
import frc.robot.ControlStructures.RobotStateLoop;
import frc.robot.ControlStructures.SubsystemController;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.ConveyorBelt;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Kickers;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter.Flywheel;
import frc.robot.Subsystems.Shooter.Hood;
import frc.robot.Subsystems.Shooter.Limelight;
import frc.robot.Subsystems.Shooter.Limelight.LedMode;
import frc.robot.sensors.Pigeon;
import frc.robot.util.Pose;
import frc.robot.util.RisingEdgeDetector;
import frc.robot.Subsystems.Shooter.ShooterMaster;


public class Robot extends TimedRobot {
  SubsystemController subsystemController;
  UsbCamera ClimbCam = CameraServer.getInstance().startAutomaticCapture(); 
  CvSink cvSink = CameraServer.getInstance().getVideo();
  CvSource outputStream = CameraServer.getInstance().putVideo("ClimbCam", 640, 480);
  Pigeon pigeon = Pigeon.getInstance();

  private double autoStartTime = 0;
  private boolean autoStarted = false;  

  @Override
  public void disabledInit() {
    Limelight.getInstance().setLEDMode(LedMode.kOff);
  }

  @Override
  public void robotInit() {

    pigeon.zeroSensor();

    subsystemController = new SubsystemController();
    subsystemController.addSubsystem(Drivetrain.getInstance());
    subsystemController.addSubsystem(ShooterMaster.getInstance());
    subsystemController.addSubsystem(ConveyorBelt.getInstance());
    subsystemController.addSubsystem(Camera.getInstance());
    subsystemController.addSubsystem(Kickers.getInstance());
    subsystemController.addSubsystem(Lift.getInstance());

    subsystemController.addSubsystem(RobotStateLoop.getInstance());

    subsystemController.init();

    //Move to auto!================================================================
    RobotState.getInstance().reset(FieldDimensions.middleStartPose);
  }

  @Override
  public void robotPeriodic() {   
  }

  @Override
  public void autonomousInit() {
    subsystemController.resetForCalibration();
    autoStarted = false;
  }

  @Override
  public void autonomousPeriodic() {
    subsystemController.updateSmartDashboard(); 

    if(!subsystemController.calibrationComplete()){
      subsystemController.runCalibration();
    } else {
      if(!autoStarted){
        autoStartTime = Timer.getFPGATimestamp();
        autoStarted = true;
        Kickers.getInstance().setPercent(Constants.kKickerShootPercent);
        Flywheel.getInstance().setRPS(275);
        Hood.getInstance().setPosition(Math.toRadians(22));
      }

      double shootingTime = Timer.getFPGATimestamp()-autoStartTime;
      if(shootingTime > 6){
        Kickers.getInstance().setPercent(0);
        Flywheel.getInstance().setRPS(0);
      }
  
      if(shootingTime > 1 && shootingTime <6){
        ConveyorBelt.getInstance().turnOnTower();
        ConveyorBelt.getInstance().turnOnVBelt();
      } else {
        ConveyorBelt.getInstance().stopTower();
        ConveyorBelt.getInstance().stopVBelt();
      }
  
      if(shootingTime > 6 && shootingTime <9){
        Drivetrain.getInstance().setPower(-0.25, -0.25);
      } else {
        Drivetrain.getInstance().setPower(0.0, 0.0);
      }
    }
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    subsystemController.onTeleLoop();
    subsystemController.updateSmartDashboard(); 

  }

  @Override
  public void testPeriodic() {
    
  }
}