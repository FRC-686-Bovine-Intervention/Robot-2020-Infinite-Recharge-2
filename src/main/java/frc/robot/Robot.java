package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.ControlStructures.SubsystemController;
import frc.robot.Subsystems.ConveyorBelt;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Kickers;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter.ShooterMaster;


public class Robot extends TimedRobot {
  // Drivetrain drivetrain;
  // Lift lift;
  // Intake intake;
  // ConveyorBelt conveyorBelt;
  // Kickers kickers;
  ShooterMaster shooter;  
  UsbCamera ClimbCam = CameraServer.getInstance().startAutomaticCapture(); 
  CvSink cvSink = CameraServer.getInstance().getVideo();
  CvSource outputStream = CameraServer.getInstance().putVideo("ClimbCam", 640, 480);

  double startTime = 0;
  

  @Override
  public void robotInit() {
    subsystemController.addSubsystem(Drivetrain.getInstance());
    subsystemController.addSubsystem(ShooterMaster.getInstance());
    subsystemController.addSubsystem(ConveyorBelt.getInstance());
    subsystemController.addSubsystem(Intake.getInstance());
    subsystemController.addSubsystem(Kickers.getInstance());
    subsystemController.addSubsystem(Lift.getInstance());

    subsystemController.init();
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
  }


  @Override
  public void testPeriodic() {
    
  }
}