package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.controllers.RobotState;
import frc.robot.controllers.RobotStateLoop;
import frc.robot.controllers.SubsystemController;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kickers;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.shooter.Limelight;
import frc.robot.subsystems.shooter.Limelight.LedMode;
import frc.robot.subsystems.shooter.ShooterMaster;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.lib.sensors.Pigeon;


public class Robot extends TimedRobot {
  SubsystemController subsystemController;
  Pigeon pigeon = Pigeon.getInstance();

  private double autoStartTime = 0;
  private boolean autoStarted = false;  
  private boolean calibrationLoop = false;
  private boolean calibrationCatch = false;

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
    subsystemController.addSubsystem(Intake.getInstance());
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
    calibrationCatch = calibrationLoop;
    calibrationLoop = false; 
  }

  @Override
  public void autonomousInit() {
    subsystemController.resetForCalibration();
    autoStarted = false;
  }

  @Override
  public void autonomousPeriodic() {
    
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
    calibrationLoop = true;
    if(!calibrationCatch){
      subsystemController.resetForCalibration();
    } else {
      subsystemController.runCalibration();
    }
    if(subsystemController.calibrationComplete()){
      Turret.getInstance().setPosition(0);
    }
  }
}