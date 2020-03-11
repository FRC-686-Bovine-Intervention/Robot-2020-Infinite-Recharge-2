package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.ControlStructures.RobotState;
import frc.robot.ControlStructures.RobotStateLoop;
import frc.robot.ControlStructures.SubsystemController;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.ConveyorBelt;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Kickers;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter.Flywheel;
import frc.robot.Subsystems.Shooter.Hood;
import frc.robot.Subsystems.Shooter.Limelight;
import frc.robot.Subsystems.Shooter.Limelight.LedMode;
import frc.robot.Subsystems.Shooter.ShooterCalcs;
import frc.robot.Subsystems.Shooter.ShooterMaster;
import frc.robot.Subsystems.Shooter.Turret;
import frc.robot.sensors.Pigeon;
import frc.robot.util.Vector2d;


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
    //Actually awful auto code but it works.
    subsystemController.updateSmartDashboard(); 
    Turret turret = Turret.getInstance();
    Hood hood = Hood.getInstance();
    Flywheel flywheel = Flywheel.getInstance();
    ConveyorBelt conveyorBelt = ConveyorBelt.getInstance();
    Kickers kickers = Kickers.getInstance();
    Intake intake = Intake.getInstance();
    Limelight limelight = Limelight.getInstance();

    if(!subsystemController.calibrationComplete()){
      subsystemController.runCalibration();
    } else {
      if(!autoStarted){
        autoStartTime = Timer.getFPGATimestamp();
        autoStarted = true;
        Kickers.getInstance().setPercent(Constants.kKickerShootPercent);
        Flywheel.getInstance().setRPS(3337/9.5493);
        Hood.getInstance().setPosition(Math.toRadians(47));
        limelight.setLEDMode(LedMode.kOn);
      }
      double shootingTime = Timer.getFPGATimestamp()-autoStartTime;
      if(shootingTime<5){
        Vector2d targetPos = ShooterCalcs.getTargetDisplacement();
        turret.setPosition((limelight.getTargetHorizontalAngleRad()/2.0)+turret.getSensedPosition());
        flywheel.setRPS(ShooterCalcs.calcShooterVelocity(targetPos.length()));
        hood.setPosition(ShooterCalcs.calcHoodPosition(targetPos.length()));
      }
      if(shootingTime > 5 & shootingTime <6){
        kickers.setPercent(0);
        hood.setPosition(0);
        flywheel.setRPS(0);
        limelight.setLEDMode(LedMode.kOff);
      }
  
      if(shootingTime > 5 && shootingTime < 9){
        Drivetrain.getInstance().setPower(0.25, 0.25);
        intake.setIntakePower(Constants.kIntakePower);
        intake.deploy();
      }
      if(shootingTime>9 && shootingTime<12){
        limelight.setLEDMode(LedMode.kOn);
        Vector2d targetPos = ShooterCalcs.getTargetDisplacement();
        turret.setPosition((limelight.getTargetHorizontalAngleRad()/2.0)+turret.getSensedPosition());
        flywheel.setRPS(ShooterCalcs.calcShooterVelocity(targetPos.length()));
        hood.setPosition(ShooterCalcs.calcHoodPosition(targetPos.length()));
        intake.retract();
        intake.setIntakePower(0.0);
        Drivetrain.getInstance().setPower(0.0, 0.0);
      }
      if((shootingTime>12 && shootingTime<16) || (shootingTime > 2 && shootingTime <5)){
        kickers.setPercent(Constants.kKickerShootPercent);
        conveyorBelt.turnOnTower();
        conveyorBelt.turnOnVBelt();
      } else {
        conveyorBelt.stopTower();
        conveyorBelt.stopVBelt();
      }
      if(shootingTime>16){
        limelight.setLEDMode(LedMode.kOff);
        flywheel.setRPS(0);
        turret.setPosition(0);
        hood.setPosition(0);
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