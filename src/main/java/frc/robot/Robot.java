package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.ControlStructures.SubsystemController;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.ConveyorBelt;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Kickers;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter.ShooterMaster;


public class Robot extends TimedRobot {
  SubsystemController subsystemController;

  @Override
  public void robotInit() {
    subsystemController.addSubsystem(Drivetrain.getInstance());
    subsystemController.addSubsystem(ShooterMaster.getInstance());
    subsystemController.addSubsystem(ConveyorBelt.getInstance());
    subsystemController.addSubsystem(Camera.getInstance());
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