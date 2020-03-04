package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Subsystems.ConveyorBelt;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Kickers;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter.ShooterMaster;
import frc.robot.Subsystems.Shooter.Turret;


public class Robot extends TimedRobot {
  // Drivetrain drivetrain;
  // Lift lift;
  // Intake intake;
  // ConveyorBelt conveyorBelt;
  // Kickers kickers;
  ShooterMaster shooter;  


  double startTime = 0;
  



  @Override
  public void robotInit() {
    // drivetrain    = Drivetrain.getInstance();
    // lift          = Lift.getInstance();
    // intake        = Intake.getInstance();
    // conveyorBelt  = ConveyorBelt.getInstance();
    // kickers       = Kickers.getInstance();
    shooter       = ShooterMaster.getInstance();
    
    // drivetrain.start();
    // lift.start();       
    // intake.start();
    // conveyorBelt.start();
    // kickers.start();  
    shooter.start();
  }

  @Override
  public void robotPeriodic() {     
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    // drivetrain.run();
    // lift.run();       
    // intake.run();
    // conveyorBelt.run();
    // kickers.run();  
    shooter.run();
  }


  @Override
  public void testPeriodic() {
    
  }
}