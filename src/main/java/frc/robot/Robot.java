package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.controllers.LoopController;
import frc.robot.controllers.RobotState;
import frc.robot.controllers.RobotStateLoop;
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
  LoopController loopController;
  Pigeon pigeon = Pigeon.getInstance();

  @Override
  public void disabledInit() {
    Limelight.getInstance().setLEDMode(LedMode.kOff);
  }

  @Override
  public void robotInit() {
    pigeon.zeroSensor();

    loopController = new LoopController();
    loopController.register(Drivetrain.getInstance());
    loopController.register(ShooterMaster.getInstance());
    loopController.register(Intake.getInstance());
    loopController.register(ConveyorBelt.getInstance());
    loopController.register(Camera.getInstance());
    loopController.register(Kickers.getInstance());
    loopController.register(Lift.getInstance());
    loopController.register(RobotStateLoop.getInstance());
    loopController.start();

    //Move to auto!================================================================
    RobotState.getInstance().reset(FieldDimensions.middleStartPose);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}
}