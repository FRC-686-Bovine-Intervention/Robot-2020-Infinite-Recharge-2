package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.RobotState;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.sensors.Pigeon;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.LoopController;
import frc.robot.loops.RobotStateLoop;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kickers;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.shooter.Limelight;
import frc.robot.subsystems.shooter.Limelight.LedMode;
import frc.robot.subsystems.shooter.ShooterMaster;

public class Robot extends TimedRobot {
  LoopController loopController;
  Pigeon pigeon = Pigeon.getInstance();
  SmartDashboardInteractions smartDashboard = SmartDashboardInteractions.getInstance();

  @Override
  public void disabledInit() {
    Limelight.getInstance().setLEDMode(LedMode.kOff);
  }

  @Override
  public void robotInit() {
    pigeon.zeroSensor();

    loopController = new LoopController();
    loopController.register(DriveLoop.getInstance());
    loopController.register(Drive.getInstance().getVelocityPIDLoop());
    loopController.register(ShooterMaster.getInstance());
    loopController.register(Intake.getInstance());
    loopController.register(ConveyorBelt.getInstance());
    // loopController.register(Camera.getInstance());
    loopController.register(Kickers.getInstance());
    loopController.register(Lift.getInstance());
    // loopController.register(RobotStateLoop.getInstance());
    loopController.start();

    //Move to auto!================================================================
    RobotState.getInstance().reset(FieldDimensions.middleStartPose);
    SelectedDriverControls.getInstance().setDriverControls( SmartDashboardInteractions.getInstance().getDriverControlsSelection() );

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    loopController.start();
  }

  @Override
  public void teleopPeriodic() {
    // TODO: I'm not sure which of the two line below is correct, but I'm fairly certain that the second will work (I can't test rn)
    // Drive.getInstance().setOpenLoop(SelectedDriverControls.getInstance().getDriveCommand());
    Drive.getInstance().setCommand(SelectedDriverControls.getInstance().getDriveCommand());
    loopController.run();

  }

  @Override
  public void testPeriodic() {
    Limelight.getInstance().setLEDMode(LedMode.kOn);
  }
}