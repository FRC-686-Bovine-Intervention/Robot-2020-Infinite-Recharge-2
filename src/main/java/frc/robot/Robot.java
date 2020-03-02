/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ResourceBundle.Control;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controls.Controls;
import frc.robot.Controls.DriverControlsEnum;
import frc.robot.Subsystems.ConveyorBelt;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Kickers;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter.Flywheel;
import frc.robot.Subsystems.Shooter.Hood;
import frc.robot.Subsystems.Shooter.Limelight;
import frc.robot.Subsystems.Shooter.ShooterMaster;
import frc.robot.Subsystems.Shooter.Turret;
import frc.robot.Subsystems.Shooter.Limelight.LedMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Drivetrain drivetrain;
  Lift lift;
  Intake intake;
  ConveyorBelt conveyorBelt;
  Kickers kickers;
  //ShooterMaster shooter;
  Turret turret;
  Flywheel flywheel;
  Hood hood;

  DigitalInput hallEffect;
  


  double startTime = 0;
  



  @Override
  public void robotInit() {
    drivetrain    = Drivetrain.getInstance();
    lift          = Lift.getInstance();
    intake        = Intake.getInstance();
    conveyorBelt  = ConveyorBelt.getInstance();
    kickers       = Kickers.getInstance();
    //shooter       = ShooterMaster.getInstance();
    turret = Turret.getInstance();
    hood = Hood.getInstance();
    flywheel = Flywheel.getInstance();

    
    drivetrain.start();
    lift.start();       
    intake.start();
    conveyorBelt.start();
    kickers.start();  
    //shooter.start();

    hallEffect = new DigitalInput(Constants.kTurretHallEffectChannel);
  }

  @Override
  public void robotPeriodic() {     
  }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    kickers.setPercent(Constants.kKickerShootPercent);
    flywheel.setRPS(275);
    hood.setPosition(Math.toRadians(22));
  }

  @Override
  public void autonomousPeriodic() {
    double shootingTime = Timer.getFPGATimestamp()-startTime;
    //wait(2);

    if(shootingTime > 6){
      kickers.setPercent(0);
      flywheel.setRPS(0);
    }

    if(shootingTime > 1 && shootingTime <6){
      conveyorBelt.turnOnTower();
      conveyorBelt.turnOnVBelt();
    } else {
      conveyorBelt.stopTower();
      conveyorBelt.stopVBelt();
    }

    if(shootingTime > 6 && shootingTime <9){
      drivetrain.setPower(-0.25, -0.25);
    } else {
      drivetrain.setPower(0.0, 0.0);
    }
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.run();
    lift.run();       
    intake.run();
    conveyorBelt.run();
    kickers.run();  
    //shooter.run();

    Limelight limelight = Limelight.getInstance();

    if(Controls.getInstance().getBoolean(DriverControlsEnum.SHOOT)){
      limelight.setLEDMode(LedMode.kOn);
      if(Controls.getInstance().getBoolean(DriverControlsEnum.MAX_HOOD)){
        hood.setPosition(Math.toRadians(45));
      } else {
        hood.setPosition(Math.toRadians(22));
      }

      


      
      flywheel.setRPS(275);

    } else {
      flywheel.setRPS(0);
      hood.setPosition(0);
      limelight.setLEDMode(LedMode.kOff);
    }

    turret.setPosition(0);

  }


  boolean hoodCalibrated, turretCalibrated, allCalibrated;
  double hoodLastPos = 0;
  double hoodCalibTolerance = 0.025;
  int successfulHoodLoops = 0;
  int requiredHoodLoops = 3;

  @Override
  public void testPeriodic() {
    if(hallEffect.get() && !turretCalibrated){
        turret.setPercent(-0.1825);
    } else {
        turret.setPercent(0.0);
        turretCalibrated = true;
        turret.zeroWithInit(-3);
    }

    if(Math.abs(hood.getSensedPos()-hoodLastPos) <= hoodCalibTolerance && !hoodCalibrated){
        successfulHoodLoops++;
    } else {
        successfulHoodLoops = 0;
    }
    if(!hoodCalibrated && successfulHoodLoops < requiredHoodLoops){
        hood.setPercent(-0.1825);
    } else {
        hood.setPercent(0.0);
        hood.zeroSensor();
        hoodCalibrated = true;
    }

    if(hoodCalibrated && turretCalibrated){
        allCalibrated = true;
        return;
    }
  }
}
