package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.controllers.Loop;
import frc.robot.lib.joysticks.Controls;
import frc.robot.lib.joysticks.DriverControlsEnum;
import frc.robot.subsystems.shooter.ShooterMaster;
import frc.robot.lib.util.RisingEdgeDetector;

public class ConveyorBelt implements Loop {
    private static ConveyorBelt instance = null;

    public static ConveyorBelt getInstance() {
        if (instance == null) {
            instance = new ConveyorBelt();
        }
        return instance;
    }

    private Controls controls;

    private VictorSPX towerMaster, towerSlave, vBeltRight;
    public TalonSRX vBeltLeft;

    private DigitalInput entranceSensor, exitSensor;

    private RisingEdgeDetector shootEdge = new RisingEdgeDetector();

    private static final double reverseTime = 0.125;
    private double reverseStartTime = 0;

    private boolean shooterChecked = false;

    private int storageCount = 0;

    public ConveyorBelt() {
        controls = Controls.getInstance();

        entranceSensor = new DigitalInput(Constants.kEntranceProximityID);
        exitSensor = new DigitalInput(Constants.kExitProximityID);

        towerMaster = new VictorSPX(Constants.kConveyorbeltMasterID);
        towerSlave = new VictorSPX(Constants.kConveyorbeltSlaveID);
        vBeltLeft = new TalonSRX(Constants.kConveyorHopperLeftID);
        vBeltRight = new VictorSPX(Constants.kConveyorHopperRightID);

        towerMaster.configFactoryDefault();
        towerSlave.configFactoryDefault();
        vBeltLeft.configFactoryDefault();
        vBeltRight.configFactoryDefault();

        towerMaster.setInverted(false);
        towerSlave.setInverted(true);
        vBeltLeft.setInverted(true);
        vBeltRight.setInverted(true);

        towerSlave.follow(towerMaster);

        SmartDashboard.putBoolean("Conveyorbelt/Debug", false);
        SmartDashboard.putNumber("Conveyorbelt/Debug/SetTowerPercent", 0);
        SmartDashboard.putNumber("Conveyorbelt/Debug/SetLeftVPercent", 0);
        SmartDashboard.putNumber("Conveyorbelt/Debug/SetRightVPercent", 0);
    }


    @Override
    public void onStart() {}

    @Override
    public void onLoop() {
        if (!SmartDashboard.getBoolean("Conveyorbelt/Debug", false)) {
            if (controls.getBoolean(DriverControlsEnum.REVERSE_BELTS)) {
                reverseTower();
                reverseVBelt();
            } else {
                if (shootEdge.update(controls.getBoolean(DriverControlsEnum.SHOOT))) {
                    reverseTower();
                    reverseVBelt();
                    reverseStartTime = Timer.getFPGATimestamp();
                }

                if (controls.getBoolean(DriverControlsEnum.SHOOT)) {
                    if (shooterChecked) {
                        turnOnTower();
                        turnOnVBelt();
                    } else {
                        shooterChecked = ShooterMaster.getInstance().readyToShoot();
                        if (Timer.getFPGATimestamp() - reverseStartTime >= reverseTime) {
                            stopTower();
                            stopVBelt();
                        }
                    }
                } else {
                    shooterChecked = false;
                    if (exitSensor.get() && storageCount < 3) {
                        turnOnVBelt();
                        if (!entranceSensor.get()) {
                            turnOnTower();
                        } else {
                            stopTower();
                        }
                    } else {
                        stopTower();
                        stopVBelt();
                    }
                }
            }
        } else {
            towerMaster.set(ControlMode.PercentOutput,
                    SmartDashboard.getNumber("Conveyorbelt/Debug/SetTowerPercent", 0));
            vBeltLeft.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Conveyorbelt/Debug/SetLeftVPercent", 0));
            vBeltRight.set(ControlMode.PercentOutput,
                    SmartDashboard.getNumber("Conveyorbelt/Debug/SetRightVPercent", 0));
        }
    }

    @Override
    public void onStop(){}

    public void feed() {
        turnOnTower();
        setVBeltPercent(Constants.kLeftHopperPercent, Constants.kRightHopperPercent);
    }

    public void reverseTower() {
        setTowerPercent(-Constants.kConveyorBackUpPercent);
    }

    public void turnOnTower() {
        setTowerPercent(Constants.kConveyorFeedPercent);
    }

    public void stopTower() {
        setTowerPercent(0.0);
    }

    public void setTowerPercent(double percent) {
        towerMaster.set(ControlMode.PercentOutput, percent);
    }

    public void reverseVBelt() {
        setVBeltPercent(-Constants.kLeftHopperPercent, -Constants.kRightHopperPercent);
    }

    public void turnOnVBelt() {
        setVBeltPercent(Constants.kLeftHopperPercent, Constants.kRightHopperPercent);
    }

    public void stopVBelt() {
        setVBeltPercent(0.0, 0.0);
    }

    public void setVBeltPercent(double leftPercent, double rightPercent) {
        vBeltLeft.set(ControlMode.PercentOutput, leftPercent);
        vBeltRight.set(ControlMode.PercentOutput, rightPercent);
    }
}


