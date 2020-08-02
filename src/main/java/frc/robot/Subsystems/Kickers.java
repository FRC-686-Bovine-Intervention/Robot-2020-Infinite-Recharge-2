package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.controllers.Subsystem;
import frc.robot.joysticks.Controls;
import frc.robot.joysticks.DriverControlsEnum;
import frc.robot.util.FallingEdgeDetector;
import frc.robot.util.RisingEdgeDetector;

public class Kickers extends Subsystem{
    private static Kickers instance = null;
    public static Kickers getInstance(){
        if(instance == null){
            instance = new Kickers();
        }
        return instance;
    }

    private Controls controls;

    private VictorSPX kickerMaster, kickerSlave;

    private RisingEdgeDetector shootRise = new RisingEdgeDetector();
    private FallingEdgeDetector shootFall = new FallingEdgeDetector();


    public Kickers(){
        controls = Controls.getInstance();

        kickerMaster = new VictorSPX(Constants.kConveyorKickerMasterID);
        kickerSlave = new VictorSPX(Constants.kConveyorKickerSlaveID);

        kickerMaster.configFactoryDefault();
        kickerSlave.configFactoryDefault();

        kickerMaster.setInverted(true);
        kickerSlave.setInverted(false);

        kickerSlave.follow(kickerMaster);

        SmartDashboard.putBoolean("Kickers/Debug", false);
        SmartDashboard.putNumber("Kickers/Debug/SetPercent", 0);
    }

    @Override
    public void init(){}

    @Override
    public void run(){
        if(!SmartDashboard.getBoolean("Kickers/Debug", false)){
            if(shootRise.update(controls.getBoolean(DriverControlsEnum.SHOOT))){
                kickerMaster.set(ControlMode.PercentOutput, Constants.kKickerShootPercent);
            }
            if(shootFall.update(controls.getBoolean(DriverControlsEnum.SHOOT))){
                kickerMaster.set(ControlMode.PercentOutput, 0.0);
            }
        } else {
            kickerMaster.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Kickers/Debug/SetPercent", 0));
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void updateSmartDashboard() {

    }


    public void setPercent(double percent){
        kickerMaster.set(ControlMode.PercentOutput,percent);
    }
}