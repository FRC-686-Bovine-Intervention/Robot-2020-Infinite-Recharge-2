package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.util.FallingEdgeDetector;
import frc.robot.lib.util.RisingEdgeDetector;

public class Kickers implements Loop {
    private static Kickers instance = null;
    public static Kickers getInstance(){
        if(instance == null){
            instance = new Kickers();
        }
        return instance;
    }

    private DriverControlsBase controls;

    private VictorSPX kickerMaster, kickerSlave;

    private RisingEdgeDetector shootRise = new RisingEdgeDetector();
    private FallingEdgeDetector shootFall = new FallingEdgeDetector();


    public Kickers(){
        controls = SelectedDriverControls.getInstance().get();

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
    public void onStart(){}

    @Override
    public void onLoop(){
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
    public void onStop(){}

    public void setPercent(double percent){
        kickerMaster.set(ControlMode.PercentOutput,percent);
    }
}