package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.robot.Controls.Controls;

public class Drivetrain {

    private static Drivetrain instance = null;
    public static Drivetrain getInstance(){
        if(instance == null){
            instance = new Drivetrain();
        }
        return instance;
    }

    private Controls controls;


    private TalonFX leftMaster, leftSlave, rightMaster, rightSlave;


    public Drivetrain(){
        controls = Controls.getInstance();

        leftMaster = new TalonFX(Constants.kLeftMotorMasterTalonId);
        leftSlave = new TalonFX(Constants.kLeftMotorSlave1TalonId);
        rightMaster = new TalonFX(Constants.kRightMotorMasterTalonId);
        rightSlave = new TalonFX(Constants.kRightMotorSlave1TalonId);

        leftMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();

        leftMaster.setInverted(true);
        leftSlave.setInverted(true);
        rightMaster.setInverted(false);
        rightSlave.setInverted(false);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
    }

    public void start(){}


    public void run(){
        double leftPower = (controls.getYAxis() +controls.getXAxis())/2.0;
        double rightPower = (controls.getYAxis()-controls.getXAxis())/2.0;
        setPower(leftPower, rightPower);
    }


    public void setPower(double leftPower, double rightPower){
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
    }
}