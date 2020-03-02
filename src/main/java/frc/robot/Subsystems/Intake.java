package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.Controls.Controls;
import frc.robot.Controls.DriverControlsEnum;

public class Intake {
    private static Intake instance = null;
    public static Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }

    private Controls controls;

    private TalonSRX intakeMotor;
    private DoubleSolenoid mainSolenoids, secondarySolenoids;


    public Intake(){
        controls = Controls.getInstance();

        intakeMotor = new TalonSRX(Constants.kIntakeTalonId);
        intakeMotor.setInverted(true);
        mainSolenoids = new DoubleSolenoid(Constants.kPCMID, Constants.kMainSolenoidFChannel, Constants.kMainSolenoidRChannel);
        secondarySolenoids = new DoubleSolenoid(Constants.kPCMID, Constants.kSecondarySolenoidFChannel, Constants.kSecondarySolenoidRChannel);
        
        intakeMotor.configFactoryDefault();
    }


    public void start(){}

    public void run(){
        if(controls.getBoolean(DriverControlsEnum.INTAKE)){
            deploy();
        } else {
            retract();
        }
    }





    public void setIntakePower(double power){
        intakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void deploy(){
        intakeMotor.set(ControlMode.PercentOutput, Constants.kIntakePower);
        mainSolenoids.set(Value.kReverse);
        secondarySolenoids.set(Value.kReverse);
    }

    public void retract(){
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
        mainSolenoids.set(Value.kForward);
        secondarySolenoids.set(Value.kForward);
    }






}