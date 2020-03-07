package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.ControlStructures.Subsystem;
import frc.robot.Controls.Controls;
import frc.robot.Controls.DriverControlsEnum;

public class Intake extends Subsystem{
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

    private double reverseCurrentThreshold = -25;
    private double reverseTime = 1;
    private double reverseStartTime = -1;

    public Intake(){
        controls = Controls.getInstance();

        intakeMotor = new TalonSRX(Constants.kIntakeTalonId);
        intakeMotor.setInverted(true);
        mainSolenoids = new DoubleSolenoid(Constants.kPCMID, Constants.kMainSolenoidFChannel, Constants.kMainSolenoidRChannel);
        secondarySolenoids = new DoubleSolenoid(Constants.kPCMID, Constants.kSecondarySolenoidFChannel, Constants.kSecondarySolenoidRChannel);
        
        intakeMotor.configFactoryDefault();
    }


    @Override
    public void init(){}

    @Override
    public void run(){
        System.out.println(intakeMotor.getStatorCurrent());
        if(controls.getBoolean(DriverControlsEnum.INTAKE)){
            deploy();
        } else if (Timer.getFPGATimestamp() - reverseStartTime >= reverseTime || reverseStartTime == -1)
        {
            retract();
            reverseStartTime = -1;
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void updateSmartDashboard() {

    }





    public void setIntakePower(double power){
        intakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void deploy(){
        if (intakeMotor.getStatorCurrent() <= reverseCurrentThreshold)
        {
            reverseStartTime = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - reverseStartTime >= reverseTime || reverseStartTime == -1)
        {
            intakeMotor.set(ControlMode.PercentOutput, Constants.kIntakePower);
            reverseStartTime = -1;
        }
        else
        {
            intakeMotor.set(ControlMode.PercentOutput, -Constants.kIntakePower);
        }
        mainSolenoids.set(Value.kReverse);
        secondarySolenoids.set(Value.kReverse);
    }

    public void retract(){
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
        mainSolenoids.set(Value.kForward);
        secondarySolenoids.set(Value.kForward);
    }

}