package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.loops.Loop;
import frc.robot.lib.joysticks.Controls;
import frc.robot.lib.joysticks.DriverControlsEnum;

public class Intake implements Loop{
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

    private double reverseCurrentThreshold = 45;
    private double reverseTime = 0.25;
    private double reverseStartTime = -1;

    public Intake(){
        controls = Controls.getInstance();

        intakeMotor = new TalonSRX(Constants.kIntakeTalonId);
        intakeMotor.configFactoryDefault();
        intakeMotor.setInverted(true);
        intakeMotor.configOpenloopRamp(0.075, Constants.kTalonTimeoutMs);
        mainSolenoids = new DoubleSolenoid(Constants.kPCMID, Constants.kMainSolenoidFChannel, Constants.kMainSolenoidRChannel);
        secondarySolenoids = new DoubleSolenoid(Constants.kPCMID, Constants.kSecondarySolenoidFChannel, Constants.kSecondarySolenoidRChannel);
        
    }

    @Override
    public void onStart(){}

    @Override
    public void onLoop(){
        double reverseElapsedTime = Timer.getFPGATimestamp()-reverseStartTime;

        if(controls.getBoolean(DriverControlsEnum.INTAKE)){
            deploy();
            if(reverseElapsedTime < reverseTime){
                setIntakePower(-Constants.kIntakePower);
            } else {
                setIntakePower(Constants.kIntakePower);
            }
        } else {
            if(reverseElapsedTime > reverseTime){
                retract();
            }
        }

        if(getCurrent() >= reverseCurrentThreshold){
            reverseStartTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void onStop(){}

    public void setIntakePower(double power){
        intakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void deploy(){
        mainSolenoids.set(Value.kReverse);
        secondarySolenoids.set(Value.kReverse);
    }

    public void retract(){
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
        mainSolenoids.set(Value.kForward);
        secondarySolenoids.set(Value.kForward);
    }

    public double getCurrent(){
        return intakeMotor.getStatorCurrent();
    }
}