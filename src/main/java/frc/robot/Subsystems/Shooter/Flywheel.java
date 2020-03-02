package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class Flywheel {
    private static Flywheel instance = null;
    public static Flywheel getInstance(){
        if(instance == null){
            instance = new Flywheel();
        }
        return instance;
    }


    TalonFX flywheelMaster, flywheelSlave;


    private static final double kF = 0.03072071;
    private static final double kP = 0.25;
    private static final double kI = 0.0;
    private static final double kD = 24.0;

    private static final double encoderUnitsPerRev = 1540;

    private static final int tolerance = Utility.rpsToUPF(10, encoderUnitsPerRev);
    private static final double accelerationTime = 3; //secs


    private double targetRPS = 0;
    private static final double rpsTolerance = 20;


    public Flywheel(){
        flywheelMaster = new TalonFX(Constants.kShooterTalonId);
        flywheelSlave = new TalonFX(Constants.kShooterSlaveId);

        flywheelSlave.configFactoryDefault();
        flywheelSlave.setInverted(false);

        flywheelMaster.configFactoryDefault();
        flywheelMaster.setInverted(true);
        flywheelMaster.setSensorPhase(false);
        flywheelMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 1, Constants.kTalonTimeoutMs);

        flywheelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		flywheelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		flywheelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		flywheelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        flywheelMaster.selectProfileSlot(Constants.kSlotIdxPos, Constants.kTalonPidIdx); 
        flywheelMaster.config_kF(Constants.kSlotIdxPos, kF, Constants.kTalonTimeoutMs); 
        flywheelMaster.config_kP(Constants.kSlotIdxPos, kP, Constants.kTalonTimeoutMs); 
        flywheelMaster.config_kI(Constants.kSlotIdxPos, kI, Constants.kTalonTimeoutMs); 
        flywheelMaster.config_kD(Constants.kSlotIdxPos, kD, Constants.kTalonTimeoutMs);
        flywheelMaster.configAllowableClosedloopError(Constants.kSlotIdxPos, tolerance, Constants.kTalonTimeoutMs);

        flywheelMaster.configClosedloopRamp(accelerationTime);


        flywheelSlave.follow(flywheelMaster);
    }






    public void setRPS(double rps){
        flywheelMaster.set(ControlMode.Velocity, Utility.rpsToUPF(rps, encoderUnitsPerRev));
        targetRPS = rps;
    }




    public boolean nearTarget(){
        if(targetRPS != 0){
            double error = targetRPS-getSensedRPS();
            return Math.abs(error) <= rpsTolerance;
        } else {
            return false;
        }
    }



    public double getSensedRPS(){
        return Utility.upfToRPS(flywheelMaster.getSelectedSensorVelocity(), encoderUnitsPerRev);
    }





}