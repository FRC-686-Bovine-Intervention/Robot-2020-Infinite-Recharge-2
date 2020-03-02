package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Hood {
    private static Hood instance = null;
    public static Hood getInstance(){
        if(instance == null){
            instance = new Hood();
        }
        return instance;
    }

    private TalonSRX hoodMotor;


    private static final double kF = 0.0;
    private static final double kI = 10.2;
    private static final double kD = 0.0;
    private static final double kP = 32.0;

    private static final double encoderUnitsPerRev = 104424;

    private static final int tolerance = Utility.radiansToEncoderUnits(0.025, encoderUnitsPerRev);
    private static final int cruiseVelocity = Utility.rpsToUPF(1, encoderUnitsPerRev);
    private static final int maxAcceleration = Utility.rpsToUPF(0, 100);

    private static final double[] limits = {0, Math.PI/4.0};




    public Hood(){
        hoodMotor = new TalonSRX(Constants.kShooterHoodID);
        hoodMotor.configFactoryDefault();
        hoodMotor.setInverted(true);
        hoodMotor.setSensorPhase(false);
        hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTalonTimeoutMs);

        hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		hoodMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        hoodMotor.selectProfileSlot(Constants.kSlotIdxPos, Constants.kTalonPidIdx); 
        hoodMotor.config_kF(Constants.kSlotIdxPos, kF, Constants.kTalonTimeoutMs); 
        hoodMotor.config_kP(Constants.kSlotIdxPos, kP, Constants.kTalonTimeoutMs); 
        hoodMotor.config_kI(Constants.kSlotIdxPos, kI, Constants.kTalonTimeoutMs); 
        hoodMotor.config_kD(Constants.kSlotIdxPos, kD, Constants.kTalonTimeoutMs);
        hoodMotor.configAllowableClosedloopError(Constants.kSlotIdxPos, tolerance, Constants.kTalonTimeoutMs);

        hoodMotor.configMotionCruiseVelocity(cruiseVelocity, Constants.kTalonTimeoutMs);
        hoodMotor.configMotionAcceleration(maxAcceleration, Constants.kTalonTimeoutMs);
    }


    public void setPosition(double radians){
        radians = Utility.bound(radians, limits);
        hoodMotor.set(ControlMode.Position, Utility.radiansToEncoderUnits(radians, encoderUnitsPerRev));
    }

    public void setPercent(double percent){
        hoodMotor.set(ControlMode.PercentOutput, percent);
    }




    public double getSensedPos(){
        return Utility.encoderUnitsToRadians(hoodMotor.getSelectedSensorPosition(), encoderUnitsPerRev);
    }



    public void zeroSensor(){
        hoodMotor.setSelectedSensorPosition(0, 0, Constants.kTalonTimeoutMs);
    }







}