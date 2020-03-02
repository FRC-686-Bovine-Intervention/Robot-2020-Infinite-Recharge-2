package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Turret {
    private static Turret instance = null;
    public static Turret getInstance(){
        if(instance == null){
            instance = new Turret();
        }
        return instance;
    }

    private TalonSRX turretMotor;

    private static final double kF = 0.0;
    private static final double kP = 5.0;  
    private static final double kI = 0.0;
    private static final double kD = 150.0; 

    private static final double encoderUnitsPerRev = 49050;

    private static final int tolerance = Utility.radiansToEncoderUnits(0.075, encoderUnitsPerRev);
    private static final int cruiseVelocity = Utility.rpsToUPF(1, encoderUnitsPerRev);
    private static final int maxAcceleration = Utility.rpspsToUPFPF(1, encoderUnitsPerRev);

    private static final double limits[] = {-(Math.PI), (Math.PI)};


    public Turret(){
        turretMotor = new TalonSRX(Constants.kShooterTurretID);

        turretMotor.configFactoryDefault();
        turretMotor.setInverted(true);
        turretMotor.setSensorPhase(false);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTalonTimeoutMs);

        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        turretMotor.selectProfileSlot(Constants.kSlotIdxPos, Constants.kTalonPidIdx); 
        turretMotor.config_kF(Constants.kSlotIdxPos, kF, Constants.kTalonTimeoutMs); 
        turretMotor.config_kP(Constants.kSlotIdxPos, kP, Constants.kTalonTimeoutMs); 
        turretMotor.config_kI(Constants.kSlotIdxPos, kI, Constants.kTalonTimeoutMs); 
        turretMotor.config_kD(Constants.kSlotIdxPos, kD, Constants.kTalonTimeoutMs);
        turretMotor.configAllowableClosedloopError(Constants.kSlotIdxPos, tolerance, Constants.kTalonTimeoutMs);

        turretMotor.configMotionCruiseVelocity(cruiseVelocity, Constants.kTalonTimeoutMs);
        turretMotor.configMotionAcceleration(maxAcceleration, Constants.kTalonTimeoutMs);
    }





    public void setPosition(double radians){
        System.out.println(radians);
        radians = Utility.bound(radians, limits);
        turretMotor.set(ControlMode.Position, Utility.radiansToEncoderUnits(radians, encoderUnitsPerRev));
    }

    public void setPercent(double percent){
        turretMotor.set(ControlMode.PercentOutput, percent);
    }


    public double getSensedPosition(){
        return Utility.encoderUnitsToRadians(turretMotor.getSelectedSensorPosition(), encoderUnitsPerRev);
    }


    public void zeroWithInit(double radians){
        turretMotor.setSelectedSensorPosition(Utility.radiansToEncoderUnits(radians, encoderUnitsPerRev), 0, Constants.kTalonPidIdx);
    }
}