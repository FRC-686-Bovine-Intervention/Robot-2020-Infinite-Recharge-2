package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.robot.ControlStructures.Subsystem;
import frc.robot.Controls.Controls;
import frc.robot.Subsystems.Lift.PTOStates;
import frc.robot.Subsystems.Shooter.Utility;

public class Drivetrain extends Subsystem{

    private static Drivetrain instance = null;
    public static Drivetrain getInstance(){
        if(instance == null){
            instance = new Drivetrain();
        }
        return instance;
    }

    private Controls controls;

    private TalonFX leftMaster, leftSlave, rightMaster, rightSlave;

    private static final double kFLeft = 0;
    private static final double kPLeft = 0;
    private static final double kILeft = 0;
    private static final double kDLeft = 0;
    private static final int leftTolerance = 0;
    private static final double encoderUnitsPerRevLeft = 1000.0;
    
    private static final double kFRight = 0;
    private static final double kPRight = 0;
    private static final double kIRight = 0;
    private static final double kDRight = 0;
    private static final int rightTolerance = 0;
    private static final double encoderUnitsPerRevRight = 1000.0;




    //Physical parameters
    public static final double wheelDiameter = 6.0; //inches 




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



        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonFXPidIdx, Constants.kTalonTimeoutMs);
        leftMaster.setSelectedSensorPosition(0, Constants.kTalonFXPidIdx, Constants.kTalonTimeoutMs);

        leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        leftMaster.selectProfileSlot(Constants.kSlotIdxSpeed, Constants.kTalonFXPidIdx); 
        leftMaster.config_kF(Constants.kSlotIdxSpeed, kFLeft, Constants.kTalonTimeoutMs); 
        leftMaster.config_kP(Constants.kSlotIdxSpeed, kPLeft, Constants.kTalonTimeoutMs); 
        leftMaster.config_kI(Constants.kSlotIdxSpeed, kILeft, Constants.kTalonTimeoutMs); 
        leftMaster.config_kD(Constants.kSlotIdxSpeed, kDLeft, Constants.kTalonTimeoutMs);
        leftMaster.configAllowableClosedloopError(Constants.kSlotIdxSpeed, leftTolerance, Constants.kTalonTimeoutMs);


        
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonFXPidIdx, Constants.kTalonTimeoutMs);
        rightMaster.setSelectedSensorPosition(0, Constants.kTalonFXPidIdx, Constants.kTalonTimeoutMs);

        rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        rightMaster.selectProfileSlot(Constants.kSlotIdxSpeed, Constants.kTalonFXPidIdx); 
        rightMaster.config_kF(Constants.kSlotIdxSpeed, kFRight, Constants.kTalonTimeoutMs); 
        rightMaster.config_kP(Constants.kSlotIdxSpeed, kPRight, Constants.kTalonTimeoutMs); 
        rightMaster.config_kI(Constants.kSlotIdxSpeed, kIRight, Constants.kTalonTimeoutMs); 
        rightMaster.config_kD(Constants.kSlotIdxSpeed, kDRight, Constants.kTalonTimeoutMs);
        rightMaster.configAllowableClosedloopError(Constants.kSlotIdxSpeed, rightTolerance, Constants.kTalonTimeoutMs);
    }

    

    @Override
    public void init(){}


    @Override
    public void run(){
        double leftPower, rightPower;
        if(Lift.getInstance().getPTOState() == PTOStates.DRIVE_ENABLED){
            leftPower = (controls.getYAxis() +controls.getXAxis())/2.0;
            rightPower = (controls.getYAxis()-controls.getXAxis())/2.0;
        } else {
            leftPower = rightPower = 0;
        }
        setPower(leftPower, rightPower);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void updateSmartDashboard() {

    }


    public void setPower(double leftPower, double rightPower){
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
    }

    /**
     * Sets velocity of drivetrain in inches/sec
     */
    public void setVelocity(double leftVel, double rightVel){
        leftVel /= (wheelDiameter/2.0);
        rightVel /= (wheelDiameter/2.0);
        leftVel = Utility.rpsToUPF(leftVel, encoderUnitsPerRevLeft);
        rightVel = Utility.rpsToUPF(rightVel, encoderUnitsPerRevRight);

        leftMaster.set(ControlMode.Velocity, leftVel);
        rightMaster.set(ControlMode.Velocity, rightVel);
    }

    public WheelSpeed getSensedVelocity(){
        return new WheelSpeed(getSensedLeftVelocity(), getSensedRightVelocity());
    }

    public double getSensedLeftVelocity(){
        return (wheelDiameter/2.0)*Utility.upfToRPS(leftMaster.getSelectedSensorVelocity(), encoderUnitsPerRevLeft);
    }

    public double getSensedRightVelocity(){
        return (wheelDiameter/2.0)*Utility.upfToRPS(rightMaster.getSelectedSensorVelocity(), encoderUnitsPerRevRight);
    }





    class WheelSpeed {
        double leftSpeed, rightSpeed;
        public WheelSpeed(double leftSpeed, double rightSpeed){
            this.leftSpeed = leftSpeed;
            this.rightSpeed = rightSpeed;
        }
    }


}