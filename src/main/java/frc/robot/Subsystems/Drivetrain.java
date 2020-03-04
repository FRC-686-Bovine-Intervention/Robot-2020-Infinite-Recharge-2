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

    private static final double kFPosLeft = 0;
    private static final double kPPosLeft = 0.32;
    private static final double kIPosLeft = 0;
    private static final double kDPosLeft = 3.2;
    private static final int leftTolerancePos = 10;

    private static final double kFVelLeft = 0.05; //0.05;
    private static final double kPVelLeft = 0.006; //0.032;
    private static final double kIVelLeft = 0.0; //0;
    private static final double kDVelLeft = 0.25; //2.56;
    private static final int leftToleranceVel = 0;
    
    private static final double encoderUnitsPerRevLeft = 24593.0;


    
    private static final double kFPosRight = 0;
    private static final double kPPosRight = 0.32;
    private static final double kIPosRight = 0;
    private static final double kDPosRight = 3.2;
    private static final int rightTolerancePos = 10;

    private static final double kFVelRight = 0.05;
    private static final double kPVelRight = 0.006;
    private static final double kIVelRight = 0.0;
    private static final double kDVelRight = 0.25;
    private static final int rightToleranceVel = 0;

    private static final double encoderUnitsPerRevRight = 23763.0;



    //Physical parameters
    public static final double wheelDiameter = 6.0; //inches 
    public static final double extensionsPerRev = 1000; //In/rev


    //20610




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


        //==============================
        //Left Motor Configurations
        //==============================
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
        leftMaster.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);

        leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
        
        //Configure position loop PID
        leftMaster.selectProfileSlot(Constants.kSlotIdxPos, Constants.kTalonPidIdx); 
        leftMaster.config_kF(Constants.kSlotIdxPos, kFPosLeft, Constants.kTalonTimeoutMs); 
        leftMaster.config_kP(Constants.kSlotIdxPos, kPPosLeft, Constants.kTalonTimeoutMs); 
        leftMaster.config_kI(Constants.kSlotIdxPos, kIPosLeft, Constants.kTalonTimeoutMs); 
        leftMaster.config_kD(Constants.kSlotIdxPos, kDPosLeft, Constants.kTalonTimeoutMs);
        leftMaster.configAllowableClosedloopError(Constants.kSlotIdxPos, leftTolerancePos, Constants.kTalonTimeoutMs);
        //Configure velocity loop PID 
        leftMaster.selectProfileSlot(Constants.kSlotIdxSpeed, Constants.kTalonPidIdx); 
        leftMaster.config_kF(Constants.kSlotIdxSpeed, kFVelLeft, Constants.kTalonTimeoutMs); 
        leftMaster.config_kP(Constants.kSlotIdxSpeed, kPVelLeft, Constants.kTalonTimeoutMs); 
        leftMaster.config_kI(Constants.kSlotIdxSpeed, kIVelLeft, Constants.kTalonTimeoutMs); 
        leftMaster.config_kD(Constants.kSlotIdxSpeed, kDVelLeft, Constants.kTalonTimeoutMs);
        leftMaster.configAllowableClosedloopError(Constants.kSlotIdxSpeed, leftToleranceVel, Constants.kTalonTimeoutMs);


        //==============================
        // Right motor configurations
        //==============================
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
        rightMaster.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);

        rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
        
        //Configure position loop PID
        rightMaster.selectProfileSlot(Constants.kSlotIdxPos, Constants.kTalonPidIdx); 
        rightMaster.config_kF(Constants.kSlotIdxPos, kFPosRight, Constants.kTalonTimeoutMs); 
        rightMaster.config_kP(Constants.kSlotIdxPos, kPPosRight, Constants.kTalonTimeoutMs); 
        rightMaster.config_kI(Constants.kSlotIdxPos, kIPosRight, Constants.kTalonTimeoutMs); 
        rightMaster.config_kD(Constants.kSlotIdxPos, kDPosRight, Constants.kTalonTimeoutMs);
        rightMaster.configAllowableClosedloopError(Constants.kSlotIdxPos, rightTolerancePos, Constants.kTalonTimeoutMs);
		//Configure velocity loop PID
        rightMaster.selectProfileSlot(Constants.kSlotIdxSpeed, Constants.kTalonPidIdx); 
        rightMaster.config_kF(Constants.kSlotIdxSpeed, kFVelRight, Constants.kTalonTimeoutMs); 
        rightMaster.config_kP(Constants.kSlotIdxSpeed, kPVelRight, Constants.kTalonTimeoutMs); 
        rightMaster.config_kI(Constants.kSlotIdxSpeed, kIVelRight, Constants.kTalonTimeoutMs); 
        rightMaster.config_kD(Constants.kSlotIdxSpeed, kDVelRight, Constants.kTalonTimeoutMs);
        rightMaster.configAllowableClosedloopError(Constants.kSlotIdxSpeed, rightToleranceVel, Constants.kTalonTimeoutMs);


        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
    }

    

    @Override
    public void init(){}


    @Override
    public void run(){
        double leftPercent, rightPercent;
        if(Lift.getInstance().getPTOState() == PTOStates.DRIVE_ENABLED){
            leftPercent = (controls.getYAxis() +controls.getXAxis())/2.0;
            rightPercent = (controls.getYAxis()-controls.getXAxis())/2.0;
        } else {
            rightPercent = leftPercent = 0;
        }
        leftPercent *= 144;
        rightPercent *= 144;
        setVelocity(leftPercent, rightPercent);
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




    // public LinearAngularSpeed getLinearAngularSpeed(){
    //     WheelSpeed wheelSpeed = getSensedVelocity();


    // }

    public WheelSpeed getSensedIPS(){
        return new WheelSpeed(getSensedRPSLeft()*(wheelDiameter/2.0), getSensedRPSRight()*(wheelDiameter/2.0));
    }

    public double getSensedRPSLeft(){
        return Utility.upfToRPS(leftMaster.getSelectedSensorVelocity(), encoderUnitsPerRevLeft);
    }

    public double getSensedRPSRight(){
        return Utility.upfToRPS(rightMaster.getSelectedSensorVelocity(), encoderUnitsPerRevRight);
    }





    class WheelSpeed {
        double leftSpeed, rightSpeed;
        public WheelSpeed(double leftSpeed, double rightSpeed){
            this.leftSpeed = leftSpeed;
            this.rightSpeed = rightSpeed;
        }
    }

    class LinearAngularSpeed {
        double linearSpeed, angularSpeed;
        public LinearAngularSpeed(){}
        public LinearAngularSpeed(double linearSpeed, double angularSpeed){
            this.linearSpeed = linearSpeed;
            this.angularSpeed = angularSpeed;
        }

    }


}