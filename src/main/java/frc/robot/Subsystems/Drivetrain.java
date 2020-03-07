package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public TalonFX leftMaster, leftSlave, rightMaster, rightSlave;

    private static final double kFPos = 0;
    private static final double kPPos = 0.32;
    private static final double kIPos = 0;
    private static final double kDPos = 3.2;
    private static final int tolerancePos = 10;

    private static final double kFVel = 0.05; //0.05;
    private static final double kPVel = 0.006; //0.032;
    private static final double kIVel = 0.0; //0;
    private static final double kDVel = 0.25; //2.56;
    private static final int toleranceVel = 0;
    
    private static final double encoderUnitsPerRevLeft = 24593.0;
    private static final double encoderUnitsPerRevRight = 23763.0;


    public static final double extensionTolerance = 2;


    //Physical parameters
    public static final double wheelDiameter = 6.0; //inches 
    public static final double drivetrainWidth = 30.0; //inches 
    public static final double maxExtension = 230; //Arbitrary 






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
        leftMaster.config_kF(Constants.kSlotIdxPos, kFPos, Constants.kTalonTimeoutMs); 
        leftMaster.config_kP(Constants.kSlotIdxPos, kPPos, Constants.kTalonTimeoutMs); 
        leftMaster.config_kI(Constants.kSlotIdxPos, kIPos, Constants.kTalonTimeoutMs); 
        leftMaster.config_kD(Constants.kSlotIdxPos, kDPos, Constants.kTalonTimeoutMs);
        leftMaster.configAllowableClosedloopError(Constants.kSlotIdxPos, tolerancePos, Constants.kTalonTimeoutMs);
        //Configure velocity loop PID 
        leftMaster.selectProfileSlot(Constants.kSlotIdxSpeed, Constants.kTalonPidIdx); 
        leftMaster.config_kF(Constants.kSlotIdxSpeed, kFVel, Constants.kTalonTimeoutMs); 
        leftMaster.config_kP(Constants.kSlotIdxSpeed, kPVel, Constants.kTalonTimeoutMs); 
        leftMaster.config_kI(Constants.kSlotIdxSpeed, kIVel, Constants.kTalonTimeoutMs); 
        leftMaster.config_kD(Constants.kSlotIdxSpeed, kDVel, Constants.kTalonTimeoutMs);
        leftMaster.configAllowableClosedloopError(Constants.kSlotIdxSpeed, toleranceVel, Constants.kTalonTimeoutMs);


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
        rightMaster.config_kF(Constants.kSlotIdxPos, kFPos, Constants.kTalonTimeoutMs); 
        rightMaster.config_kP(Constants.kSlotIdxPos, kPPos, Constants.kTalonTimeoutMs); 
        rightMaster.config_kI(Constants.kSlotIdxPos, kIPos, Constants.kTalonTimeoutMs); 
        rightMaster.config_kD(Constants.kSlotIdxPos, kDPos, Constants.kTalonTimeoutMs);
        rightMaster.configAllowableClosedloopError(Constants.kSlotIdxPos, tolerancePos, Constants.kTalonTimeoutMs);
		//Configure velocity loop PID
        rightMaster.selectProfileSlot(Constants.kSlotIdxSpeed, Constants.kTalonPidIdx); 
        rightMaster.config_kF(Constants.kSlotIdxSpeed, kFVel, Constants.kTalonTimeoutMs); 
        rightMaster.config_kP(Constants.kSlotIdxSpeed, kPVel, Constants.kTalonTimeoutMs); 
        rightMaster.config_kI(Constants.kSlotIdxSpeed, kIVel, Constants.kTalonTimeoutMs); 
        rightMaster.config_kD(Constants.kSlotIdxSpeed, kDVel, Constants.kTalonTimeoutMs);
        rightMaster.configAllowableClosedloopError(Constants.kSlotIdxSpeed, toleranceVel, Constants.kTalonTimeoutMs);


        leftMaster.configClosedloopRamp(0.375, Constants.kTalonTimeoutMs);
        rightMaster.configClosedloopRamp(0.375, Constants.kTalonTimeoutMs);


        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        SmartDashboard.putBoolean("Drivetrain/ZeroWheelPos", false);
    }

    

    @Override
    public void init(){}


    @Override
    public void run(){
        if(SmartDashboard.getBoolean("Drivetrain/ZeroWheelPos", false)){
            leftMaster.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
            rightMaster.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);

        }
        double leftPercent, rightPercent, IPSLeft, IPSRight;
        if(Lift.getInstance().getPTOState() == PTOStates.DRIVE_ENABLED){
            leftPercent = (controls.getYAxis() +controls.getXAxis())/2.0;
            rightPercent = (controls.getYAxis()-controls.getXAxis())/2.0;
            IPSLeft = leftPercent*144;
            IPSRight = rightPercent*144;
        } else if(Lift.getInstance().getPTOState() == PTOStates.LIFT_ENABLED){
            double avgExtension = -(getSensedInchesLeft()+getSensedInchesRight()/2.0);
            if((avgExtension < maxExtension || controls.getYAxis() < 0) && (avgExtension > 0 || controls.getYAxis() > 0)){
                //Negative IPS makes lift go up
                double error = getSensedInchesRight()-getSensedInchesLeft();
                double baseIPS = -controls.getYAxis()*144;
                if(Math.abs(error) > extensionTolerance){
                    IPSRight = -(error*2.0) + baseIPS;
                    IPSLeft = (error*2.0) + baseIPS;
                } else {
                    IPSRight = IPSLeft = baseIPS;
                }
            } else {
                IPSLeft = IPSRight = 0;
            }
        } else {
            IPSLeft = IPSRight = 0;
        }
        setDriveIPS(IPSLeft, IPSRight);
    }

    @Override
    public void zeroSensors() {
        leftMaster.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
        rightMaster.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
    }

    @Override
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Drivetrain/InchesLeft", getSensedInchesLeft());
        SmartDashboard.putNumber("Drivetrain/InchesRight", getSensedInchesRight());

    }




    /**
     * Sets velocity of drivetrain in inches/sec
     */
    public void setDriveIPS(double leftVel, double rightVel){
        leftVel /= (wheelDiameter/2.0);
        rightVel /= (wheelDiameter/2.0);
        setRPS(leftVel, rightVel);
    }

    public void setRPS(double leftVel, double rightVel){
        leftVel = Utility.rpsToUPF(leftVel, encoderUnitsPerRevLeft);
        rightVel = Utility.rpsToUPF(rightVel, encoderUnitsPerRevRight);

        leftMaster.set(ControlMode.Velocity, leftVel);
        rightMaster.set(ControlMode.Velocity, rightVel);
    }

    public void setPower(double leftPower, double rightPower){
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
    }


    /**
     * What up fam?
     * @return The linear and angular speed of the center of the robot in IPS and RPS, respectively
     */
    public LinearAngularSpeed getLinearAngularSpeed(){
        WheelSpeed wheelSpeed = getSensedIPS();
        double linearSpeed = (wheelSpeed.rightSpeed+wheelSpeed.leftSpeed)/2.0;
        double angularSpeed = (wheelSpeed.rightSpeed-wheelSpeed.leftSpeed)/drivetrainWidth;
        return new LinearAngularSpeed(linearSpeed, angularSpeed);
    }



    public WheelSpeed getSensedIPS(){
        return new WheelSpeed(getSensedRPSLeft()*(wheelDiameter/2.0), getSensedRPSRight()*(wheelDiameter/2.0));
    }

    public double getSensedRPSLeft(){
        return Utility.upfToRPS(leftMaster.getSelectedSensorVelocity(), encoderUnitsPerRevLeft);
    }

    public double getSensedRPSRight(){
        return Utility.upfToRPS(rightMaster.getSelectedSensorVelocity(), encoderUnitsPerRevRight);
    }
    

    public double getSensedInchesLeft(){
        return Utility.encoderUnitsToRadians(leftMaster.getSelectedSensorPosition(), encoderUnitsPerRevLeft)*(wheelDiameter/2.0);
    }

    public double getSensedInchesRight(){
        return Utility.encoderUnitsToRadians(rightMaster.getSelectedSensorPosition(), encoderUnitsPerRevRight)*(wheelDiameter/2.0);
    }




    class WheelSpeed {
        double leftSpeed, rightSpeed;
        public WheelSpeed(double leftSpeed, double rightSpeed){
            this.leftSpeed = leftSpeed;
            this.rightSpeed = rightSpeed;
        }
    }

    public class LinearAngularSpeed {
        public double linearSpeed, angularSpeed;
        public LinearAngularSpeed(){}
        public LinearAngularSpeed(double linearSpeed, double angularSpeed){
            this.linearSpeed = linearSpeed;
            this.angularSpeed = angularSpeed;
        }

    }


}