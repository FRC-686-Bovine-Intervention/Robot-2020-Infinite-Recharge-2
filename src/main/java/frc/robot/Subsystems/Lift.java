package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ControlStructures.AdvancedSubsystem;
import frc.robot.Controls.Controls;
import frc.robot.Controls.DriverControlsEnum;
import frc.robot.util.RisingEdgeDetector;

public class Lift extends AdvancedSubsystem{
    private static Lift instance = null;
    public static Lift getInstance(){

        if(instance == null){
            instance = new Lift();
        }
        return instance;
    }

    private Controls controls;

    private Solenoid ptoSolenoids, lockSolenoids; 

    public enum PTOStates {
        DRIVE_ENABLED,
        LIFT_ENABLED;
    }

    public PTOStates ptoState = PTOStates.DRIVE_ENABLED; 


    private boolean driveVal = false;
    private boolean lockVal = false;

    private RisingEdgeDetector ptoEdge = new RisingEdgeDetector();


    private boolean calibrationComplete = false;
    private boolean leftCalibrated = false, rightCalibrated = false;
    private static final double calibrationTolerance = 0.6; //Rads/sec
    private double calibrationStartTime = 0;
    private static final double calibrationWaitDuration = 0.25; 


    public Lift(){
        controls = Controls.getInstance();

        ptoSolenoids = new Solenoid(Constants.kPCMID, Constants.kPTOSolenoidChannel);
        lockSolenoids = new Solenoid(Constants.kPCMID, Constants.kLiftLockSolenoidChannel);

        SmartDashboard.putBoolean("Lift/Debug", false);
        SmartDashboard.putBoolean("Lift/Debug/LockSolenoids", false);
        SmartDashboard.putBoolean("Lift/Debug/PTOSolenoids", false);
    }


    @Override
    public void init(){
        shiftToDrive();
    }

    @Override
    public void run(){
        if(!SmartDashboard.getBoolean("Lift/Debug", false)){
            if(ptoEdge.update(controls.getBoolean(DriverControlsEnum.TOGGLE_PTO))){
                if(ptoState == PTOStates.DRIVE_ENABLED){
                    shiftToLift();
                } else {
                    shiftToDrive();
                }
            }

            if(controls.getBoolean(DriverControlsEnum.LOCK_LIFT)){
                lockLift();
                shiftToDrive();
            } else if(controls.getBoolean(DriverControlsEnum.UNLOCK_LIFT)){
                unlockLift();
            }
        } else {
            //Debug stuff
            lockSolenoids.set(SmartDashboard.getBoolean("Lift/Debug/LockSolenoids", false));
            ptoSolenoids.set(SmartDashboard.getBoolean("Lift/Debug/PTOSolenoids", false));
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void updateSmartDashboard() {

    }

    @Override
    public void calibrateInit() {
        leftCalibrated = false;
        rightCalibrated = false;
        calibrationComplete = false;
        calibrationStartTime = Timer.getFPGATimestamp();
        shiftToLift();
        Drivetrain.getInstance().leftMaster.set(ControlMode.PercentOutput, 0.1);
        Drivetrain.getInstance().rightMaster.set(ControlMode.PercentOutput, 0.1);
    }

    @Override
    public void calibrateLoop() {
        if(Timer.getFPGATimestamp()-calibrationStartTime > calibrationWaitDuration){
            if(!leftCalibrated && Math.abs(Drivetrain.getInstance().getSensedRPSLeft()) <= calibrationTolerance){
                leftCalibrated = true;
                Drivetrain.getInstance().leftMaster.set(ControlMode.Velocity, 0.0);
            }

            if(!rightCalibrated && Math.abs(Drivetrain.getInstance().getSensedRPSRight()) <= calibrationTolerance){
                rightCalibrated = true;
                Drivetrain.getInstance().rightMaster.set(ControlMode.Velocity, 0.0);
            }

            if(leftCalibrated && rightCalibrated){
                Drivetrain.getInstance().zeroSensors();
                calibrationComplete = true;
                shiftToDrive();
            }
        }
    }

    @Override
    public boolean calibrateFinished() {
        return calibrationComplete;
    }

    public void lockLift(){
        lockSolenoids.set(lockVal);
    }

    public void unlockLift(){
        lockSolenoids.set(!lockVal);
    }

    public void shiftToDrive(){
        ptoSolenoids.set(driveVal);
        ptoState = PTOStates.DRIVE_ENABLED;
        lockLift();
    }
    public void shiftToLift(){
        ptoSolenoids.set(!driveVal);
        ptoState = PTOStates.LIFT_ENABLED;
        unlockLift();
    }

    public PTOStates getPTOState(){
        return ptoState;
    } 
}