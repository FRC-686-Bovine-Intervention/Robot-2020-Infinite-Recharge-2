package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.ControlStructures.AdvancedSubsystem;
import frc.robot.Controls.Controls;
import frc.robot.Controls.DriverControlsEnum;
import frc.robot.Subsystems.Shooter.Utility;
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
    private double driveLastPos = 100000;
    private static final double calibrationTolerance = Math.toRadians(2); //Radians


    public Lift(){
        controls = Controls.getInstance();

        ptoSolenoids = new Solenoid(Constants.kPCMID, Constants.kPTOSolenoidChannel);
        lockSolenoids = new Solenoid(Constants.kPCMID, Constants.kLiftLockSolenoidChannel);
    }


    @Override
    public void init(){
        shiftToDrive();
    }

    @Override
    public void run(){
        // if(ptoEdge.update(controls.getBoolean(DriverControlsEnum.TOGGLE_PTO))){
        //     if(ptoState == PTOStates.DRIVE_ENABLED){
        //         shiftToLift();
        //     } else {
        //         shiftToDrive();
        //     }
        // }

        // if(controls.getBoolean(DriverControlsEnum.LOCK_LIFT)){
        //     lockLift();
        // } else if(controls.getBoolean(DriverControlsEnum.UNLOCK_LIFT)){
        //     unlockLift();
        // }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void updateSmartDashboard() {

    }

    

    @Override
    public void calibrateInit() {

    }

    @Override
    public void calibrateLoop() {

    }

    @Override
    public boolean calibrateFinished() {
        return false;
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