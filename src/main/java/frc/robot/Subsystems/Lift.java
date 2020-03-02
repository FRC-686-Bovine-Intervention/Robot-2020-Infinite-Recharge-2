package frc.robot.Subsystems;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Controls.Controls;
import frc.robot.Controls.DriverControlsEnum;
import frc.robot.util.RisingEdgeDetector;

public class Lift {
    private static Lift instance = null;
    public static Lift getInstance(){
        if(instance == null){
            instance = new Lift();
        }
        return instance;
    }

    private Controls controls;


    private Solenoid ptoSolenoids, lockSolenoids; 

    public enum ptoStates {
        DRIVE_ENABLED,
        LIFT_ENABLED;
    }

    public ptoStates ptoState = ptoStates.DRIVE_ENABLED; 


    private boolean driveVal = false;
    private boolean lockVal = false;

    private RisingEdgeDetector ptoEdge = new RisingEdgeDetector();


    public Lift(){
        controls = Controls.getInstance();

        ptoSolenoids = new Solenoid(Constants.kPCMID, Constants.kPTOSolenoidChannel);
        lockSolenoids = new Solenoid(Constants.kPCMID, Constants.kLiftLockSolenoidChannel);
    }



    public void start(){
        shiftToDrive();
    }

    public void run(){
        if(ptoEdge.update(controls.getBoolean(DriverControlsEnum.TOGGLE_PTO))){
            if(ptoState == ptoStates.DRIVE_ENABLED){
                shiftToLift();
            } else {
                shiftToDrive();
            }
        }

        if(controls.getBoolean(DriverControlsEnum.LOCK_LIFT)){
            lockLift();
        } else if(controls.getBoolean(DriverControlsEnum.UNLOCK_LIFT)){
            unlockLift();
        }
    }



    public void lockLift(){
        lockSolenoids.set(lockVal);
    }

    public void unlockLift(){
        lockSolenoids.set(!lockVal);
    }

    public void shiftToDrive(){
        ptoSolenoids.set(driveVal);
        ptoState = ptoStates.DRIVE_ENABLED;
        lockLift();
    }
    public void shiftToLift(){
        ptoSolenoids.set(!driveVal);
        ptoState = ptoStates.LIFT_ENABLED;
        unlockLift();
    }
}