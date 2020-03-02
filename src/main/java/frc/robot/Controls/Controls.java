package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;


public class Controls {

    private static Controls instance = null;
    public static Controls getInstance(){
        if(instance == null){
            instance = new Controls();
        }
        return instance;
    }

    Joystick thrustmaster;
    Joystick buttonBoard;
    


    public static final int kXAxis = 0;
    public static final int kYAxis = 1;


    public Controls(){
        thrustmaster = new Joystick(0);
        buttonBoard = new Joystick(1);
    }

    public double getXAxis(){
        return thrustmaster.getRawAxis(kXAxis);
    }

    public double getYAxis(){
        return -thrustmaster.getRawAxis(kYAxis);
    }


    public boolean getBoolean(DriverControlsEnum _controls){
        switch (_controls)
        {
            case SHOOT:                         return thrustmaster.getRawButton(Thrustmaster.kTriggerButton);
            case DRIVE_ASSIST:                  return false;
            case INTAKE:                        return thrustmaster.getRawButton(Thrustmaster.kBottomThumbButton);
            case REVERSE_BELTS:                 return thrustmaster.getRawButton(Thrustmaster.kLeftThumbButton);
            case LOCK_LIFT:                     return buttonBoard.getRawButton(ButtonBoard.kButtonY);
            case UNLOCK_LIFT:                   return buttonBoard.getRawButton(ButtonBoard.kButtonX);
            case TOGGLE_PTO:                    return thrustmaster.getRawButton(Thrustmaster.kBottomButton6);
            case CALIBRATE:                     return buttonBoard.getRawButton(ButtonBoard.kButtonA);
            case RESET:                         return false;
            case QUICK_TURN:                    return false;
            case MAX_HOOD:                      return buttonBoard.getRawButton(ButtonBoard.kButtonB);
            default:                            return false;
        }
    }
}