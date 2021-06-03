package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.NonLinearityEnum;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.PTOStates;


public class DriverControlsMadcatz extends DriverControlsBase
{
	// singleton class
    private static DriverControlsThrustmaster instance = null;
    public static DriverControlsThrustmaster getInstance() 
    { 
        if (instance == null) {
            instance = new DriverControlsThrustmaster();
        }
        return instance;
    }
    
    
    // Joystick Port Constants
    public static int kControllerPort = 0;
    public static double kMaxSpeedPercent = 0.5;

    public static JoystickBase controller;

    public static SteeringBase steeringControls;
    DriveCommand driveCmd = new DriveCommand(0,0);

    DeadbandNonLinearity deadbandNonLinearity;
 
    public DriverControlsMadcatz() 
    {
        controller = new Madcatz(kControllerPort);

        double throttleDeadband =     0.02;
        double turnDeadband =         0.02;
        NonLinearityEnum throttleNonLinearity = NonLinearityEnum.SQUARED;
        NonLinearityEnum turnNonLinearity =     NonLinearityEnum.SQUARED;

        steeringControls = new TmArcadeDriveSteering(controller, new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity));
        deadbandNonLinearity = new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity);
    }

    public DriveCommand getDriveCommand() 
    {
        double throttle = -controller.getAxis(Madcatz.kLeftYAxis)*kMaxSpeedPercent; 
        double turn =     -controller.getAxis(Madcatz.kLeftXAxis)*kMaxSpeedPercent;
        
        //'Turning' is reversed for lift
        if(Lift.getInstance().getPTOState() == PTOStates.LIFT_ENABLED){
            driveCmd = SteeringLib.arcadeDrive(throttle, -turn, deadbandNonLinearity);
        } else {
            driveCmd = SteeringLib.arcadeDrive(throttle, turn, deadbandNonLinearity);
        }
        
		return driveCmd;  
    }


    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        // intake when: driving forward or turning, when outtake button is not pressed
        //              (so no intake when driving backwards or stopped)
        DriveCommand driveCmd = Drive.getInstance().getCommand();

        switch (_control)
        {
            case SHOOT:                         return controller.getButton(Madcatz.kAButton);
            // case DRIVE_ASSIST:                  return controller.getButton();
            case INTAKE_TOGGLE:                 return controller.getButton(Madcatz.kBButton);
            // case INTAKE_STORED:                 return controller.getButton();
            // case LOCK_LIFT:                     return controller.getButton();
            // case UNLOCK_LIFT:                   return controller.getButton();
            // case TOGGLE_PTO:                    return controller.getButton();
            // case QUICK_TURN:                    return false;
            default:                            return false;
        }
    }

    public double getAxis( DriverAxisEnum _axis ) 
    {
        switch (_axis)
        {
            case SHOOTER_SPEED_CORRECTION:      return controller.getAxis(Thrustmaster.kSliderAxis);
            default:                            return 0.0;
        }
    }


    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            if (controller != null)         { controller.getLogger().log(); }
            if (steeringControls != null)   { steeringControls.getLogger().log(); }
        }
    };        
}
