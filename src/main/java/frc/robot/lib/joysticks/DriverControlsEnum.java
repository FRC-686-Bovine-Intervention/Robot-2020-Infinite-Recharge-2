package frc.robot.lib.joysticks;
// A list of all driver controls to be mapped to joystick buttons

public enum DriverControlsEnum {    // Controls Description
    SHOOT,                          // Shoot at outer high
    SEARCH,                         // Used by secondary driver to start searching sooner than shooting
    INTAKE,                         // Toggle between stored and ground
    REVERSE_BELTS,                  // Reverse all belts for jams
    DRIVE_ASSIST,                   // Drives assists
    QUICK_TURN,                     // to make TriggerDrive joysticks happy
    LOCK_LIFT,                      // Activates the cylinder used to lock the lift
    UNLOCK_LIFT,                    // Unlocks the lift
    TOGGLE_PTO,                     // Toggles the transmission for PTO and drive
    RESET,                          // Sets all subsystem to initial state
    CALIBRATE,                      // Calibrates shooter in the event it is needed
    MAX_HOOD;                       //For spitting balls
}

