package frc.robot;


/**
 * Attribution: adapted from FRC Team 254
 */

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants {
    public static double kLoopDt = 0.01;
    public static double kDriveWatchdogTimerThreshold = 0.500;
    public static int kTalonTimeoutMs = 5; // ms
    public static int kTalonPidIdx = 0; // 0 for non-cascaded PIDs, 1 for cascaded PIDs
    public static int kTalonFXPidIdx = 1; // 0 for non-cascaded PIDs, 1 for cascaded PIDs
    public static double kTalonLoopPeriodSec = 0.1;
        
    public static double kNominalBatteryVoltage = 12.0;


    // Hardware Port Definitions
    public static int kPCMID =                      0;
    public static int kRightMotorMasterTalonId =    11;
    public static int kRightMotorSlave1TalonId =    12;
    public static int kLeftMotorMasterTalonId =     13;
    public static int kLeftMotorSlave1TalonId =     14;

    public static int kIntakeTalonId =              21;
    public static int kConveyorbeltMasterID =       22;
    public static int kConveyorbeltSlaveID =        23;
    public static int kConveyorHopperRightID =      24;
    public static int kConveyorHopperLeftID =       25;
    public static int kConveyorKickerMasterID=      26;
    public static int kConveyorKickerSlaveID=       27;
    public static int kEntranceProximityID =        1;
    public static int kExitProximityID =            2;

    public static int kShooterTalonId =             31;
    public static int kShooterSlaveId =             32;
    public static int kShooterTurretID =            33;
    public static int kShooterHoodID =              34;
    public static int kTurretHallEffectChannel =    0;

    public static int kPanelMasterId =              41;

    public static int kMainSolenoidFChannel =       7;
    public static int kMainSolenoidRChannel =       5;
    public static int kSecondarySolenoidFChannel =  6;
    public static int kSecondarySolenoidRChannel =  4;
    public static int kPTOSolenoidChannel =        3; 
    public static int kLiftLockSolenoidChannel =   2; 


    public static double kIntakePower = 1.0;
    public static double kConveyorFeedIPS = 7; //Inches per sec, feed rate of the tower

    public static double kConveyorFeedPercent = 0.5;
    public static double kConveyorBackUpPercent = 0.5;
    public static double kKickerShootPercent = 1.0;

    public static double kConveyorBackupDist = 3;

    public static double kLeftHopperPercent = 0.25;
    public static double kRightHopperPercent = 0.35;
    public static double kKickerProportion = 0.5; //a fraction of the speed of the shooter
    public static double kHopperReversePercent = 0.25; //For both motors


    public static double kWheelDiameter = 6.0; //inches
    public static double kDrivetrainWidth = 30.0; //inches
    

    //Encoder stuff
    public static final int kSlotIdxSpeed = 0;
    public static final int kSlotIdxPos   = 1;













    // Gyro
    public enum GyroSelectionEnum { BNO055, NAVX, PIGEON; }
    //public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.BNO055;
    public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.PIGEON;
    // public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.PIGEON;



    // Vision constants
    public static double kCameraFrameRate = 90.0;		// frames per second
    
    public static double kVisionMaxVel    = 20.0; // inches/sec  		
    public static double kVisionMaxAccel  = 20.0; // inches/sec^2		
    public static double kTargetWidthInches = 14.625;    
    public static double kTargetHeightInches = 6.00;
    public static double kCenterOfTargetHeightInches = 27.75;
    
    public static double kVisionCompletionTolerance = 1.0; 
    public static double kVisionMaxDistanceInches = 240;		// ignore targets greater than this distance
    public static double kVisionLookaheadDist = 24.0;	// inches
    
    // Shooter Constants
    public static double kAutoAimPredictionTime =   0;	// set to 0 since we don't have a turret and need to point the entire robot





}
