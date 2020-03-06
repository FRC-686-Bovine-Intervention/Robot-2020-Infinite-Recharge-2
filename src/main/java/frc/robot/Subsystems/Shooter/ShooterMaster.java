package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControlStructures.AdvancedSubsystem;
import frc.robot.Controls.Controls;
import frc.robot.Controls.DriverControlsEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Shooter.Limelight.LedMode;
import frc.robot.util.Vector2d;

public class ShooterMaster extends AdvancedSubsystem {
    private static ShooterMaster instance = null;
    public static ShooterMaster getInstance(){
        if(instance == null){
            instance = new ShooterMaster();
        }
        return instance;
    }

    private Controls controls;

    private Turret turret;
    private Hood hood;
    private Flywheel flywheel;
    private Limelight limelight;




    //Decision variables =======================================
    public class Decision{
        public int priority, id;
        public boolean elected = false;
        public Decision(int id, int priority){
            this.priority = priority;
            this.id = id;
        }
        public void clear(){
            elected = false;
        }
        public void vote(){
            elected = true;
        }
    }

    private static final int iCalibrate = 1, iSearch =2, iShoot = 3, iIdle = 4, iDebug = 5;

    Decision debug = new Decision(iDebug, 1);
    Decision calibrate = new Decision(iCalibrate, 2);
    Decision search = new Decision(iSearch, 3);
    Decision shoot = new Decision(iShoot, 4);
    Decision idle = new Decision(iIdle, 5);

    Decision[] options = {calibrate, search, shoot, idle, debug};

    private Decision cDecision = idle;
    


    private int failedLoops = 0;
    private static final int maxFailedLoops = 3;

    private boolean allCalibrated = false;



    //Search variables
    private double sweepStartTime = 0;
    private static final double sweepPeriod = 1.75;
    private static final double sweepRange = Math.PI/2.0;
    private boolean sweepFirstRun = true;


    //Shooting variables:
    //private double lastTargetPos = 0;
    private double lastDrivePosLeft = 0;
    private double lastDrivePosRight = 0;
    private Vector2d runningTargetPos = null, lastTargetPos = null;



    private boolean calibrationComplete = false;



    
    public ShooterMaster(){
        controls = Controls.getInstance();
        flywheel = Flywheel.getInstance();
        hood = Hood.getInstance();
        turret = Turret.getInstance();

        limelight = Limelight.getInstance();
        limelight.setLEDMode(LedMode.kOff);

        SmartDashboard.putBoolean("Shooter/Debug", false);
        SmartDashboard.putBoolean("Shooter/Debug/Autotargeting", false);
        SmartDashboard.putNumber("Shooter/Debug/HoodPosition", 0);
        SmartDashboard.putNumber("Shooter/Debug/TurretPosition", 0);
        SmartDashboard.putNumber("Shooter/Debug/FlywheelRPS", 0);
        SmartDashboard.putBoolean("Shooter/Debug/Limelight", false);
    }

    

    @Override
    public void init() {

    }

    @Override
    public void run(){
        if(!limelight.getIsTargetFound() && (controls.getBoolean(DriverControlsEnum.SHOOT))){
            failedLoops++;
        } else {
            failedLoops = 0;
        }



        for(Decision option : options){
            option.clear();
        }

        //Check conditions for each:
        if((cDecision.id == calibrate.id && !allCalibrated) || controls.getBoolean(DriverControlsEnum.CALIBRATE)){
            calibrate.vote();
        }
        if(failedLoops >= maxFailedLoops){
            search.vote();
        }
        if(controls.getBoolean(DriverControlsEnum.SHOOT)){
            shoot.vote();
        }
        if(SmartDashboard.getBoolean("Shooter/Debug", false)){
            debug.vote();
        }


        //Default:
        idle.vote();

        //Choose:
        Decision bestOption = null;
        for(Decision option : options){
            if(option.elected){
                if(bestOption == null){
                    bestOption = option;
                } else {
                    bestOption = bestOption.priority < option.priority ? bestOption : option;
                }
            }
        }
        if(cDecision.id != search.id && bestOption.id == search.id){
            sweepFirstRun = true;
        }
        cDecision = bestOption;


        //Behavior:
        switch(cDecision.id){
            case iDebug:
                if(SmartDashboard.getBoolean("Shooter/Debug/Limelight", false)){
                    limelight.setLEDMode(LedMode.kOn);
                } else {
                    limelight.setLEDMode(LedMode.kOff);
                }

                if(SmartDashboard.getBoolean("Shooter/Debug/Autotargeting", false)){
                    double cTurretPos = turret.getSensedPosition();
                    double cHorizRad = limelight.getTargetHorizontalAngleRad();
                    turret.setPosition(cTurretPos +cHorizRad/2.0);
                } else {
                    turret.setPosition(Math.toRadians(SmartDashboard.getNumber("Shooter/Debug/TurretPosition", 0)));
                }

                flywheel.setRPS(SmartDashboard.getNumber("Shooter/Debug/FlywheelRPM", 0)/9.5493);
                hood.setPosition(Math.toRadians(SmartDashboard.getNumber("Shooter/Debug/HoodPosition", 0)));
                break;

            case iCalibrate:
                limelight.setLEDMode(LedMode.kOff);

               
                break;



            case iSearch:
                limelight.setLEDMode(LedMode.kOn);

                if(sweepFirstRun){
                    sweepStartTime = Timer.getFPGATimestamp();
                    sweepFirstRun = false;
                }
                double elapsedTime = Timer.getFPGATimestamp()-sweepStartTime;
                double position = Math.sin((elapsedTime/sweepPeriod)*Math.PI)*sweepRange;
                turret.setPosition(position);

                flywheel.setRPS(0.0);
                hood.setPosition(0.0);
                break;



            case iShoot:
                limelight.setLEDMode(LedMode.kOn);
                if(limelight.getIsTargetFound()){
                    runningTargetPos = ShooterCalcs.getTargetDisplacement(runningTargetPos, limelight.getTargetVerticalAngleRad(),
                                     limelight.getTargetHorizontalAngleRad(), turret.getSensedPosition());
                    lastDrivePosLeft = Drivetrain.getInstance().getSensedInchesLeft();
                    lastDrivePosRight = Drivetrain.getInstance().getSensedInchesRight();
                } else {
                    runningTargetPos = ShooterCalcs.getNewTargetPos(lastTargetPos, lastDrivePosLeft, lastDrivePosRight);
                }
                

                ShooterCalcs.calcShooterLeadVelocity(distance)


                // double targetDisplacement;
                // if(limelight.getIsTargetFound()){
                //     targetDisplacement = getTargetDisplacement();
                //     lastTargetPos = targetDisplacement;
                // } else {
                //     targetDisplacement = lastTargetPos;
                // }

                // double cHorizRad = limelight.getTargetHorizontalAngleRad();
                // double cTurretPos = turret.getSensedPosition();
                // turret.setPosition(cTurretPos +cHorizRad/2.0);

                // int keyRPS = getLinear(targetDisplacement, dataTable);
                // double nominalSpeed = handleLinear(targetDisplacement, dataTable[keyRPS][0], dataTable[keyRPS+1][0], dataTable[keyRPS][1], dataTable[keyRPS+1][1]);

                // int keyHood = getLinear(targetDisplacement, dataTable);
                // double nominalPosition = handleLinear(targetDisplacement, dataTable[keyHood][0], dataTable[keyHood+1][0], dataTable[keyHood][1], dataTable[keyHood+1][1]);

                // flywheel.setRPS(nominalSpeed/9.5493);
                // hood.setPosition(Math.toRadians(nominalPosition));
                break;



            case iIdle:
                limelight.setLEDMode(LedMode.kOff);
                turret.setPosition(0.0);
                hood.setPosition(0.0);
                flywheel.setRPS(0.0);
                break;
        }
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Shooter/TargetDist", getTargetDisplacement());
        SmartDashboard.putNumber("Shooter/TurretSensedPos", Math.toDegrees(turret.getSensedPosition()));
        SmartDashboard.putNumber("Shooter/HoodSensedPos",  Math.toDegrees(hood.getSensedPosition()));
        SmartDashboard.putNumber("Shooter/FlywheelSensedRPM", flywheel.getSensedRPS()*9.5493);
    }




    @Override
    public void calibrateInit() {
        calibrationComplete = false;
        hood.calibrateStart();
        turret.calibrateStart();
    }

    @Override
    public void calibrateLoop() {
        hood.calibrate();
        turret.calibrate();

        if(hood.calibrationFinished() && turret.calibrationFinished()){
            calibrationComplete = true;
        }
    }

    @Override
    public boolean calibrateFinished() {
        return calibrationComplete;
    }
    

}