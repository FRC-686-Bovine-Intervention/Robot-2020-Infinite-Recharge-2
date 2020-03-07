package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControlStructures.AdvancedSubsystem;
import frc.robot.Controls.Controls;
import frc.robot.Controls.DriverControlsEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Lift.PTOStates;
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

    private static final int iSearch = 1, iShoot = 2, iIdle = 3, iDebug = 4, iClimb = 5;

    Decision debug = new Decision(iDebug, 1);
    Decision climbing = new Decision(iDebug, 2);
    Decision search = new Decision(iSearch, 3);
    Decision shoot = new Decision(iShoot, 4);
    Decision idle = new Decision(iIdle, 5);

    Decision[] options = {search, shoot, idle, debug, climbing};

    private Decision cDecision = idle;
    


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

        for(Decision option : options){
            option.clear();
        }

        //Check conditions for each:
        if(!limelight.getIsTargetFound() && cDecision.id != iShoot && controls.getBoolean(DriverControlsEnum.SHOOT)){
            search.vote();
        }
        if(controls.getBoolean(DriverControlsEnum.SHOOT)){
            shoot.vote();
        }
        if(SmartDashboard.getBoolean("Shooter/Debug", false)){
            debug.vote();
        }
        if(Lift.getInstance().getPTOState() == PTOStates.LIFT_ENABLED){
            climbing.vote();
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

                if(SmartDashboard.getBoolean("Shooter/Debug/Autotargeting", false) && limelight.getIsTargetFound()){
                    double cTurretPos = turret.getSensedPosition();
                    double cHorizRad = limelight.getTargetHorizontalAngleRad();
                    turret.setPosition(cTurretPos +cHorizRad/2.0);
                } else {
                    turret.setPosition(Math.toRadians(SmartDashboard.getNumber("Shooter/Debug/TurretPosition", 0)));
                }

                flywheel.setRPS(SmartDashboard.getNumber("Shooter/Debug/FlywheelRPM", 0)/9.5493);
                hood.setPosition(Math.toRadians(SmartDashboard.getNumber("Shooter/Debug/HoodPosition", 0)));
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
                
                //Determining what displacement vector will be used for the target
                Vector2d targetPos;
                if(limelight.getIsTargetFound()){
                    runningTargetPos = ShooterCalcs.getTargetDisplacement(runningTargetPos, limelight.getTargetVerticalAngleRad(),
                                     limelight.getTargetHorizontalAngleRad(), turret.getSensedPosition());
                    targetPos = runningTargetPos;

                    //Updating backup variables:
                    lastTargetPos = runningTargetPos;
                    lastDrivePosLeft = Drivetrain.getInstance().getSensedInchesLeft();
                    lastDrivePosRight = Drivetrain.getInstance().getSensedInchesRight();
                } else {
                    targetPos = ShooterCalcs.getNewTargetPos(lastTargetPos, 
                                    Drivetrain.getInstance().getSensedInchesLeft()- lastDrivePosLeft, Drivetrain.getInstance().getSensedInchesRight() - lastDrivePosRight);
                }
                
                //Calculating information necessary for making shot along with applying lead
                Vector2d shooterVelocity = ShooterCalcs.calcShooterLeadVelocity(targetPos, Drivetrain.getInstance().getLinearAngularSpeed());
                double hoodPosition = ShooterCalcs.calcHoodPosition(targetPos.length());

                //Respond physically
                flywheel.setRPS(shooterVelocity.length());
                turret.setPosition(shooterVelocity.angle());
                hood.setPosition(hoodPosition);

                //Old Code:

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
            case iClimb:
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
        if(lastTargetPos != null){
            SmartDashboard.putNumber("Shooter/TargetDist", lastTargetPos.length());
        }
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
        System.out.println(Lift.getInstance().calibrateFinished());
        if(Lift.getInstance().calibrateFinished()){
            hood.calibrate();
            turret.calibrate();

            if(hood.calibrationFinished() && turret.calibrationFinished()){
                calibrationComplete = true;
            }
        }
    }

    @Override
    public boolean calibrateFinished() {
        return calibrationComplete;
    }
    

}