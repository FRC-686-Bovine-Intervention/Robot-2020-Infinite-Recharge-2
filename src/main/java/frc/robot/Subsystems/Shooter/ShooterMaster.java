package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Controls.Controls;
import frc.robot.Controls.DriverControlsEnum;
import frc.robot.Subsystems.Shooter.Limelight.LedMode;
import frc.robot.util.Vector2d;

public class ShooterMaster {
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
    private DigitalInput hallEffect;
    private Limelight limelight;


    //Physical Variables =====================================
    private static final double targetHeight = 0;//99;
    private static final double cameraHeight = 0; //41;
    private static final double cameraAngleElevation = 0;//25;




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

    private static final int iCalibrate = 1, iSearch =2, iShoot = 3, iIdle = 4;

    Decision Calibrate = new Decision(iCalibrate, 1);
    Decision Search = new Decision(iSearch, 2);
    Decision Shoot = new Decision(iShoot,3);
    Decision Idle = new Decision(iIdle, 4);

    Decision[] options = {Calibrate, Search, Shoot, Idle};

    private Decision cDecision = Idle;
    


    private int failedLoops = 0;
    private static final int maxFailedLoops = 3;

    private boolean turretCalibrated, hoodCalibrated, allCalibrated = false;
    private int successfulHoodLoops = 0;
    private static final int requiredHoodLoops = 30;
    private static final double hoodCalibTolerance = 0.15;
    private double hoodLastPos = 0;



    //Search variables
    private double sweepStartTime = 0;
    private static final double sweepPeriod = 1.75;
    private static final double sweepRange = Math.PI/2.0;
    private boolean sweepFirstRun = true;


    //Shooting variables:
    private double lastTargetPos = 0;




    public double[][] dataTable = {
        {24,3600,0},
        {49,2750,22},
        {105,3000,35},
        {189,4000,45},
        {265,4750,47},
    };



    
    public ShooterMaster(){
        controls = Controls.getInstance();
        flywheel = Flywheel.getInstance();
        hood = Hood.getInstance();
        turret = Turret.getInstance();

        hallEffect = new DigitalInput(Constants.kTurretHallEffectChannel);
        limelight = Limelight.getInstance();
        limelight.setLEDMode(LedMode.kOff);
    }



    public void start(){}


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
        if((cDecision.id == Calibrate.id && !allCalibrated) || controls.getBoolean(DriverControlsEnum.CALIBRATE)){
            Calibrate.vote();
        }
        if(failedLoops >= maxFailedLoops){
            Search.vote();
        }
        if(controls.getBoolean(DriverControlsEnum.SHOOT)){
            Shoot.vote();
        }

        System.out.println("Loops: " + failedLoops);

        //Default:
        Idle.vote();

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
        if(cDecision.id != Search.id && bestOption.id == Search.id){
            sweepFirstRun = true;
        }
        cDecision = bestOption;
        System.out.println("Id: " + cDecision.id);


        //Behavior:
        switch(cDecision.id){
            case iCalibrate:
                limelight.setLEDMode(LedMode.kOff);
                if(hallEffect.get() && !turretCalibrated){
                    turret.setPercent(-0.1825);
                } else {
                    turret.setPercent(0.0);
                    turretCalibrated = true;
                    turret.zeroWithInit(-3);
                }
                System.out.print(Math.abs(hood.getSensedPos()-hoodLastPos));

                if(Math.abs(hood.getSensedPos()-hoodLastPos) <= hoodCalibTolerance && !hoodCalibrated){
                    successfulHoodLoops++;
                } else {
                    successfulHoodLoops = 0;
                }
                if(!hoodCalibrated && successfulHoodLoops < requiredHoodLoops){
                    hood.setPercent(-0.1825);
                } else {
                    hood.setPercent(0.0);
                    hood.zeroSensor();
                    hoodCalibrated = true;
                }

                if(hoodCalibrated && turretCalibrated){
                    allCalibrated = true;
                }
                break;



            case iSearch:
                limelight.setLEDMode(LedMode.kOn);
                if(sweepFirstRun){
                    sweepStartTime = Timer.getFPGATimestamp();
                    sweepFirstRun = false;
                }
                double elapsedTime = Timer.getFPGATimestamp()-sweepStartTime;
                double position = Math.sin((elapsedTime/sweepPeriod)*Math.PI)*sweepRange;
                System.out.println("Position: "+ position);
                turret.setPosition(position);

                flywheel.setRPS(0.0);
                hood.setPosition(0.0);
                break;



            case iShoot:
                limelight.setLEDMode(LedMode.kOn);
                double targetDisplacement;
                if(limelight.getIsTargetFound()){
                    targetDisplacement = getTargetDisplacement();
                    lastTargetPos = targetDisplacement;
                } else {
                    targetDisplacement = lastTargetPos;
                }

                double cHorizRad = limelight.getTargetHorizontalAngleRad();
                double cTurretPos = turret.getSensedPosition();
                turret.setPosition(cTurretPos +cHorizRad/2.0);

                int keyRPS = getLinear(targetDisplacement, dataTable);
                double nominalSpeed = handleLinear(targetDisplacement, dataTable[keyRPS][0], dataTable[keyRPS+1][0], dataTable[keyRPS][1], dataTable[keyRPS+1][1]);

                int keyHood = getLinear(targetDisplacement, dataTable);
                double nominalPosition = handleLinear(targetDisplacement, dataTable[keyHood][0], dataTable[keyHood+1][0], dataTable[keyHood][1], dataTable[keyHood+1][1]);


                flywheel.setRPS(nominalSpeed * 0.016666);
                hood.setPosition(nominalPosition);
                break;



            case iIdle:
                limelight.setLEDMode(LedMode.kOff);
                turret.setPosition(0.0);
                hood.setPosition(0.0);
                flywheel.setRPS(0.0);
                break;
        }
    }

    



    public double getTargetDisplacement(){
        double targetY = (targetHeight-cameraHeight)/Math.tan(limelight.getTargetVerticalAngleRad()+Math.toRadians(cameraAngleElevation));
        double targetX = targetY*Math.tan(-limelight.getTargetHorizontalAngleRad()); //Negative is to ensure that left of camera is positive from top-view
        Vector2d detectedTargetPos = new Vector2d(targetX, targetY);
        return detectedTargetPos.length();
    }

    public int getLinear (double d, double table[][])
    {
        double distance = Math.max(Math.min(d, table[table.length-1][0]), table[0][0]);
        int k;
        for (k=0;k<table.length;k++)
        {
            if (distance <= table[k][0])
            {
                break;
            }
        }
        return Math.max(k-1, 0);
    }


    public double handleLinear (double d, double dL, double dH, double sL, double sH)
    {
        return (sH-sL)*Math.min((d-dL)/(dH-dL),1)+sL;
    }
}