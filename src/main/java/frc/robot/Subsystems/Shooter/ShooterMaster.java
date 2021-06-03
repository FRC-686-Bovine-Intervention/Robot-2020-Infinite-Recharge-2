package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.Loop;
import frc.robot.SmartDashboardInteractions;
import frc.robot.command_status.RobotState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.PTOStates;
import frc.robot.subsystems.shooter.Limelight.LedMode;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.sensors.Pigeon;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.lib.util.Vector2d;

public class ShooterMaster implements Loop {
    private static ShooterMaster instance = null;
    public static ShooterMaster getInstance(){
        if(instance == null){
            instance = new ShooterMaster();
        }
        return instance;
    }

    private DriverControlsBase controls = SelectedDriverControls.getInstance().get();

    private Turret turret = Turret.getInstance();
    private Hood hood = Hood.getInstance();
    private Flywheel flywheel = Flywheel.getInstance();
    private Limelight limelight = Limelight.getInstance();

    public enum ShooterState {
        DEBUG,
        CLIMBING,
        SEARCHING,
        SHOOTING,
        IDLING
    }
    private ShooterState cState = ShooterState.IDLING;
    
    //Search variables
    private double sweepStartTime = 0;
    private static final double sweepPeriod = 2.0;
    private static final double sweepRange = Math.toRadians(120);
    private boolean sweepFirstRun = true;
    private double targetLossStart = 0;
    private RisingEdgeDetector searchEdgeDetector = new RisingEdgeDetector();
    private boolean searchEdge = false;
    private static final double targetLossMax = 1.5;

    //Shooting variables:
    private Vector2d runningTargetPos = null, lastTargetPos = null;
    private double runningVariation = 1;
    private static final double variationTolerance = 48; //inches. Target "radius"
    private static final double variationAlpha = (1.0/5.0);

    private boolean calibrationComplete = false;
    
    public ShooterMaster(){
        limelight.setLEDMode(LedMode.kOff);

        SmartDashboard.putBoolean("Shooter/Debug", false);
        SmartDashboard.putBoolean("Shooter/Debug/Autotargeting", false);
        SmartDashboard.putNumber("Shooter/Debug/HoodPosition", 0);
        SmartDashboard.putNumber("Shooter/Debug/TurretPosition", 0);
        SmartDashboard.putNumber("Shooter/Debug/FlywheelRPM", 0);
        SmartDashboard.putBoolean("Shooter/Debug/Limelight", false);
    }

    @Override
    public void onStart() {}

    @Override
    public void onLoop(){
        SmartDashboard.putNumber("Shooter/Debug/SensedDistance", ShooterCalcs.getTargetDisplacement().length());

        //===========================
        //Target tracking:
        //===========================
        Vector2d sensedTargetPos;
        if(limelight.getIsTargetFound()){
            sensedTargetPos = ShooterCalcs.getTargetDisplacement();
            double variationError = RobotState.getInstance().getLatestFieldToVehicle().getPosition().sub(sensedTargetPos).length();
            //runningVariation = ShooterCalcs.expSmoothing(runningVariation, variationError, variationAlpha);
            runningVariation = 0; // For testing
        } else {
            sensedTargetPos = ShooterCalcs.getNewTargetPos();
            runningVariation = 0;
        }
        
        //Averaging:
        if(runningTargetPos == null) runningTargetPos = sensedTargetPos;
        else runningTargetPos = runningTargetPos.expAverage(sensedTargetPos, ShooterCalcs.targetSmoothing);

        if(limelight.getIsTargetFound()){
            //Updating robot pos based on limelight sensing
            Pose newPose = ShooterCalcs.getRobotPoseFromTargetPos(runningTargetPos, Pigeon.getInstance().getHeadingDeg());
            RobotState.getInstance().addFieldToVehicleObservation(Timer.getFPGATimestamp(), newPose);
        }

        //=============================
        //Decision Making:
        //=============================
        if(SmartDashboard.getBoolean("Shooter/Debug", false)){
            cState = ShooterState.DEBUG;
        } else if(Lift.getInstance().getPTOState() == PTOStates.LIFT_ENABLED){
            cState = ShooterState.CLIMBING;
        } else if(controls.getBoolean(DriverControlsEnum.SHOOT) && runningVariation <= variationTolerance){
            cState = ShooterState.SHOOTING;
        } else if(controls.getBoolean(DriverControlsEnum.SEARCH) || controls.getBoolean(DriverControlsEnum.SHOOT)){
            cState = ShooterState.SEARCHING;
        } else {
            cState = ShooterState.IDLING;
        }
        searchEdge = searchEdgeDetector.update(cState == ShooterState.SEARCHING);

        //====================================
        //Carrying out state behaviors:
        //====================================
        switch(cState){
            case DEBUG:
                if(SmartDashboard.getBoolean("Shooter/Debug/Limelight", false)) limelight.setLEDMode(LedMode.kOn);
                else limelight.setLEDMode(LedMode.kOff);

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


            case SEARCHING:
                limelight.setLEDMode(LedMode.kOn);
                if(searchEdge) targetLossStart = Timer.getFPGATimestamp();

                if(!limelight.getIsTargetFound()){
                    if(Timer.getFPGATimestamp()-targetLossStart > targetLossMax){
                        if(sweepFirstRun){
                            sweepStartTime = Timer.getFPGATimestamp();
                            sweepFirstRun = false;
                        }
                        double elapsedTime = Timer.getFPGATimestamp()-sweepStartTime;
                        double position = Math.sin((elapsedTime/sweepPeriod)*Math.PI)*sweepRange;
                        turret.setPosition(position);
                    } else {
                        turret.setPosition(runningTargetPos.angle());
                    }

                    flywheel.setRPS(0.0);
                    hood.setPosition(0.0);
                } else {
                    turret.setPosition((limelight.getTargetHorizontalAngleRad()/2.0)+turret.getSensedPosition());
                }
                break;

            case SHOOTING:
                limelight.setLEDMode(LedMode.kOn);
                
                // Commenting code below for sake of quickfix
                //Calculating information necessary for making shot along with applying lead
                // TODO: calc out velocity
                // Vector2d shooterVelocity = new Vector2d(); //ShooterCalcs.calcShooterLeadVelocity(runningTargetPos, Drivetrain.getInstance().getLinearAngularSpeed());
                // double hoodPosition = ShooterCalcs.calcHoodPosition(runningTargetPos.length());

                // //Respond physically
                // flywheel.setRPS(shooterVelocity.length());
                // turret.setPosition(((shooterVelocity.angle()-turret.getSensedPosition())*.99)+(turret.getSensedPosition()));
                // hood.setPosition(hoodPosition);

                double distance = ShooterCalcs.getTargetDisplacement().length();
                double hoodPos = ShooterCalcs.calcHoodPosition(distance);
                double flyWheelRPS = ShooterCalcs.calcShooterVelocity(distance);

                hood.setPosition(hoodPos);
                flywheel.setRPS(flyWheelRPS);
                turret.setPosition((limelight.getTargetHorizontalAngleRad()/2.0)+turret.getSensedPosition());

                break;

            case CLIMBING:
            case IDLING:
                limelight.setLEDMode(LedMode.kOff);
                turret.setPosition(0.0);
                hood.setPosition(0.0);
                flywheel.setRPS(0.0);
                break;
        }
    }

    @Override
    public void onStop(){}

    public void calibrateInit() {
        calibrationComplete = false;
        hood.calibrateStart();
        turret.calibrateStart();
    }

    public void calibrateLoop() {
        if(Lift.getInstance().calibrateFinished()){
            hood.calibrate();
            turret.calibrate();

            if(hood.calibrationFinished() && turret.calibrationFinished()){
                calibrationComplete = true;
            }
        }
    }

    public boolean calibrateFinished() {
        return calibrationComplete;
    }

    public boolean readyToShoot(){
        return (runningVariation <= variationTolerance) && flywheel.nearTarget();
    }

}