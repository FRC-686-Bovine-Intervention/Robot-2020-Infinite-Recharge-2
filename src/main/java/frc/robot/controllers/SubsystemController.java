package frc.robot.controllers;

import java.util.ArrayList;
import java.util.List;

public class SubsystemController{

    private List<Subsystem> subsystems = new ArrayList<Subsystem>();
    private List<AdvancedSubsystem> advancedSubsystems = new ArrayList<AdvancedSubsystem>();

    private boolean calibrationStarted = false;
    private boolean calibrateFinished = false;


    public SubsystemController(){}


    public void init(){
        for(Subsystem subsystem : subsystems){
            subsystem.init();
        }

        for(AdvancedSubsystem advancedSubsystem : advancedSubsystems){
            advancedSubsystem.init();
        }
    }


    public void onTeleLoop(){
        for(Subsystem subsystem : subsystems){
            subsystem.run();
        }

        for(AdvancedSubsystem advancedSubsystem : advancedSubsystems){
            advancedSubsystem.run();
        }
    }




    public void resetForCalibration(){
        calibrationStarted = false;
        calibrateFinished = false;
    }



    public void runCalibration(){
        if(!calibrationStarted){
            for(AdvancedSubsystem advancedSubsystem : advancedSubsystems){
                advancedSubsystem.calibrateInit();
            }
            calibrationStarted = true;
        } else {
            calibrateFinished = true;
            for(AdvancedSubsystem advancedSubsystem : advancedSubsystems){
                if(!advancedSubsystem.calibrateFinished()){
                    advancedSubsystem.calibrateLoop();
                    calibrateFinished = false;
                } 
            }
        }
    }

    public boolean calibrationComplete(){
        return calibrateFinished;
    }

    public void updateSmartDashboard(){
        for(Subsystem subsystem : subsystems){
            subsystem.updateSmartDashboard();
        }

        for(AdvancedSubsystem advancedSubsystem : advancedSubsystems){
            advancedSubsystem.updateSmartDashboard();
        }
    }












    public void addSubsystem(Subsystem subsystem){
        subsystems.add(subsystem);
    }

    public void addSubsystem(AdvancedSubsystem advancedSubsystem){
        advancedSubsystems.add(advancedSubsystem);
    }





}