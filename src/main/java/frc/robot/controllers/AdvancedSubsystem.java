package frc.robot.controllers;

public abstract class AdvancedSubsystem {
    public abstract void calibrateInit();
    public abstract void calibrateLoop();
    public abstract boolean calibrateFinished();
}