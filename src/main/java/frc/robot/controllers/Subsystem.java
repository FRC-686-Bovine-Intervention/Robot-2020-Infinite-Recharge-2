package frc.robot.controllers;


public abstract class Subsystem {
    public abstract void init();
    public abstract void run();
    public abstract void zeroSensors();
    public abstract void updateSmartDashboard();
}