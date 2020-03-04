package frc.robot.ControlStructures;




public abstract class AdvancedSubsystem extends Subsystem{
    public abstract void calibrateInit();
    public abstract void calibrateLoop();
    public abstract boolean calibrateFinished();
}