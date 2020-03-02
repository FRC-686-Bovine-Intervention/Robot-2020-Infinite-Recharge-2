package frc.robot.util;



public class FallingEdgeDetector
{
    private boolean lastValue = false;

    public boolean update(boolean newValue) 
    {
        boolean rv = (!newValue && lastValue);
        lastValue = newValue;
        return rv;
    }
}