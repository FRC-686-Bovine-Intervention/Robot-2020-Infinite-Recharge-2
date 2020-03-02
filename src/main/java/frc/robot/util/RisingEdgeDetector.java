package frc.robot.util;

public class RisingEdgeDetector
{
    private boolean lastValue = false;
    private boolean edge = false;

    public boolean update(boolean newValue) 
    {
        edge = (newValue && !lastValue);
        lastValue = newValue;
        return edge;
    }

    public boolean get()
    {
        return edge;
    }
}