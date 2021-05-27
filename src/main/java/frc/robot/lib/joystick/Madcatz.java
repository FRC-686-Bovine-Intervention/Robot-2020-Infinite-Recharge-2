package frc.robot.lib.joystick;

import frc.robot.lib.util.DataLogger;

public class Madcatz extends JoystickBase
{
    public static int kLeftXAxis =              0;
    public static int kLeftYAxis =              1;
    public static int kRightXAxis =             4;
    public static int kRightYAxis =             5;
    public static int kAButton =                0;

    public int port;

    // constructor
    public Madcatz(int _port)
    {
        super(_port);
        port = _port;
    }

    public DataLogger getLogger() { return logger; }
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            // TODO: Fill this in
        }
    };
}