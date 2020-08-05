package frc.robot.lib.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import frc.robot.subsystems.ConveyorBelt;

public class Pigeon extends GyroBase 
{
	// singleton class
        private static Pigeon instance = null;
        public static Pigeon getInstance() 
        { 
            if (instance == null) {
                instance = new Pigeon();
            }
            return instance;
        }

        PigeonIMU pigeon;

        // yaw/picth/roll constants
        final int kYAW = 0;
        final int kPITCH = 1;
        final int kROLL = 2;
        final int kYPR_SIZE = kROLL + 1;

        // calibration values
        double calPitch = 0.0;
        double calRoll = 0.0;

        double physicalStartAngle = 0;

        // constructors
        public Pigeon() 
        {
                pigeon = new PigeonIMU(ConveyorBelt.getInstance().vBeltLeft);
        }

        /**
         * Returns heading for the GyroBase class.
         *
         */
        public double getHeadingDeg() 
        {
                double[] ypr = new double[kYPR_SIZE]; // yaw/pitch/roll array
                pigeon.getYawPitchRoll(ypr); // fill array
                double yaw = ypr[kYAW];
                return yaw;
        }

        public double getPitchDeg()
        {
                double[] ypr = new double[kYPR_SIZE]; // yaw/pitch/roll array
                pigeon.getYawPitchRoll(ypr); // fill array
                double pitch = ypr[kPITCH];
                return pitch - calPitch;
        }

        public double getRollDeg()
        {
                double[] ypr = new double[kYPR_SIZE]; // yaw/pitch/roll array
                pigeon.getYawPitchRoll(ypr); // fill array
                double roll = ypr[kROLL];
                return roll - calRoll;
        }

        @Override
        public void zeroSensor() 
        {
                pigeon.setYaw(0.0, Constants.kTalonTimeoutMs);

                double[] ypr = new double[kYPR_SIZE]; // yaw/pitch/roll array
                pigeon.getYawPitchRoll(ypr); // fill array
                calPitch = ypr[kPITCH];
                calRoll = ypr[kROLL];
        }
}
