package frc.robot.Subsystems.Shooter;

import frc.robot.Constants;

public class Utility {
    private Utility(){}




    public static double bound(double value, double[] range){
        return value<range[0] ? range[0] : value >range[1] ? range[1] : value;
    }


    //Unit Conversion:
    public static int radiansToEncoderUnits(double radians, double encoderUnitsPerRev){
        return (int)(radians*(encoderUnitsPerRev/(Math.PI*2.0)));
    }

    /**
     * Hello!
     * @param rps   radians per seconds
     * @param encoderUnitsPerRev 
     * @return  The rps in encoder units per frame
     */
    public static int rpsToUPF(double rps, double encoderUnitsPerRev){
        return (int)(rps*(encoderUnitsPerRev/(Math.PI*2.0))*(Constants.kTalonLoopPeriodSec));
    }


    /**
     * Hello2!
     * @param rpsps radians per sec per sec
     * @param encoderUnitsPerRev
     * @return The rpsps as encoder units per frame per frame
     */
    public static int rpspsToUPFPF(double rpsps, double encoderUnitsPerRev){
        return (int)(rpsps*(encoderUnitsPerRev/(Math.PI*2.0))*(Constants.kTalonLoopPeriodSec*Constants.kTalonLoopPeriodSec));
    }





    
    public static double encoderUnitsToRadians(int encoderUnits, double encoderUnitsPerRev){
        return ((double)encoderUnits*((Math.PI*2.0)/encoderUnitsPerRev));
    }

    public static double upfToRPS(int UPF, double encoderUnitsPerRev){
        return ((double)UPF*((Math.PI*2.0)/encoderUnitsPerRev)*(1.0/Constants.kTalonLoopPeriodSec));
    }

    public static double upfpfToRPSPS(int upfpf, double encoderUnitsPerRev){
        return ((double)upfpf*((Math.PI*2.0)/encoderUnitsPerRev)*(1.0/(Constants.kTalonLoopPeriodSec*Constants.kTalonLoopPeriodSec)));
    }
}