package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.modes.BallTheftAuto;
import frc.robot.auto.modes.CalibrateAuto;
import frc.robot.auto.modes.EightBallAuto;
import frc.robot.auto.modes.FieldDimensions;
import frc.robot.auto.modes.MovementAuto;
import frc.robot.auto.modes.SixBallAuto;
import frc.robot.auto.modes.StandStillMode;
import frc.robot.auto.modes.SumoAuto;
import frc.robot.util.Pose;

/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions 
{
	private static SmartDashboardInteractions instance = null;

	public static SmartDashboardInteractions getInstance() {
		if (instance == null) {
			instance = new SmartDashboardInteractions();
		}
		return instance;
    }
        
    public SmartDashboardInteractions()
    {
        initWithDefaults();
    }


    public void initWithDefaults() 
    {
        startDelayChooser = new SendableChooser<StartDelayOption>();
        startDelayChooser.setDefaultOption(StartDelayOption.DELAY_0_SEC.toString(), StartDelayOption.DELAY_0_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_1_SEC.toString(), StartDelayOption.DELAY_1_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_2_SEC.toString(), StartDelayOption.DELAY_2_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_3_SEC.toString(), StartDelayOption.DELAY_3_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_4_SEC.toString(), StartDelayOption.DELAY_4_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_5_SEC.toString(), StartDelayOption.DELAY_5_SEC);
        SmartDashboard.putData("Auto Start Delay", startDelayChooser);


        autoModeChooser = new SendableChooser<AutoModeOption>();
        autoModeChooser.addOption(AutoModeOption.MOVEMENT_AUTO.toString(), AutoModeOption.MOVEMENT_AUTO);
        autoModeChooser.addOption(AutoModeOption.BALL_THEFT_AUTO.toString(), AutoModeOption.BALL_THEFT_AUTO);
        autoModeChooser.addOption(AutoModeOption.SIX_BALL_AUTO.toString(), AutoModeOption.SIX_BALL_AUTO);
        autoModeChooser.addOption(AutoModeOption.EIGHT_BALL_AUTO.toString(), AutoModeOption.EIGHT_BALL_AUTO);
        autoModeChooser.addOption(AutoModeOption.SUMO_AUTO.toString(), AutoModeOption.SUMO_AUTO);
        autoModeChooser.addOption(AutoModeOption.CALIBRATE_AUTO.toString(), AutoModeOption.CALIBRATE_AUTO);
        autoModeChooser.setDefaultOption(AutoModeOption.SUMO_AUTO.toString(), AutoModeOption.SUMO_AUTO);
        SmartDashboard.putData("Auto Selection", autoModeChooser);

        //Choosing start position
        startPoseChooser = new SendableChooser<StartPoseSelection>();
        startPoseChooser.addOption(StartPoseSelection.PORT_START.name, StartPoseSelection.PORT_START);
        startPoseChooser.addOption(StartPoseSelection.MIDDLE_START.name, StartPoseSelection.MIDDLE_START);
        startPoseChooser.addOption(StartPoseSelection.PLAYER_STATION.name, StartPoseSelection.PLAYER_STATION);
        startPoseChooser.setDefaultOption(StartPoseSelection.PORT_START.name, StartPoseSelection.PORT_START);
        SmartDashboard.putData("Start Pose Selection", startPoseChooser);


        
    }

        

    SendableChooser<StartDelayOption> startDelayChooser;

    public enum StartDelayOption {
        DELAY_0_SEC("0 Sec", 0.0), DELAY_1_SEC("1 Sec", 1.0), DELAY_2_SEC("2 Sec", 2.0), DELAY_3_SEC("3 Sec", 3.0),
        DELAY_4_SEC("4 Sec", 4.0), DELAY_5_SEC("5 Sec", 5.0);

        public final String name;
        public final double delaySec;

        StartDelayOption(final String name, final double delaySec) {
            this.name = name;
            this.delaySec = delaySec;
        }
    }

    public double getStartDelay() {
        return startDelayChooser.getSelected().delaySec;
    }

    SendableChooser<AutoModeOption> autoModeChooser;

    enum AutoModeOption {
        MOVEMENT_AUTO("Movement Auto"),
        BALL_THEFT_AUTO("Ball Theft Auto"),
        SIX_BALL_AUTO("Movement Auto"),
        EIGHT_BALL_AUTO("Movement Auto"),
        SUMO_AUTO("Movement Auto"),
        CALIBRATE_AUTO("Calibrate Auto");

        public final String name;

        AutoModeOption(final String name) {
            this.name = name;
        }
    }

    public AutoModeBase getAutoModeSelection() {
        final AutoModeOption autoMode = (AutoModeOption) autoModeChooser.getSelected();

        switch (autoMode) {
        case MOVEMENT_AUTO:
            return new MovementAuto();
        case BALL_THEFT_AUTO:
            return new BallTheftAuto();
        case SIX_BALL_AUTO:
            return new SixBallAuto();
        case EIGHT_BALL_AUTO:
            return new EightBallAuto();
        case SUMO_AUTO:
            return new SumoAuto();
        case CALIBRATE_AUTO:
            return new CalibrateAuto();
        default:
            System.out.println("ERROR: unexpected auto mode: " + autoMode);
            return new StandStillMode();
        }
    }

    SendableChooser<StartPoseSelection> startPoseChooser;
    public enum StartPoseSelection {
        PORT_START("Port Position", FieldDimensions.portStartPose),
        MIDDLE_START("Middle Position", FieldDimensions.middleStartPose),
        PLAYER_STATION("Player Station", FieldDimensions.playerStationStartPose);


        public final String name;
        public final Pose pose;

        StartPoseSelection(final String name, final Pose pose){
            this.name = name;
            this.pose = pose;
        }
    }

    public Pose getSelectedStartPose(){
        return startPoseChooser.getSelected().pose;
    }
}
   


