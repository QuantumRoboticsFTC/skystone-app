package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

@Config
public class DriveConstants {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static final boolean RUN_USING_ENCODER = true;
    public static final PIDCoefficients MOTOR_VELO_PID = null;

    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 1;

    public static double kV = 1.0 / rpmToVelocity(getMaxRpm());
    public static double kA = 0.0;
    public static double kStatic = 0.0;

    public static double getTicksPerRev() {
        return 753.2;
    }

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / getTicksPerRev();
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return 223;
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        return (getMaxRpm() * getTicksPerRev() / 60.0);
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / getTicksPerSec();
    }
}
