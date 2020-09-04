package eu.qrobotics.skystone.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

@Config
public class DriveConstants {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6.2, 0, 2.7);
    public static PIDCoefficients LATERAL_PID = new PIDCoefficients(8.2, 0, 2);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7.6, 0, 1.6);
    public static final boolean RUN_USING_ENCODER = false;
    public static final PIDCoefficients MOTOR_VELO_PID = null;

    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 17.7;
    public static double WHEEL_BASE = 13.25;
    public static double LATERAL_MULTIPLIER = 1.6;

    public static double kV = 0.015;
    public static double kA = 0.0002;
    public static double kStatic = 0.012;

    public static double getTicksPerRev() {
        return 383.6;
    }

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            80.0, 50.0, 30.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    public static DriveConstraints SLOW_CONSTRAINTS = new DriveConstraints(
            50.0, 20.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / getTicksPerRev();
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return 435;
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        return (getMaxRpm() * getTicksPerRev() / 60.0);
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (getMaxRpm() * getTicksPerRev());
    }
}
