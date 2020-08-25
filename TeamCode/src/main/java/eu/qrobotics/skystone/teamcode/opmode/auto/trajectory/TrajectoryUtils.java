package eu.qrobotics.skystone.teamcode.opmode.auto.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

public class TrajectoryUtils {
    public static final double STONE_L = 8.0;
    public static final double STONE_W = 4.0;
    public static final double ROBOT_W = 18;
    public static final double ROBOT_L = 18;
    public static final double PAD = 24.0;
    public static final double BOTTOM_WALL = -3.0 * PAD;
    public static final double INTAKE_L = -6;
    public static final double INTAKE_CENTER_DISTANCE = ROBOT_L / 2 + INTAKE_L;
    public static final double STONE_Y = -22;
    public static final double COLLECT_ANGLE = Math.toRadians(145);
    public static final double STONE_SPLINE_DISTANCE = INTAKE_CENTER_DISTANCE + 1;

    public static final Pose2d START_POSE_STONES = new Pose2d(-33.0, -63.0, Math.toRadians(90.0));
    public static final Pose2d START_POSE_FOUNDATION = new Pose2d(33.0, -63.0, Math.toRadians(270.0));
    public static final Pose2d START_POSE_SIDE = new Pose2d(-33.0, -63.0, Math.toRadians(90.0));

    public enum Alliance {
        RED,
        BLUE
    }
    public enum SkystonePattern {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static Pose2d mirrorForAlliance(Alliance alliance, Pose2d pose) {
        if (alliance == Alliance.RED)
            return pose;
        return new Pose2d(pose.getX(), -pose.getY(), -pose.getHeading());
    }

    public static Vector2d mirrorForAlliance(Alliance alliance, Vector2d vector) {
        if (alliance == Alliance.RED)
            return vector;
        return new Vector2d(vector.getX(), -vector.getY());
    }

    public static double mirrorForAlliance(Alliance alliance, double angle) {
        if (alliance == Alliance.RED)
            return angle;
        return -angle;
    }

    public static Pose2d getStoneSideArm(int stoneNumber) {
        return new Pose2d(BOTTOM_WALL + stoneNumber * STONE_L + STONE_L / 2 + ROBOT_L / 2 - 2.5, STONE_Y - STONE_W / 2 - ROBOT_W / 2 - 3, Math.toRadians(180));
    }

    public static final double STONE_INTAKE_ANGLE = Math.toRadians(135);

    public static Pose2d getStoneIntake(int stoneNumber) {
        return new Pose2d(BOTTOM_WALL + stoneNumber * STONE_L + STONE_L + Math.sin(STONE_INTAKE_ANGLE) * INTAKE_CENTER_DISTANCE, STONE_Y - STONE_W / 2 + Math.cos(STONE_INTAKE_ANGLE) * INTAKE_CENTER_DISTANCE + 2, STONE_INTAKE_ANGLE);
    }
}
