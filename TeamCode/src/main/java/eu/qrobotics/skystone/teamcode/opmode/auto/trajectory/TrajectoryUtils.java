package eu.qrobotics.skystone.teamcode.opmode.auto.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class TrajectoryUtils {
    static final double STONE_L = 8.0;
    static final double STONE_W = 4.0;
    static final double ROBOT_W = 17;
    static final double ROBOT_L = 13.35;
    static final double PAD = 24.0;
    static final double BOTTOM_WALL = -3.0 * PAD;
    static final double INTAKE_L = 4.50;
    static final double INTAKE_CENTER_DISTANCE = ROBOT_L / 2 + INTAKE_L;
    static final double STONE_Y = -21;
    static final double COLLECT_ANGLE = Math.toRadians(145);
    static final double STONE_SPLINE_DISTANCE = INTAKE_CENTER_DISTANCE + 1;

    public static final Pose2d START_POSE_STONES = new Pose2d(-33.0, -63.0, Math.toRadians(90.0));
    public static final Pose2d START_POSE_FOUNDATION = new Pose2d(33.0, -63.0, Math.toRadians(270.0));

    public enum Alliance {
        RED,
        BLUE
    }

    public static Pose2d flipIfBlue(Alliance alliance, Pose2d pose) {
        if(alliance == Alliance.RED)
            return pose;
        return new Pose2d(pose.getX(), -pose.getY(), -pose.getHeading());
    }

    public static Vector2d flipIfBlue(Alliance alliance, Vector2d vector) {
        if(alliance == Alliance.RED)
            return vector;
        return new Vector2d(vector.getX(), -vector.getY());
    }

    public static Pose2d flipIfBluePositive(Alliance alliance, Pose2d pose) {
        Pose2d flipped = flipIfBlue(alliance, pose);
        if(flipped.getHeading() == Math.toRadians(-180)) {
            flipped = new Pose2d(flipped.getX(), flipped.getY(), Math.toRadians(180));
        }
        return flipped;
    }
}
