package eu.qrobotics.skystone.teamcode.opmode.auto.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.BOTTOM_WALL;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.ROBOT_L;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.ROBOT_W;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.STONE_L;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.STONE_W;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.STONE_Y;
import static eu.qrobotics.skystone.teamcode.subsystems.DriveConstants.BASE_CONSTRAINTS;
import static eu.qrobotics.skystone.teamcode.subsystems.DriveConstants.SLOW_CONSTRAINTS;

public class AutoTrajectoryGenerator {
    private TrajectoryUtils.Alliance alliance;
    private Pose2d startPose;

    public AutoTrajectoryGenerator(TrajectoryUtils.Alliance alliance, Pose2d start) {
        this.alliance = alliance;
        startPose = start;
    }

    public enum SkystonePattern {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public TrajectoryUtils.Alliance getAlliance() {
        return alliance;
    }

    public List<Trajectory> getTrajectoriesMoveFoundation() {
        SkystoneTrajectoryBuilder.reset(startPose);
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder()
                .toFoundation()
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .moveFoundation()
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .park()
                .build());

        return trajectories;
    }

    public List<Trajectory> getTrajectories1Stone(SkystonePattern skystonePattern) {
        int firstSkystone;
        if (skystonePattern == SkystonePattern.LEFT) {
            firstSkystone = 3;
        } else if (skystonePattern == SkystonePattern.MIDDLE) {
            firstSkystone = 4;
        } else {
            firstSkystone = 2;
        }

        SkystoneTrajectoryBuilder.reset(startPose);
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(Speed.SLOW)
                .getStone(firstSkystone)
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .actualSetReversed(true)
                .passBridge()
                .toFoundation()
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .moveFoundation()
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .actualSetReversed(false)
                .park()
                .build());

        return trajectories;
    }

    public List<Trajectory> getTrajectories2Stones(SkystonePattern skystonePattern) {
        int firstSkystone;
        int secondSkystone;
        if (skystonePattern == SkystonePattern.LEFT) {
            firstSkystone = 3;
            secondSkystone = 0;
        } else if (skystonePattern == SkystonePattern.MIDDLE) {
            firstSkystone = 4;
            secondSkystone = 1;
        } else {
            firstSkystone = 5;
            secondSkystone = 2;
        }

        SkystoneTrajectoryBuilder.reset(startPose);
        List<Trajectory> trajectories = new ArrayList<>();

        trajectories.add(makeTrajectoryBuilder(Speed.SLOW)
                .getStone(firstSkystone)
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .actualSetReversed(true)
                .passBridge()
                .toFoundation()
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .moveFoundation()
                .build());

        trajectories.add(makeTrajectoryBuilder(Speed.SLOW)
                .passBridge()
                .getStone(secondSkystone)
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .actualSetReversed(true)
                .passBridge()
                .actualStrafeTo(new Vector2d(45.0, -39.0))
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .actualSetReversed(false)
                .park()
                .build());

        return trajectories;
    }

    public List<Trajectory> getTrajectoriesSideArm2Stones(SkystonePattern skystonePattern) {
        SkystoneTrajectoryBuilder.reset(startPose);

        List<Trajectory> trajectories = new ArrayList<>();

        int[] stonesOrder = new int[]{};
        if (skystonePattern == SkystonePattern.LEFT) {
            stonesOrder = new int[]{0, 3};
        } else if (skystonePattern == SkystonePattern.MIDDLE) {
            stonesOrder = new int[]{1, 4};
        } else if (skystonePattern == SkystonePattern.RIGHT) {
            stonesOrder = new int[]{2, 5};
        }

        Pose2d firstSkystone = new Pose2d(BOTTOM_WALL + stonesOrder[0] * STONE_L + STONE_L / 2 + ROBOT_L / 2 - 3.5 + 1 - (alliance == Alliance.BLUE ? 1 : 0), STONE_Y - STONE_W / 2 - ROBOT_W / 2 - 2.5, Math.toRadians(180));
        Pose2d secondSkystone = new Pose2d(BOTTOM_WALL + stonesOrder[1] * STONE_L + STONE_L / 2 + ROBOT_L / 2 - 3.5 + 4 - (alliance == Alliance.BLUE ? 1 : 0), STONE_Y - STONE_W / 2 - ROBOT_W / 2 - 1.5, Math.toRadians(180));

        if (skystonePattern == SkystonePattern.LEFT && alliance == Alliance.RED) {
            trajectories.add(makeTrajectoryBuilder()
                    .actualSetReversed(false)
                    .actualStrafeTo(new Vector2d(-36.0, -63.0))
                    .actualSplineTo(firstSkystone)
                    .build());
        } else {
            trajectories.add(makeTrajectoryBuilder()
                    .actualSetReversed(false)
                    .actualSplineTo(firstSkystone)
                    .build());
        }
        trajectories.add(makeTrajectoryBuilder()
                .actualSetReversed(true)
                .actualSplineTo(new Pose2d(-5, -31.0 - 4 - 7.5, Math.toRadians(180)))
                .actualSplineTo(new Pose2d(5, -31.0 - 4 - 7.5, Math.toRadians(180)))
                .actualSplineTo(new Pose2d(49.5, -31.0 - 4.25 + (alliance == Alliance.BLUE ? 2 : 0), Math.toRadians(180)))
                .build());

        if (skystonePattern == SkystonePattern.RIGHT) {
            trajectories.add(makeTrajectoryBuilder()
                    .actualSetReversed(false)
                    .actualSplineTo(new Pose2d(5, -31.0 - 2 - 7.5, Math.toRadians(180)))
                    .actualSplineTo(secondSkystone)
                    .build());
            trajectories.add(makeTrajectoryBuilder()
                    .actualSetReversed(false)
                    .actualForward(6)
                    .actualSetReversed(true)
                    .actualSplineTo(new Pose2d(0, -31.0 - 3.5 - 8, Math.toRadians(180)))
                    .actualSplineTo(new Pose2d(49.5, -31.0 - 1.5 + (alliance == Alliance.BLUE ? 2 : 0), Math.toRadians(180)))
                    .build());
        } else {
            trajectories.add(makeTrajectoryBuilder()
                    .actualSetReversed(false)
                    .actualSplineTo(new Pose2d(5, -31.0 - 2 - 7.5, Math.toRadians(180)))
                    .actualSplineTo(new Pose2d(-5, -31.0 - 2 - 7.5, Math.toRadians(180)))
                    .actualSplineTo(secondSkystone)
                    .build());
            trajectories.add(makeTrajectoryBuilder()
                    .actualSetReversed(true)
                    .actualSplineTo(new Pose2d(-5, -31.0 - 3.5 - 7.5, Math.toRadians(180)))
                    .actualSplineTo(new Pose2d(5, -31.0 - 3.5 - 7.5, Math.toRadians(180)))
                    .actualSplineTo(new Pose2d(49.5, -31.0 - 1.5 + (alliance == Alliance.BLUE ? 2 : 0), Math.toRadians(180)))
                    .build());
        }

        trajectories.add(makeTrajectoryBuilder()
                .actualSetReversed(false)
                .actualStrafeTo(new Vector2d(49.5, -31.0 - 1.5 + (alliance == Alliance.BLUE ? 2 : 0) - 2))
                .actualSplineTo(new Pose2d(43, -44, Math.toRadians(-90)))
                .actualStrafeTo(new Vector2d(43, -28 + (alliance == Alliance.BLUE ? 6 : 0)))
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .actualSplineTo(new Pose2d(24.0, -52.0, Math.toRadians(180.0)))
                .actualStrafeTo(new Vector2d(45.0, -52.0))
                .build());
        trajectories.add(makeTrajectoryBuilder()
                .park()
                .build());

        return trajectories;
    }

    private enum Speed {
        SLOW,
        NORMAL
    }

    private SkystoneTrajectoryBuilder makeTrajectoryBuilder() {
        return makeTrajectoryBuilder(Speed.NORMAL);
    }

    private SkystoneTrajectoryBuilder makeTrajectoryBuilder(Speed speed) {
        switch (speed) {
            case SLOW:
                return new SkystoneTrajectoryBuilder(SLOW_CONSTRAINTS, alliance);
            case NORMAL:
            default:
                return new SkystoneTrajectoryBuilder(BASE_CONSTRAINTS, alliance);
        }
    }
}
