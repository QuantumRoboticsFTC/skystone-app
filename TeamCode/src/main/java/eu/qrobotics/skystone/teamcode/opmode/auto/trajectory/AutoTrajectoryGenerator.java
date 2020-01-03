package eu.qrobotics.skystone.teamcode.opmode.auto.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.flipIfBlue;
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
        } else  {
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
