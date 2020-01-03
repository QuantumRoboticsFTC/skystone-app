package eu.qrobotics.skystone.teamcode.opmode.auto.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;

import org.jetbrains.annotations.NotNull;

import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.Alliance;

import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.BOTTOM_WALL;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.COLLECT_ANGLE;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.STONE_L;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.STONE_SPLINE_DISTANCE;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.STONE_W;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.STONE_Y;
import static eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.flipIfBlue;

public class SkystoneTrajectoryBuilder extends PublicTrajectoryBuilder {

    public SkystoneTrajectoryBuilder(@NotNull TrajectoryConstraints trajectoryConstraints, Alliance alliance) {
        super(flipIfBlue(alliance, currentPose), trajectoryConstraints);
        this.alliance = alliance;
        actualSetReversed(isReversed);
    }

    private Alliance alliance;
    private static Pose2d currentPose = new Pose2d(0, 0, 0);
    private static boolean isReversed = false;
    private static Pose2d precision = new Pose2d(0, 0, 0);

    public static void reset(Pose2d startPose) {
        currentPose = startPose;
    }

    /***
     * Creates a trajectory from the location to the target skystone
     * @param skystoneNumber the skystone to get (0 = next to wall, 5 = next to skybridge)
     * @return the TrajectoryBuilder with the path
     */
    public SkystoneTrajectoryBuilder getStone(int skystoneNumber) {
        if (skystoneNumber < 0)
            skystoneNumber = 0;
        if (skystoneNumber > 5)
            skystoneNumber = 5;
        skystoneNumber++;
        Vector2d skystoneCorner = new Vector2d(BOTTOM_WALL + skystoneNumber * STONE_L, STONE_Y - STONE_W / 2);

        boolean previousReversed = isReversed;

        return actualSetReversed(false)
                .actualSplineTo(new Pose2d(skystoneCorner.getX() + STONE_SPLINE_DISTANCE * Math.cos(Math.PI - COLLECT_ANGLE), skystoneCorner.getY() - 16, COLLECT_ANGLE))
                .actualStrafeTo(new Vector2d(skystoneCorner.getX() + STONE_SPLINE_DISTANCE * Math.cos(Math.PI - COLLECT_ANGLE), skystoneCorner.getY() - STONE_SPLINE_DISTANCE * Math.sin(Math.PI - COLLECT_ANGLE)))
                .actualForward(4)
                .actualSetReversed(previousReversed);
    }

    public SkystoneTrajectoryBuilder bridgeSafeUp() {
        if (currentPose.getHeading() == 0 || currentPose.getHeading() == Math.PI || currentPose.getHeading() == -Math.PI)
            return actualStrafeTo(new Vector2d(Math.max(currentPose.getX() - 10, 20), -36.0));
        if (currentPose.getHeading() < -Math.PI / 2)
            return actualSplineTo(new Pose2d(Math.max(currentPose.getX() - 10, 20), -36.0, -Math.PI));
        if (currentPose.getHeading() > Math.PI / 2)
            return actualSplineTo(new Pose2d(Math.max(currentPose.getX() - 10, 20), -36.0, -Math.PI));
        return actualSplineTo(new Pose2d(Math.max(currentPose.getX() - 10, 20), -36.0, 0));
    }

    public SkystoneTrajectoryBuilder bridgeSafeDown() {
        if (currentPose.getHeading() == 0 || currentPose.getHeading() == Math.PI || currentPose.getHeading() == -Math.PI)
            return actualStrafeTo(new Vector2d(Math.min(currentPose.getX() + 10, -20), -36.0));
        if (currentPose.getHeading() < -Math.PI / 2)
            return actualSplineTo(new Pose2d(Math.min(currentPose.getX() + 10, -20), -36.0, -Math.PI));
        if (currentPose.getHeading() > Math.PI / 2)
            return actualSplineTo(new Pose2d(Math.min(currentPose.getX() + 10, -20), -36.0, -Math.PI));
        return actualSplineTo(new Pose2d(Math.min(currentPose.getX() + 10, -20), -36.0, 0));
    }



    public SkystoneTrajectoryBuilder bridgeSafe() {
        if(currentPose.getX() > 0)
            return bridgeSafeUp();
        if(currentPose.getX() < 0)
            return bridgeSafeDown();
        return actualStrafeTo(new Vector2d(0, -36.0));
    }

    public SkystoneTrajectoryBuilder passBridge() {
        if(currentPose.getX() > 0)
            return bridgeSafe().bridgeSafeDown();
        return bridgeSafe().bridgeSafeUp();
    }

    public SkystoneTrajectoryBuilder toFoundation() {
        boolean previousReversed = isReversed;
        return actualSetReversed(true)
                .actualSplineTo(new Pose2d(41.5, -37.0, Math.toRadians(270.0)))
                .actualStrafeTo(new Vector2d(41.5, -29.0))
                .actualSetReversed(previousReversed);
    }

    public SkystoneTrajectoryBuilder moveFoundation() {
        boolean previousReversed = isReversed;
        return actualSetReversed(false)
                .actualSplineTo(new Pose2d(24.0, -48.0, Math.toRadians(180.0)))
                .actualStrafeTo(new Vector2d(45.0, -48.0))
                .actualSetReversed(previousReversed);
    }

    public SkystoneTrajectoryBuilder park() {
        return bridgeSafe()
                .actualStrafeTo(new Vector2d(5, -36.0));
    }

    public SkystoneTrajectoryBuilder actualSetReversed(boolean reversed) {
        setReversed(reversed);
        isReversed = reversed;
        return this;
    }

    public SkystoneTrajectoryBuilder actualSplineTo(Pose2d pose2d) {
        pose2d = pose2d.minus(precision);
        splineTo(flipIfBlue(alliance, pose2d));
        currentPose = pose2d;
        return this;
    }

    public SkystoneTrajectoryBuilder actualStrafeTo(Vector2d vector2d) {
        vector2d = vector2d.minus(precision.vec());
        strafeTo(flipIfBlue(alliance, vector2d));
        currentPose = new Pose2d(vector2d, currentPose.getHeading());
        return this;
    }

    public SkystoneTrajectoryBuilder actualForward(double distance) {
        return actualStrafeTo(new Vector2d(currentPose.getX() + distance * currentPose.headingVec().getX(), currentPose.getY() + distance * currentPose.headingVec().getY()));
    }

    public SkystoneTrajectoryBuilder actualLineTo(Vector2d vector2d) {
        return actualLineTo(vector2d, new ConstantInterpolator(currentPose.getHeading()));
    }

    public SkystoneTrajectoryBuilder actualLineTo(Vector2d vector2d, HeadingInterpolator interpolator) {
        vector2d = vector2d.minus(precision.vec());
        lineTo(vector2d, interpolator);
        currentPose = new Pose2d(vector2d, interpolator.end());
        return this;
    }

    private static final double SLIDE_SPLINE_RATIO = 0.85;

    public SkystoneTrajectoryBuilder actualSlideTo(Pose2d pose2d) {
        Pose2d splineEnd = mult(pose2d, SLIDE_SPLINE_RATIO).plus(mult(currentPose, 1 - SLIDE_SPLINE_RATIO));
        splineEnd = new Pose2d(splineEnd.getX(), splineEnd.getY(), pose2d.getHeading());
        return actualSplineTo(splineEnd).actualStrafeTo(new Vector2d(pose2d.getX(), pose2d.getY()));
    }

    public SkystoneTrajectoryBuilder actualBack(double distance) {
        return actualForward(-distance);
    }

    /***
     * Adds to precision loss adjusting. Workaround for not having localization
     * @param diff difference between real and known position
     * @return reference for chaining
     */
    public SkystoneTrajectoryBuilder precisionLoss(Pose2d diff) {
        precision = precision.plus(diff);
        return this;
    }

    public SkystoneTrajectoryBuilder precisionLossX(double x) {
        return precisionLoss(new Pose2d(x, 0, 0));
    }

    public SkystoneTrajectoryBuilder precisionLossY(double y) {
        return precisionLoss(new Pose2d(0, y, 0));
    }

    public SkystoneTrajectoryBuilder precisionLossH(double h) {
        return precisionLoss(new Pose2d(0, 0, h));
    }

    public SkystoneTrajectoryBuilder setRealLocation(Pose2d realLocation) {
        precision = realLocation.minus(currentPose);
        return this;
    }

    private Pose2d mult(Pose2d a, double scalar) {
        return new Pose2d(a.getX() * scalar, a.getY() * scalar, a.getHeading());
    }

    private Pose2d div(Pose2d a, double scalar) {
        return mult(a, 1/scalar);
    }
}
