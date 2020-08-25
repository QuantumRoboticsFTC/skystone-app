package com.visualizer

import com.acmerobotics.roadrunner.trajectory.*

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.path.PathBuilder
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints
import com.acmerobotics.roadrunner.util.Angle
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.mirrorForAlliance
import kotlin.math.PI

private fun zeroPosition(state: MotionState) = MotionState(0.0, state.v, state.a, state.j)

private data class ConstraintsInterval(
    val start: Double,
    val end: Double,
    val constraints: TrajectoryConstraints
)

private class MergedTrajectoryConstraints(
    val baseConstraints: TrajectoryConstraints,
    val constraintsOverrideIntervals: List<ConstraintsInterval>
) : TrajectoryConstraints {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, secondDeriv: Pose2d): SimpleMotionConstraints {
        for (interval in constraintsOverrideIntervals) {
            val (start, end, constraints) = interval
            if (s in start..end) {
                return constraints[s, pose, deriv, secondDeriv]
            }
        }
        return baseConstraints[s, pose, deriv, secondDeriv]
    }
}

/**
 * Builder for trajectories with *dynamic* constraints.
 */
class MirrorTrajectoryBuilder private constructor(
        startPose: Pose2d?,
        startTangent: Double?,
        trajectory: Trajectory?,
        t: Double?,
        private val alliance: TrajectoryUtils.Alliance,
        private val constraints: TrajectoryConstraints,
        private val start: MotionState,
        private val resolution: Double
){
    /**
     * Create a builder from a start pose and motion state. This is the recommended constructor for creating
     * trajectories from rest.
     */
    @JvmOverloads constructor(
        alliance: TrajectoryUtils.Alliance,
        startPose: Pose2d,
        startTangent: Double = startPose.heading,
        constraints: TrajectoryConstraints,
        resolution: Double = 0.25
    ) : this(startPose, startTangent, null, null, alliance, constraints, MotionState(0.0, 0.0, 0.0), resolution)

    @JvmOverloads constructor(
        alliance: TrajectoryUtils.Alliance,
        startPose: Pose2d,
        reversed: Boolean,
        constraints: TrajectoryConstraints,
        resolution: Double = 0.25
    ) : this(alliance, startPose, Angle.norm(startPose.heading + if (reversed) PI else 0.0), constraints, resolution)

    /**
     * Create a builder from an active trajectory. This is useful for interrupting a live trajectory and smoothly
     * transitioning to a new one.
     */
    @JvmOverloads constructor(
        alliance: TrajectoryUtils.Alliance,
        trajectory: Trajectory,
        t: Double,
        constraints: TrajectoryConstraints,
        resolution: Double = 0.25
    ) : this(null, null, trajectory, t, alliance, constraints, zeroPosition(trajectory.profile[t]), resolution)

    protected var pathBuilder: PathBuilder = if (startPose == null) {
        PathBuilder(trajectory!!.path, trajectory.profile[t!!].x)
    } else {
        PathBuilder(startPose, startTangent!!)
    }

    private val temporalMarkers = mutableListOf<TemporalMarker>()
    private val displacementMarkers = mutableListOf<DisplacementMarker>()
    private val spatialMarkers = mutableListOf<SpatialMarker>()
    
    private val constraintsOverrideIntervals = mutableListOf<ConstraintsInterval>()

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineTo(endPosition: Vector2d):MirrorTrajectoryBuilder{
        pathBuilder.lineTo(mirrorForAlliance(alliance, endPosition))

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineToConstantHeading(endPosition: Vector2d):MirrorTrajectoryBuilder{
        pathBuilder.lineToConstantHeading(mirrorForAlliance(alliance, endPosition))

        return this
    }

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToLinearHeading(endPose: Pose2d):MirrorTrajectoryBuilder{
        pathBuilder.lineToLinearHeading(mirrorForAlliance(alliance, endPose))

        return this
    }

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToSplineHeading(endPose: Pose2d):MirrorTrajectoryBuilder{
        pathBuilder.lineToSplineHeading(mirrorForAlliance(alliance, endPose))

        return this
    }

    /**
     * Adds a strafe path segment.
     *
     * @param endPosition end position
     */
    fun strafeTo(endPosition: Vector2d):MirrorTrajectoryBuilder{
        pathBuilder.strafeTo(mirrorForAlliance(alliance, endPosition))

        return this
    }

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double):MirrorTrajectoryBuilder{
        pathBuilder.forward(distance)

        return this
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     */
    fun back(distance: Double):MirrorTrajectoryBuilder{
        pathBuilder.back(distance)

        return this
    }

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     */
    fun strafeLeft(distance: Double):MirrorTrajectoryBuilder{
        pathBuilder.strafeLeft(distance)

        return this
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     */
    fun strafeRight(distance: Double):MirrorTrajectoryBuilder{
        pathBuilder.strafeRight(distance)

        return this
    }

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineTo(endPosition: Vector2d, endTangent: Double):MirrorTrajectoryBuilder{
        pathBuilder.splineTo(mirrorForAlliance(alliance, endPosition), mirrorForAlliance(alliance, endTangent))

        return this
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineToConstantHeading(endPosition: Vector2d, endTangent: Double):MirrorTrajectoryBuilder{
        pathBuilder.splineToConstantHeading(mirrorForAlliance(alliance, endPosition), mirrorForAlliance(alliance, endTangent))

        return this
    }

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToLinearHeading(endPose: Pose2d, endTangent: Double):MirrorTrajectoryBuilder{
        pathBuilder.splineToLinearHeading(mirrorForAlliance(alliance, endPose), mirrorForAlliance(alliance, endTangent))

        return this
    }

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToSplineHeading(endPose: Pose2d, endTangent: Double):MirrorTrajectoryBuilder{
        pathBuilder.splineToSplineHeading(mirrorForAlliance(alliance, endPose), mirrorForAlliance(alliance, endTangent))

        return this
    }

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun lineTo(endPosition: Vector2d, constraintsOverride: TrajectoryConstraints): MirrorTrajectoryBuilder {
        val start = 0.0

        lineTo(mirrorForAlliance(alliance, endPosition))

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToConstantHeading(endPosition: Vector2d, constraintsOverride: TrajectoryConstraints): MirrorTrajectoryBuilder {
        val start = 0.0

        lineToConstantHeading(mirrorForAlliance(alliance, endPosition))

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToLinearHeading(
        endPose: Pose2d,
        constraintsOverride: TrajectoryConstraints
    ): MirrorTrajectoryBuilder {
        val start = 0.0

        lineToLinearHeading(mirrorForAlliance(alliance, endPose))

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToSplineHeading(
        endPose: Pose2d,
        constraintsOverride: TrajectoryConstraints
    ): MirrorTrajectoryBuilder {
        val start = 0.0

        lineToLinearHeading(mirrorForAlliance(alliance, endPose))

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a strafe path segment.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeTo(endPosition: Vector2d, constraintsOverride: TrajectoryConstraints): MirrorTrajectoryBuilder {
        val start = 0.0

        strafeTo(mirrorForAlliance(alliance, endPosition))

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     * @param constraintsOverride segment-specific constraints
     */
    fun forward(distance: Double, constraintsOverride: TrajectoryConstraints): MirrorTrajectoryBuilder {
        val start = 0.0

        forward(distance)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     * @param constraintsOverride segment-specific constraints
     */
    fun back(distance: Double, constraintsOverride: TrajectoryConstraints) =
        forward(-distance, constraintsOverride)

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeLeft(distance: Double, constraintsOverride: TrajectoryConstraints): MirrorTrajectoryBuilder {
        val start = 0.0

        strafeLeft(distance)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeRight(distance: Double, constraintsOverride: TrajectoryConstraints) =
        strafeLeft(-distance, constraintsOverride)

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     * @param constraintsOverride segment-specific constraints
     */
    fun splineTo(
        endPosition: Vector2d,
        endTangent: Double,
        constraintsOverride: TrajectoryConstraints
    ): MirrorTrajectoryBuilder {
        val start = 0.0

        splineTo(mirrorForAlliance(alliance, endPosition), mirrorForAlliance(alliance, endTangent))

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     * @param constraintsOverride segment-specific constraints
     */
    fun splineToConstantHeading(
        endPosition: Vector2d,
        endTangent: Double,
        constraintsOverride: TrajectoryConstraints
    ): MirrorTrajectoryBuilder {
        val start = 0.0

        splineToConstantHeading(mirrorForAlliance(alliance, endPosition), mirrorForAlliance(alliance, endTangent))

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     * @param constraintsOverride segment-specific constraints
     */
    fun splineToLinearHeading(
        endPose: Pose2d,
        endTangent: Double,
        constraintsOverride: TrajectoryConstraints
    ): MirrorTrajectoryBuilder {
        val start = 0.0

        splineToLinearHeading(mirrorForAlliance(alliance, endPose), mirrorForAlliance(alliance, endTangent))

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun splineToSplineHeading(
        endPose: Pose2d,
        endHeading: Double,
        constraintsOverride: TrajectoryConstraints
    ): MirrorTrajectoryBuilder {
        val start = 0.0

        splineToSplineHeading(mirrorForAlliance(alliance, endPose), mirrorForAlliance(alliance, endHeading))

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a marker to the trajectory at [time].
     */
    fun addTemporalMarker(time: Double, callback: MarkerCallback) =
        addTemporalMarker(0.0, time, callback)

    /**
     * Adds a marker to the trajectory at [scale] * trajectory duration + [offset].
     */
    fun addTemporalMarker(scale: Double, offset: Double, callback: MarkerCallback) =
        addTemporalMarker({ scale * it + offset }, callback)

    /**
     * Adds a marker to the trajectory at [time] evaluated with the trajectory duration.
     */
    fun addTemporalMarker(time: (Double) -> Double, callback: MarkerCallback):MirrorTrajectoryBuilder{
        temporalMarkers.add(TemporalMarker(time, callback))

        return this
    }

    /**
     * Adds a marker that will be triggered at the closest trajectory point to [point].
     */
    fun addSpatialMarker(point: Vector2d, callback: MarkerCallback):MirrorTrajectoryBuilder{
        spatialMarkers.add(SpatialMarker(mirrorForAlliance(alliance, point), callback))

        return this
    }

    /**
     * Adds a marker at the current position of the trajectory.
     */
    fun addDisplacementMarker(callback: MarkerCallback) =
        addDisplacementMarker(pathBuilder.build().length(), callback)

    /**
     * Adds a marker to the trajectory at [displacement].
     */
    fun addDisplacementMarker(displacement: Double, callback: MarkerCallback) =
        addDisplacementMarker(0.0, displacement, callback)

    /**
     * Adds a marker to the trajectory at [scale] * path length + [offset].
     */
    fun addDisplacementMarker(scale: Double, offset: Double, callback: MarkerCallback) =
        addDisplacementMarker({ scale * it + offset }, callback)

    /**
     * Adds a marker to the trajectory at [displacement] evaluated with path length.
     */
    fun addDisplacementMarker(displacement: (Double) -> Double, callback: MarkerCallback):MirrorTrajectoryBuilder{
        displacementMarkers.add(DisplacementMarker(displacement, callback))

        return this
    }

    fun length() = pathBuilder.build().length()

    /**
     * Constructs the [Trajectory] instance.
     */
    fun build() = buildTrajectory(pathBuilder.build(), temporalMarkers, displacementMarkers, spatialMarkers)

    fun buildTrajectory(
        path: Path,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): Trajectory {
        val goal = MotionState(path.length(), 0.0, 0.0)
        return TrajectoryGenerator.generateTrajectory(
            path,
            MergedTrajectoryConstraints(constraints,
                mutableListOf<ConstraintsInterval>().apply { addAll(constraintsOverrideIntervals) }),
            start,
            goal,
            temporalMarkers,
            displacementMarkers,
            spatialMarkers,
            resolution
        )
    }
}
