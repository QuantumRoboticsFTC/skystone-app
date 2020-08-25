package com.visualizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints
import eu.qrobotics.skystone.teamcode.opmode.auto.trajectory.TrajectoryUtils.*
import eu.qrobotics.skystone.teamcode.subsystems.DriveConstants.BASE_CONSTRAINTS
import eu.qrobotics.skystone.teamcode.subsystems.DriveConstants.SLOW_CONSTRAINTS


class MultiTrajectoryBuilder(private val alliance: Alliance, private val startPose: Pose2d) {
    var trajectoryBuilders: MutableList<MirrorTrajectoryBuilder> = ArrayList()

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineTo(endPosition: Vector2d): MultiTrajectoryBuilder {
        trajectoryBuilders.last().lineTo(endPosition)

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineToConstantHeading(endPosition: Vector2d): MultiTrajectoryBuilder{
        trajectoryBuilders.last().lineToConstantHeading(endPosition)

        return this
    }

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToLinearHeading(endPose: Pose2d): MultiTrajectoryBuilder{
        trajectoryBuilders.last().lineToLinearHeading(endPose)

        return this
    }

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPose end pose
     */
    fun lineToSplineHeading(endPose: Pose2d): MultiTrajectoryBuilder{
        trajectoryBuilders.last().lineToSplineHeading(endPose)

        return this
    }

    /**
     * Adds a strafe path segment.
     *
     * @param endPosition end position
     */
    fun strafeTo(endPosition: Vector2d): MultiTrajectoryBuilder{
        trajectoryBuilders.last().strafeTo(endPosition)

        return this
    }

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double): MultiTrajectoryBuilder{
        trajectoryBuilders.last().forward(distance)

        return this
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     */
    fun back(distance: Double): MultiTrajectoryBuilder{
        trajectoryBuilders.last().back(distance)

        return this
    }

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     */
    fun strafeLeft(distance: Double): MultiTrajectoryBuilder{
        trajectoryBuilders.last().strafeLeft(distance)

        return this
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     */
    fun strafeRight(distance: Double): MultiTrajectoryBuilder{
        trajectoryBuilders.last().strafeRight(distance)

        return this
    }

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineTo(endPosition: Vector2d, endTangent: Double): MultiTrajectoryBuilder {
        trajectoryBuilders.last().splineTo(endPosition, endTangent)

        return this
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     */
    fun splineToConstantHeading(endPosition: Vector2d, endTangent: Double): MultiTrajectoryBuilder {
        trajectoryBuilders.last().splineToConstantHeading(endPosition, endTangent)

        return this
    }

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToLinearHeading(endPose: Pose2d, endTangent: Double): MultiTrajectoryBuilder {
        trajectoryBuilders.last().splineToLinearHeading(endPose, endTangent)

        return this
    }

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param endTangent end tangent
     */
    fun splineToSplineHeading(endPose: Pose2d, endTangent: Double): MultiTrajectoryBuilder {
        trajectoryBuilders.last().splineToSplineHeading(endPose, endTangent)

        return this
    }

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun lineTo(endPosition: Vector2d, constraintsOverride: TrajectoryConstraints): MultiTrajectoryBuilder {
        trajectoryBuilders.last().lineTo(endPosition, constraintsOverride)

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToConstantHeading(endPosition: Vector2d, constraintsOverride: TrajectoryConstraints): MultiTrajectoryBuilder {
        trajectoryBuilders.last().lineToConstantHeading(endPosition, constraintsOverride)

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
    ): MultiTrajectoryBuilder {
        trajectoryBuilders.last().lineToLinearHeading(endPose, constraintsOverride)

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
    ): MultiTrajectoryBuilder {
        trajectoryBuilders.last().lineToSplineHeading(endPose, constraintsOverride)

        return this
    }

    /**
     * Adds a strafe path segment.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeTo(endPosition: Vector2d, constraintsOverride: TrajectoryConstraints): MultiTrajectoryBuilder {
        trajectoryBuilders.last().strafeTo(endPosition, constraintsOverride)

        return this
    }

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     * @param constraintsOverride segment-specific constraints
     */
    fun forward(distance: Double, constraintsOverride: TrajectoryConstraints): MultiTrajectoryBuilder {
        trajectoryBuilders.last().forward(distance, constraintsOverride)

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
    fun strafeLeft(distance: Double, constraintsOverride: TrajectoryConstraints): MultiTrajectoryBuilder {
        trajectoryBuilders.last().strafeLeft(distance, constraintsOverride)

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
    ): MultiTrajectoryBuilder {
        trajectoryBuilders.last().splineTo(endPosition, endTangent, constraintsOverride)

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
    ): MultiTrajectoryBuilder {
        trajectoryBuilders.last().splineToConstantHeading(endPosition, endTangent, constraintsOverride)

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
    ): MultiTrajectoryBuilder {
        trajectoryBuilders.last().splineToLinearHeading(endPose, endTangent, constraintsOverride)

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
    ): MultiTrajectoryBuilder {
        trajectoryBuilders.last().splineToSplineHeading(endPose, endHeading, constraintsOverride)

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
    fun addTemporalMarker(time: (Double) -> Double, callback: MarkerCallback): MultiTrajectoryBuilder{
        trajectoryBuilders.last().addTemporalMarker(time, callback)

        return this
    }

    /**
     * Adds a marker that will be triggered at the closest trajectory point to [point].
     */
    fun addSpatialMarker(point: Vector2d, callback: MarkerCallback): MultiTrajectoryBuilder {
        trajectoryBuilders.last().addSpatialMarker(point, callback)

        return this
    }

    /**
     * Adds a marker at the current position of the trajectory.
     */
    fun addDisplacementMarker(callback: MarkerCallback) =
        addDisplacementMarker(trajectoryBuilders.last().length(), callback)

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
    fun addDisplacementMarker(displacement: (Double) -> Double, callback: MarkerCallback): MultiTrajectoryBuilder {
        trajectoryBuilders.last().addDisplacementMarker(displacement, callback)
        return this
    }

    fun build(): List<Trajectory> {
        return trajectoryBuilders.map { trajectoryBuilder -> trajectoryBuilder.build() }
    }

    enum class Speed {
        SLOW, NORMAL
    }

    fun makeTrajectoryBuilder(tangent: Double): MultiTrajectoryBuilder {
        return makeTrajectoryBuilder(Speed.NORMAL, tangent)
    }

    fun makeTrajectoryBuilder(speed: Speed, tangent: Double): MultiTrajectoryBuilder {
        when (speed) {
            Speed.SLOW -> trajectoryBuilders.add(MirrorTrajectoryBuilder(
                alliance,
                getTrajectoryStartPose(),
                mirrorForAlliance(alliance, tangent),
                SLOW_CONSTRAINTS
            ))
            Speed.NORMAL -> trajectoryBuilders.add(MirrorTrajectoryBuilder(
                alliance,
                getTrajectoryStartPose(),
                mirrorForAlliance(alliance, tangent),
                BASE_CONSTRAINTS
            ))
        }

        return this
    }
    private fun getTrajectoryStartPose(): Pose2d {
        return if (trajectoryBuilders.isEmpty()) mirrorForAlliance(
            alliance,
            startPose
        ) else trajectoryBuilders.last().build().end()
    }

}
