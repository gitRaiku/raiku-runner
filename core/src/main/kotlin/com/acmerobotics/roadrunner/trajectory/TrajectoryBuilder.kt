package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI

private fun MotionState.zeroPosition() = MotionState(0.0, v, a, j)

private data class IntervalVelocityConstraint(
    val start: Double,
    val end: Double,
    val constraint: TrajectoryVelocityConstraint
)

private class PiecewiseVelocityConstraint(
    private val baseConstraint: TrajectoryVelocityConstraint,
    private val constraintOverrides: List<IntervalVelocityConstraint>
) : TrajectoryVelocityConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseRobotVel: Pose2d): Double {
        for (interval in constraintOverrides) {
            val (start, end, constraint) = interval
            if (s in start..end) {
                return constraint[s, pose, deriv, baseRobotVel]
            }
        }
        return baseConstraint[s, pose, deriv, baseRobotVel]
    }
}

private data class IntervalAccelerationConstraint(
    val start: Double,
    val end: Double,
    val constraint: TrajectoryAccelerationConstraint
)

private class PiecewiseAccelerationConstraint(
    private val baseConstraint: TrajectoryAccelerationConstraint,
    private val constraintOverrides: List<IntervalAccelerationConstraint>
) : TrajectoryAccelerationConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseRobotVel: Pose2d): Double {
        for (interval in constraintOverrides) {
            val (start, end, constraint) = interval
            if (s in start..end) {
                return constraint[s, pose, deriv, baseRobotVel]
            }
        }
        return baseConstraint[s, pose, deriv, baseRobotVel]
    }
}

/**
 * Builder for trajectories with *dynamic* constraints.
 */
@Suppress("unused", "KDocUnresolvedReference")
class TrajectoryBuilder private constructor(
    startPose: Pose2d?,
    startTangent: Double?,
    trajectory: Trajectory?,
    t: Double?,
    private val baseVelConstraint: TrajectoryVelocityConstraint,
    private val baseAccelConstraint: TrajectoryAccelerationConstraint,
    private val baseDecelConstraint: TrajectoryAccelerationConstraint,
    private val start: MotionState,
    private val resolution: Double
) : BaseTrajectoryBuilder<TrajectoryBuilder>(startPose, startTangent, trajectory, t) {
    /**
     * Create a builder from a start pose and motion state. This is the recommended constructor for creating
     * trajectories from rest.
     */
    @JvmOverloads
    constructor(
        startPose: Pose2d,
        startTangent: Double = startPose.heading,
        baseVelConstraint: TrajectoryVelocityConstraint,
        baseAccelConstraint: TrajectoryAccelerationConstraint,
        baseDecelConstraint: TrajectoryAccelerationConstraint,
        resolution: Double = 0.25
    ) : this(
        startPose,
        startTangent,
        null,
        null,
        baseVelConstraint,
        baseAccelConstraint,
        baseDecelConstraint,
        MotionState(0.0, 0.0, 0.0),
        resolution
    )

    @JvmOverloads
    constructor(
        startPose: Pose2d,
        reversed: Boolean,
        baseVelConstraint: TrajectoryVelocityConstraint,
        baseAccelConstraint: TrajectoryAccelerationConstraint,
        baseDecelConstraint: TrajectoryAccelerationConstraint,
        resolution: Double = 0.25
    ) : this(
        startPose,
        Angle.norm(startPose.heading + if (reversed) PI else 0.0),
        baseVelConstraint,
        baseAccelConstraint,
        baseDecelConstraint,
        resolution
    )

    /**
     * Create a builder from an active trajectory. This is useful for interrupting a live trajectory and smoothly
     * transitioning to a new one.
     */
    @JvmOverloads
    constructor(
        trajectory: Trajectory,
        t: Double,
        baseVelConstraint: TrajectoryVelocityConstraint,
        baseAccelConstraint: TrajectoryAccelerationConstraint,
        baseDecelConstraint: TrajectoryAccelerationConstraint,
        resolution: Double = 0.25
    ) : this(
        null,
        null,
        trajectory,
        t,
        baseVelConstraint,
        baseAccelConstraint,
        baseDecelConstraint,
        trajectory.profile[t].zeroPosition(),
        resolution
    )

    private val velConstraintOverrides = mutableListOf<IntervalVelocityConstraint>()
    private val accelConstraintOverrides = mutableListOf<IntervalAccelerationConstraint>()
    private val decelConstraintOverrides = mutableListOf<IntervalAccelerationConstraint>()

    private fun addSegment(
        add: () -> Unit,
        velConstraintOverride: TrajectoryVelocityConstraint?=null,
        accelConstraintOverride: TrajectoryAccelerationConstraint?=null,
        decelConstraintOverride: TrajectoryAccelerationConstraint?=null
    ): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        add()

        val end = pathBuilder.build().length()

        if (velConstraintOverride != null) {
            velConstraintOverrides.add(IntervalVelocityConstraint(start, end, velConstraintOverride))
        }

        if (accelConstraintOverride != null) {
            accelConstraintOverrides.add(IntervalAccelerationConstraint(start, end, accelConstraintOverride))
        }

        if (decelConstraintOverride != null) {
            decelConstraintOverrides.add(IntervalAccelerationConstraint(start, end, decelConstraintOverride))
        }

        return this
    }

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun lineTo(
        endPosition: Vector2d,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ lineTo(endPosition) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToConstantHeading(
        endPosition: Vector2d,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ lineToConstantHeading(endPosition) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToLinearHeading(
        endPose: Pose2d,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ lineToLinearHeading(endPose) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToSplineHeading(
        endPose: Pose2d,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ lineToSplineHeading(endPose) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * Adds a strafe path segment.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeTo(
        endPosition: Vector2d,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ strafeTo(endPosition) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     * @param constraintsOverride segment-specific constraints
     */
    fun forward(
        distance: Double,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ forward(distance) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     * @param constraintsOverride segment-specific constraints
     */
    fun back(
        distance: Double,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ back(distance) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeLeft(
        distance: Double,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ strafeLeft(distance) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeRight(
        distance: Double,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ strafeRight(distance) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

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
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ splineTo(endPosition, endTangent) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * TODO: Get old comments
     *
     */
    fun funnyRaikuCurve(
        endPose: Pose2d,
        p1: Vector2d,
        p2: Vector2d,
        h1: Double,
        h2: Double,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ funnyRaikuCurve(endPose, p1, p2, h1, h2) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    fun funnyRaikuCurveLinear(
        endPose: Pose2d,
        p1: Vector2d,
        p2: Vector2d,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ funnyRaikuCurveLinear(endPose, p1, p2) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)


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
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ splineToConstantHeading(endPosition, endTangent) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

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
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ splineToLinearHeading(endPose, endTangent) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun splineToSplineHeading(
        endPose: Pose2d,
        endTangent: Double,
        velConstraintOverride: TrajectoryVelocityConstraint?,
        accelConstraintOverride: TrajectoryAccelerationConstraint?,
        decelConstraintOverride: TrajectoryAccelerationConstraint?
    ) =
        addSegment({ splineToSplineHeading(endPose, endTangent) }, velConstraintOverride, accelConstraintOverride, decelConstraintOverride)

    override fun buildTrajectory(
        path: Path,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): Trajectory {
        val goal = MotionState(path.length(), 0.0, 0.0)
        return TrajectoryGenerator.generateTrajectory(
            path,
            PiecewiseVelocityConstraint(baseVelConstraint, velConstraintOverrides),
            PiecewiseAccelerationConstraint(baseAccelConstraint, accelConstraintOverrides),
            PiecewiseAccelerationConstraint(baseDecelConstraint, decelConstraintOverrides),
            start,
            goal,
            temporalMarkers,
            displacementMarkers,
            spatialMarkers,
            resolution
        )
    }
}
