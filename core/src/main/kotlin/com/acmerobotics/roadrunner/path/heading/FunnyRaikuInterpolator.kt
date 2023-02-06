package com.acmerobotics.roadrunner.path.heading

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.ParametricCurve
import com.acmerobotics.roadrunner.path.QuinticPolynomial
import com.acmerobotics.roadrunner.path.UnfunnyRaikuCurve
import com.acmerobotics.roadrunner.util.Angle

/**
 * Spline heading interpolator for transitioning smoothly between headings without violating continuity (and hence
 * allowing for integration into longer profiles).
 *
 * @param startHeading start heading
 * @param endHeading end heading
 */
// note: the spline parameter is transformed linearly into a pseudo-arclength parameter
class FunnyRaikuInterpolator constructor(
    private val startHeading: Double,
    private val angle: Double,
    p1: Double,
    p2: Double,
) : HeadingInterpolator() {
    private val headingSpline = UnfunnyRaikuCurve(0.0, p1, p2, 1.0)

    override fun internalGet(s: Double, t: Double) = Angle.norm(startHeading + headingSpline[s / curve.length()] * angle)

    override fun internalDeriv(s: Double, t: Double): Double {
        val len = curve.length()
        return headingSpline.deriv(s / len) * angle / len
    }

    override fun internalSecondDeriv(s: Double, t: Double): Double {
        val len = curve.length()
        return headingSpline.secondDeriv(s / len) * angle / (len * len)
    }
}
