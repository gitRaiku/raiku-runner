package com.acmerobotics.roadrunner.path

/**
 *
 *
 * @param c0
 * @param c1
 * @param c2
 * @param c3
 */
@Suppress("MemberVisibilityCanBePrivate")
class CubicBezierCurve(
    val c0: Double,
    val c1: Double,
    val c2: Double,
    val c3: Double
) {
    /**
     * Returns the value of the polynomial at [t].
     */
    operator fun get(t: Double) =
        t * (t * (t * (-c0 + 3 * c1 - 3 * c2 + c3) + 3 * c0 - 6 * c1 + 3 * c2) - 3 * c0 + 3 * c1) + c0

    /**
     * Returns the derivative of the polynomial at [t].
     */
    fun deriv(t: Double) =
        t * (t * (-3 * c0 - 9 * c2 + 3 * c3) + 6 * c0 + 6 * c2) - 3 * c0 + c1 * (t * (9 * t - 12) + 3)

    /**
     * Returns the second derivative of the polynomial at [t].
     */
    fun secondDeriv(t: Double) = -6 * (c0 * (t - 1) - c1 * (3 * t - 2) + c2 * (3 * t - 1) - c3 * t)

    /**
     * Returns the third derivative of the polynomial at [t].
     */
    fun thirdDeriv(t: Double) = -6 * c0 + 18 * c1 - 18 * c2 + 6 * c3 + 0 * t

    override fun toString() = String.format("%.5f*t^5+%.5f*t^4+%.5f*t^3+%.5f*t^2+%.5f*t+%.5f", c0, c1, c2, c3)
}
