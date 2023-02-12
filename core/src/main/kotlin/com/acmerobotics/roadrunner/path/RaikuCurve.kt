package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Vector2d
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * @param start start waypoint
 * @param end end waypoint
 * @param c1 first control point in polar coordinates
 * @param c2 second control point in polar coordinates
 */
class RaikuCurve(
    start: Vector2d,
    end: Vector2d,
    c1: Vector2d,
    c2: Vector2d
) : ParametricCurve() {
    val x: CubicBezierCurve = CubicBezierCurve(start.x, c1.x, c2.x, end.x)
    val y: CubicBezierCurve = CubicBezierCurve(start.y, c1.y, c2.y, end.y)

    private var length: Double = 0.0

    private val ss = mutableListOf(0.0)
    private val ts = mutableListOf(0.0)

    companion object {
        const val MAXD = 10
        const val MIND = 8
        const val MACC = 0.95
    }

    private fun parameterize(p1: Double, p3: Double, depth: Int = 0) {
        val p2 = (p1 + p3) / 2
        val v1 = internalGet(p1)
        val v2 = internalGet(p2)
        val v3 = internalGet(p3)
        val v12 = v1 distTo v2
        val v23 = v2 distTo v3
        val v13 = v1 distTo v3
        if (depth > MAXD || ((v13 / (v12 + v23) < MACC) && depth >= MIND)) {
            length += v12 + v23
            ss.add(length)
            ts.add(p3)
        } else {
            parameterize(p1, p2, depth + 1)
            parameterize(p2, p3, depth + 1)
        }
    }


    init {
        parameterize(0.0, 1.0)
    }

    override fun internalGet(t: Double) = Vector2d(x[t], y[t])

    override fun internalDeriv(t: Double) = Vector2d(x.deriv(t), y.deriv(t))

    override fun internalSecondDeriv(t: Double) =
        Vector2d(x.secondDeriv(t), y.secondDeriv(t))

    override fun internalThirdDeriv(t: Double) =
        Vector2d(x.thirdDeriv(t), y.thirdDeriv(t))

    private fun lerp(t: Double, p1: Double, p2: Double): Double {
        return p1 + (p2 - p1) * t
    }

    override fun reparam(s: Double): Double {
        if (s <= 0.0) {
            return 0.0
        } else if (s >= length) {
            return 1.0
        } else {
            var l = 0
            var h = ss.size

            while (l <= h) {
                val m = (l + h) / 2
                if (s < ss[m]) {
                    h = m - 1
                } else {
                    if (s <= ss[m]) {
                        return ts[m]
                    }
                    l = m + 1
                }
            }

            return lerp((s - ss[l]) / (ss[h] - ss[l]), ts[l], ts[h])
        }
    }


    override fun paramDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        return 1.0 / sqrt(deriv.x * deriv.x + deriv.y * deriv.y)
    }

    override fun paramSecondDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val numerator = -(deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return numerator / (denominator * denominator)
    }

    override fun paramThirdDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val thirdDeriv = internalThirdDeriv(t)

        val firstNumeratorSqrt = 2.0 * (deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val secondNumerator = secondDeriv.x * secondDeriv.x + secondDeriv.y * secondDeriv.y +
                deriv.x * thirdDeriv.x + deriv.y * thirdDeriv.y

        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return firstNumeratorSqrt * firstNumeratorSqrt / denominator.pow(3.5) -
                secondNumerator / denominator.pow(2.5)
    }

    override fun length() = length

    override fun toString() = "($x,$y)"
}
