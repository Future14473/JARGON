package org.futurerobotics.temporaryname.pathing.path.reparam

import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.function.VectorFunction
import org.futurerobotics.temporaryname.math.squared
import org.futurerobotics.temporaryname.math.zcross
import org.futurerobotics.temporaryname.pathing.path.Curve
import org.futurerobotics.temporaryname.pathing.path.CurvePointInfo

/**
 * A [Curve] that works by reparameterizing an arbitrary C2 continuous [VectorFunction] ([func]) and a [ReparamMapping] ([mapping]) that maps
 * arc length to the original function parameter.
 */
class ReparamCurve(private val func: VectorFunction, internal val mapping: ReparamMapping) : Curve {

    /** Gets the t parameter on the original function that corresponds to an arc length of [s] */
    fun tOfS(s: Double): Double = mapping.tOfS(s)

    /** The number of samples of this ReparamCurve's [ReparamMapping] */
    val numSamples: Int get() = mapping.numSamples

    override val length: Double get() = mapping.length
//
//    override fun pose(s: Double) = curve.pose(tOfS(s))
//    override fun poseDeriv(s: Double) = curve.poseDeriv(tOfS(s)).normalizedByVec()
//    override fun poseSecondDeriv(s: Double): Pose2d {
//        val t = tOfS(s)
//        val poseDeriv = curve.poseDeriv(t)
//        val poseSecondDeriv = curve.poseSecondDeriv(t)
//        val vecDerivLen2 = poseDeriv.vec.lengthSquared
//        return (poseSecondDeriv - poseDeriv * ((poseDeriv vecDot poseSecondDeriv )/ vecDerivLen2)) / vecDerivLen2
//    }

    override fun position(s: Double): Vector2d = func.vec(tOfS(s))
    override fun positionDeriv(s: Double): Vector2d = func.vecDeriv(tOfS(s)).normalized()
    override fun positionSecondDeriv(s: Double): Vector2d {
        val t = tOfS(s)
        val v = func.vecDeriv(t)
        return (v cross func.vecSecondDeriv(t)) / v.lengthSquared.squared() zcross v //= tanAngleDeriv zcross positionDeriv
        //minimal allocations. HURRY UP PROJECT VALHALLA
    }

    override fun tanAngle(s: Double): Double = func.vecDeriv(tOfS(s)).angle
    override fun tanAngleDeriv(s: Double): Double = func.curvature(tOfS(s))
    override fun tanAngleSecondDeriv(s: Double): Double {
        val t = tOfS(s)
        return func.curvatureDeriv(t) / func.vecDeriv(t).length
    }

    override fun getPointInfo(s: Double): CurvePointInfo {
        val t = tOfS(s)
        return ReparamCurvePointInfo(func.vec(t), func.vecDeriv(t), func.vecSecondDeriv(t), func.vecThirdDeriv(t))
    }

    override fun getAllPointInfo(allS: List<Double>): List<CurvePointInfo> {
        return mapping.getAllTOfS(allS).map { t ->
            ReparamCurvePointInfo(func.vec(t), func.vecDeriv(t), func.vecSecondDeriv(t), func.vecThirdDeriv(t))
        }
    }

    /** Slight optimizations. */
    private class ReparamCurvePointInfo(
        private val p: Vector2d, private val v: Vector2d, private val a: Vector2d, private val j: Vector2d
    ) : CurvePointInfo {
        override val position: Vector2d get() = p
        override val positionDeriv: Vector2d get() = v.normalized()
        override val positionSecondDeriv: Vector2d
            get() = tanAngleDeriv zcross positionDeriv
        override val tanAngle: Double
            get() = v.angle
        override val tanAngleDeriv: Double = v cross a / v.lengthPow(3.0)
        override val tanAngleSecondDeriv: Double
            get() = (v cross j) / v.lengthSquared.squared() - 3 * tanAngleDeriv * (v dot a) / v.lengthPow(3.0)
    }
}
