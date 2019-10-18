package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.mechanics.DcMotorModel
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.mechanics.FixedWheelModel
import org.futurerobotics.jargon.mechanics.TransmissionModel
import java.util.*
import kotlin.math.abs
import kotlin.math.sign

/** Perturbs this [Double] with noise of with standard deviation [std] */
fun Double.perturbed(std: Double, random: Random = Random()): Double = this + random.nextGaussian() * std

/** Perturbs this [Vector2d] with noise of with standard deviation [std] */
fun Vector2d.perturbed(std: Double, random: Random = Random()): Vector2d =
    Vector2d(x.perturbed(std, random), y.perturbed(std, random))

/** Perturbs this [Pose2d] with noise of with standard deviation [std] */
fun Pose2d.perturbed(std: Double, random: Random = Random()): Pose2d =
    Pose2d(vec.perturbed(std, random), heading.perturbed(std, random))

/** Perturbs this [Vec] with noise of with standard deviation [std] */
fun Vec.perturbed(std: Double, random: Random = Random()): Vec = map {
    it.perturbed(std, random)
}

/** Perturbs this [Mat] with noise of with standard deviation [std] */
fun Mat.perturbed(std: Double, random: Random = Random()): Mat {
    return copy().apply {
        repeat(rows) { i ->
            repeat(cols) { j ->
                this[i, j] = this[i, j].perturbed(std, random)
            }
        }
    }
}

/**
 * Represents something that can perturb a model.
 */
interface Perturber<T> {
    /**
     * Perturbs the given [model] using the given [random] with white noise.
     */
    fun perturb(model: T, random: Random = Random()): T
}


/**
 * Perturbs a [DcMotorModel].
 * @param ktPerturb the std to perturb kt
 * @param rPerturb the std to perturb r
 * @param kvPerturb the std to perturb kv
 * @param i0Perturb the std to perturb i0
 */
class DcMotorModelPerturb(
    private val ktPerturb: Double,
    private val rPerturb: Double,
    private val kvPerturb: Double,
    private val i0Perturb: Double
) : Perturber<DcMotorModel> {
    constructor(std: Double) : this(std, std, std, std)

    override fun perturb(model: DcMotorModel, random: Random): DcMotorModel = model.let {
        DcMotorModel.fromCoefficients(
            it.kt.perturbed(ktPerturb, random).coerceAtLeast(1e-4),
            it.r.perturbed(rPerturb, random).coerceAtLeast(1e-4),
            it.kv.perturbed(kvPerturb, random).coerceAtLeast(1e-4),
            it.i0.perturbed(i0Perturb, random).coerceAtLeast(0.0)
        )
    }
}

/**
 * Perturbs a [TransmissionModel].
 * @param gearRatioStd the std to perturb the gearRatio
 * @param constantTorqueLossStd the std to perturb the constantTorqueLoss
 * @param ratioTorqueLossStd the std to perturb the ratio torque loss
 * @param motorPerturb how to perturb the motor mode.
 */
class TransmissionModelPerturb(
    private val gearRatioStd: Double,
    private val constantTorqueLossStd: Double,
    private val ratioTorqueLossStd: Double,
    private val motorPerturb: DcMotorModelPerturb
) : Perturber<TransmissionModel> {
    constructor(std: Double, motor: DcMotorModelPerturb) : this(std, std, std, motor)

    override fun perturb(model: TransmissionModel, random: Random): TransmissionModel = model.let {
        TransmissionModel.fromTorqueLosses(
            motorPerturb.perturb(it.motor, random),
            it.gearRatio.perturbed(gearRatioStd, random).let { x ->
                abs(x).coerceAtLeast(0.0) * sign(it.gearRatio)
            },
            it.constantTorqueLoss.perturbed(constantTorqueLossStd, random).coerceAtLeast(0.0),
            it.ratioTorqueLoss.perturbed(ratioTorqueLossStd, random).coerceIn(0.0, 1.0)
        )
    }
}

/**
 * Perturbs a [FixedWheelModel].
 * @param positionStd the std to perturb the position
 * @param radiusStd the std to perturb the radius
 * @param angleStd the std to perturb the wheel angle
 * @param transmissionPerturb how to perturb the transmission
 */
class FixedWheelModelPerturb(
    private val positionStd: Double,
    private val radiusStd: Double,
    private val angleStd: Double,
    private val transmissionPerturb: TransmissionModelPerturb
) : Perturber<FixedWheelModel> {
    constructor(std: Double, transmission: TransmissionModelPerturb) : this(std, std, std, transmission)

    override fun perturb(model: FixedWheelModel, random: Random): FixedWheelModel = model.let {
        FixedWheelModel.fromWheelAngle(
            transmissionPerturb.perturb(it.transmission, random),
            it.position.perturbed(positionStd, random),
            it.radius.perturbed(radiusStd, random),
            it.orientation.angle.perturbed(angleStd, random)
        )
    }
}

/**
 * Perturbs a [DcMotorModel].
 * @param massStd the std to perturb the mass
 * @param moiStd the std to perturb the moi
 * @param wheelPerturb how to perturb the wheels
 */
class FixedDriveModelPerturber(
    private val massStd: Double,
    private val moiStd: Double,
    private val wheelPerturb: FixedWheelModelPerturb
) : Perturber<FixedDriveModel> {
    constructor(std: Double, wheels: FixedWheelModelPerturb) : this(std, std, wheels)

    override fun perturb(model: FixedDriveModel, random: Random): FixedDriveModel = model.let {
        FixedDriveModel(
            it.mass.perturbed(massStd, random),
            it.moi.perturbed(moiStd, random),
            it.wheels.map { w -> wheelPerturb.perturb(w, random) },
            it.isHolonomic
        )
    }
}
