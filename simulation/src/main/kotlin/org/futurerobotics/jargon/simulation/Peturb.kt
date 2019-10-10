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
fun Double.perturb(std: Double, random: Random = Random()): Double {
    return this + random.nextGaussian() * std
}

/** Perturbs this [Vector2d] with noise of with standard deviation [std] */
fun Vector2d.perturb(std: Double, random: Random = Random()): Vector2d {
    return Vector2d(x.perturb(std, random), y.perturb(std, random))
}

/** Perturbs this [Pose2d] with noise of with standard deviation [std] */
fun Pose2d.perturb(std: Double, random: Random = Random()): Pose2d {
    return Pose2d(vec.perturb(std, random), heading.perturb(std, random))
}

/** Perturbs this [Vec] with noise of with standard deviation [std] */
fun Vec.perturb(std: Double, random: Random = Random()): Vec = map {
    it.perturb(std, random)
}

/** Perturbs this [Mat] with noise of with standard deviation [std] */
fun Mat.perturb(std: Double, random: Random = Random()): Mat {
    return copy().apply {
        repeat(rows) { i ->
            repeat(cols) { j ->
                this[i, j] = this[i, j].perturb(std, random)
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
 * @param kt the std to perturb kt
 * @param r the std to perturb r
 * @param kv the std to perturb kv
 * @param i0 the std to perturb i0
 */
class DcMotorModelPerturb(
    private val kt: Double,
    private val r: Double,
    private val kv: Double,
    private val i0: Double
) : Perturber<DcMotorModel> {
    constructor(std: Double) : this(std, std, std, std)

    override fun perturb(model: DcMotorModel, random: Random): DcMotorModel = model.let {
        DcMotorModel.fromCoefficients(
            it.kt.perturb(kt, random).coerceAtLeast(1e-4),
            it.r.perturb(r, random).coerceAtLeast(1e-4),
            it.kv.perturb(kv, random).coerceAtLeast(1e-4),
            it.i0.perturb(i0, random).coerceAtLeast(0.0)
        )
    }
}

/**
 * Perturbs a [TransmissionModel].
 * @param gearRatio the std to perturb the gearRatio
 * @param constantTorqueLoss the std to perturb the constantTorqueLoss
 * @param ratioTorqueLoss the std to perturb the ratio torque loss
 * @param motor how to perturb the motor mode.
 */
class TransmissionModelPerturb(
    private val gearRatio: Double,
    private val constantTorqueLoss: Double,
    private val ratioTorqueLoss: Double,
    private val motor: DcMotorModelPerturb
) : Perturber<TransmissionModel> {
    constructor(std: Double, motor: DcMotorModelPerturb) : this(std, std, std, motor)

    override fun perturb(model: TransmissionModel, random: Random): TransmissionModel = model.let {
        TransmissionModel.fromTorqueLosses(
            motor.perturb(it.motor, random),
            it.gearRatio.perturb(gearRatio, random).let { x ->
                abs(x).coerceAtLeast(0.0) * sign(it.gearRatio)
            },
            it.constantTorqueLoss.perturb(constantTorqueLoss, random).coerceAtLeast(0.0),
            ratioTorqueLoss.perturb(ratioTorqueLoss, random).coerceIn(0.0, 1.0)
        )
    }
}

/**
 * Perturbs a [FixedWheelModel].
 * @param position the std to perturb the position
 * @param radius the std to perturb the radius
 * @param angle the std to perturb the wheel angle
 * @param transmission how to perturb the transmission
 */
class FixedWheelModelPerturb(
    private val position: Double,
    private val radius: Double,
    private val angle: Double,
    private val transmission: TransmissionModelPerturb
) : Perturber<FixedWheelModel> {
    constructor(std: Double, transmission: TransmissionModelPerturb) : this(std, std, std, transmission)

    override fun perturb(model: FixedWheelModel, random: Random): FixedWheelModel = model.let {
        FixedWheelModel.fromWheelAngle(
            transmission.perturb(it.transmission, random),
            it.position.perturb(position, random),
            it.radius.perturb(radius, random).coerceAtLeast(1e-4),
            it.orientation.angle.perturb(angle, random)
        )
    }
}

/**
 * Perturbs a [DcMotorModel].
 * @param mass the std to perturb the mass
 * @param moi the std to perturb the moi
 * @param wheels how to perturb the wheels
 */
class FixedDriveModelPerturber(
    private val mass: Double,
    private val moi: Double,
    private val wheels: FixedWheelModelPerturb
) : Perturber<FixedDriveModel> {
    constructor(std: Double, wheels: FixedWheelModelPerturb) : this(std, std, wheels)

    override fun perturb(model: FixedDriveModel, random: Random): FixedDriveModel = model.let {
        FixedDriveModel(
            it.mass.perturb(mass, random),
            it.moi.perturb(moi, random),
            it.wheels.map { w -> wheels.perturb(w, random) },
            it.isHolonomic
        )
    }
}
