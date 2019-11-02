package org.futurerobotics.jargon.learning

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.mechanics.MotorVelocityModel
import org.futurerobotics.jargon.statespace.DiscreteLinearStateSpaceModel
import org.futurerobotics.jargon.util.asUnmodifiableList
import java.io.Serializable

/**
 * A discrete and online-updatable variant of something like a [MotorVelocityModel].
 *
 * This is a model in the form of `x_(t+1) = A*x + B*u + F*sign(x)`,
 *
 * - Measurement directly corresponds with state.
 * - x is a motor velocity vector
 * - u is motor voltage signal
 * - A is system matrix
 * - B is control matrix
 * - F is the contribution of friction (can be addressed later via feed-forward).
 *
 *
 * This is also a [MultipleMatrixPredictor] with matrices (A,B,F), so it can be updated with a [StochasticUpdatingBlock]
 * online. We don't say the ML-word.
 *
 * This is also a [DiscreteLinearStateSpaceModel] without the F term; A feed forward of `B^-1*F*sign(x)`
 * can be added to compensate to friction according to this model.
 *
 * This is also [Serializable] so that the model can be stored as a file.
 *
 */
class UpdatableMotorVelocityModel private constructor(
    override val A: Mat,
    override val B: Mat,
    val F: Mat,
    override val period: Double
) : DiscreteLinearStateSpaceModel, MultipleMatrixPredictor, Serializable {

    override val mats: List<Mat> = listOf(A, B, F).asUnmodifiableList()

    override val ySize: Int = mats.fold(-1) { acc, cur ->
        if (acc == -1)
            cur.rows
        else {
            require(cur.rows == acc) { "All matrices must have same # of rows" }
            acc
        }
    }
    override val numInputs: Int get() = mats.size
    override val xSizes: List<Int> = mats.map { it.cols }.asUnmodifiableList()

    override fun predict(input: List<Vec>): Vec = mats.zip(input, Mat::times).reduce(Vec::add)

    override val stateSize: Int
        get() = ySize
    override val inputSize: Int
        get() = ySize
    override val outputSize: Int
        get() = ySize

    override fun processState(x: Vec, u: Vec): Vec = A * x + B * u

    override fun processOutput(x: Vec, u: Vec): Vec = x.copy()

    fun getFeedForward(x: Vec): Vec = B.inv() * F * sign(x)

    override val C: Mat = idenMat(ySize)
    override val D: Mat = zeroMat(ySize, ySize)

    companion object {
        private const val serialVersionUID: Long = -88231094383843
    }
}
