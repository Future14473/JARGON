package org.futurerobotics.jargon.learning

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.util.asUnmodifiableList
import org.futurerobotics.jargon.util.toImmutableList
import org.futurerobotics.jargon.util.zipForEach
import org.futurerobotics.jargon.util.zipForEachIndexed
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.pow

/**
 * A model, which tries to predict some output of multiple variables `y`, which is the sum of several matrices times
 * several input vectors, `x_i`. Each matrix can then have different parameters when it comes to learning.
 *
 * The matrices are allowed to be modifiable, but not the sizes.
 */
interface MultipleMatrixPredictor : Predictor<List<Vec>, Vec> {

    /** The size of the output vector. */
    val ySize: Int
    /** The number of input vectors. */
    val numInputs: Int get() = mats.size
    /** The size of the input vectors. */
    val xSizes: List<Int>
    /** Gets all the matrices used to transform the input vectors into the output vectors. */
    val mats: List<Mat>

    /** Gets the predicted output vector given multiple [input] vectors. */
    override fun predict(input: List<Vec>): Vec
}

/**
 * A base implementation of a [MultipleMatrixPredictor], which just takes compatible matrices.
 */
open class BaseMultipleMatrixPredictor(mats: List<Mat>) : MultipleMatrixPredictor {

    final override val ySize: Int

    init {
        require(mats.isNotEmpty()) { "Must have at least one matrix" }
        ySize = mats.fold(-1) { acc, cur ->
            if (acc == -1)
                cur.rows
            else {
                require(cur.rows == acc) { "All matrices must have same # of rows" }
                acc
            }
        }
    }

    override val numInputs: Int get() = mats.size
    override val xSizes: List<Int> = mats.map { it.cols }.asUnmodifiableList()
    override val mats: List<Mat> = mats.map { it.copy() }.asUnmodifiableList()

    override fun predict(input: List<Vec>): Vec = mats.zip(input, Mat::times).reduce(Vec::add)
}

/**
 * Parameters for 'gradient' descent for fitting [MultipleMatrixPredictor]s.
 */
class MultipleDescentParams(
    learningRates: List<Double>,
    regularization: List<Double>
) {

    /** The learning rate for each of the matrices. */
    val learningRates: List<Double> = learningRates.toImmutableList().also {
        require(it.all { n -> n > 0 }) { "All learning rates must be not zero" }
    }
    /** The regularization parameter for each of the matrices. */
    val regularization: List<Double> = regularization.toImmutableList().also {
        require(it.all { n -> n > 0 }) { "All regularization parameters must be not zero" }
    }
}

/**
 * Base implementation of a [MultipleMatrixPredictor] fitter.
 *
 * @param params the [MultipleDescentParams]
 */
abstract class AbstractMultipleMatrixFitter(protected val params: MultipleDescentParams) :
    Fitter<List<Vec>, Vec, MultipleMatrixPredictor> {

    override fun cost(predictor: MultipleMatrixPredictor, inputs: List<List<Vec>>, outputs: List<Vec>): Double {
        require(inputs.size == outputs.size) { "Inputs and output lists must be same size" }
        return predictor.predict(inputs).zip(outputs) { pred, real ->
            (pred - real).norm.pow(2)
        }.average()
    }

    /**
     * Gets the gradient of one of the matrixes with respect to cost (including regularization), given:
     *
     * - the [error], y - yPred
     * - [xi], the corresponding input vector
     * - [mat], the original matrix
     * - [lambda], the regularization
     */
    protected fun getGradient(
        error: Vec,
        xi: Vec,
        mat: Mat,
        lambda: Double
    ): Mat {
        return genMat(error.size, xi.size) { r, c ->
            (-error[r] * xi[c] + mat[r, c] * lambda)
        }
    }
}

/**
 * A [Fitter] for a [MultipleMatrixPredictor] that uses batch gradient descent
 */
class BatchMultipleMatrixFitter(
    private val batchSize: Int, params: MultipleDescentParams, private val random: Random = Random()
) : AbstractMultipleMatrixFitter(params) {

    override fun fitOnce(predictor: MultipleMatrixPredictor, inputs: List<List<Vec>>, outputs: List<Vec>) {
        require(inputs.size == outputs.size) { "Inputs and output lists must be same size" }
        val mats = predictor.mats
        inputs.zip(outputs).shuffled(random).windowed(batchSize, batchSize, true).forEach {
            //batch
            val descents = mats.mapTo(ArrayList(mats.size)) { m -> zeroMat(m.rows, m.cols) }
            it.forEach { (x, y) ->
                //item in the batch
                val yPred = predictor.predict(x)
                val error = y - yPred
                descents.zipForEachIndexed(x) { matI, desc, xi ->
                    val mat = mats[matI]

                    val grad = getGradient(error, xi, mat, params.regularization[matI]) / it.size.toDouble()
                    desc -= grad * params.learningRates[matI]
                }
            }
            mats.zipForEach(descents) { mat, desc ->
                mat += desc
            }
        }
    }
}

/**
 * A [Fitter] for a [MultipleMatrixPredictor] that uses batch gradient descent
 */
class StochasticMultipleMatrixFitter(
    params: MultipleDescentParams, private val random: Random = Random()
) : AbstractMultipleMatrixFitter(params),
    StochasticFitter<List<Vec>, Vec, MultipleMatrixPredictor> {

    override fun stochasticUpdate(predictor: MultipleMatrixPredictor, input: List<Vec>, output: Vec) {
        val mats = predictor.mats
        val yPred = predictor.predict(input)
        val error = output - yPred
        input.forEachIndexed { matI, xi ->
            val mat = mats[matI]

            val grad = getGradient(error, xi, mat, params.regularization[matI])
            mat -= grad * params.learningRates[matI]
        }
    }

    override fun fitOnce(predictor: MultipleMatrixPredictor, inputs: List<List<Vec>>, outputs: List<Vec>) {
        require(inputs.size == outputs.size) { "Inputs and output lists must be same size" }
        inputs.zip(outputs).shuffled(random).forEach { (x, y) ->
            stochasticUpdate(predictor, x, y)
        }
    }
}
