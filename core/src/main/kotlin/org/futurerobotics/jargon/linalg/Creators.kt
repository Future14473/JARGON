@file:JvmName("MatCreators")

package org.futurerobotics.jargon.linalg

import org.hipparchus.linear.Array2DRowRealMatrix
import org.hipparchus.linear.ArrayRealVector
import org.hipparchus.linear.BlockRealMatrix
import org.hipparchus.linear.MatrixUtils
import java.util.*

/** Creates a matrix with the given [rows] and [cols], and filling values with the given [init]. */
inline fun genMat(rows: Int, cols: Int, init: (r: Int, c: Int) -> Double): Mat = zeroMat(rows, cols).apply {
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = init(r, c)
        }
    }
}

/** Creates a [Mat] using a 2d double array. */
@JvmOverloads
fun createMat(data: Array<DoubleArray>, copy: Boolean = true): Mat =
    if (data.size * data[0].size <= 4096)
        Array2DRowRealMatrix(data, copy)
    else
        BlockRealMatrix(data)

/** Creates a matrix filled with zeros with the given [rows] and [cols]. */
fun zeroMat(rows: Int, cols: Int): Mat = MatrixUtils.createRealMatrix(rows, cols)

/** Creates an identity matrix with the given [size]. */
fun idenMat(size: Int): Mat = MatrixUtils.createRealIdentityMatrix(size)

/** Creates a matrix with the given [values] along the diagonal. */
fun diagMat(values: DoubleArray): Mat = MatrixUtils.createRealDiagonalMatrix(values)

/** Creates a matrix with the given [values] along the diagonal. */
fun diagMat(values: List<Double>): Mat = MatrixUtils.createRealDiagonalMatrix(values.toDoubleArray())

/** Creates a matrix with the given [values] along the diagonal. */
@JvmName("diagVararg")
fun diagMat(vararg values: Double): Mat = MatrixUtils.createRealDiagonalMatrix(values)

/** Creates a matrix with the given [size], and filling values with the given [init]. */
inline fun genVec(size: Int, init: (Int) -> Double): Vec = zeroVec(size).apply {
    repeat(size) {
        this[it] = init(it)
    }
}

/** Creates a vector with the given [values], and copying the array if [copy] is true. */
@JvmOverloads
fun createVec(values: DoubleArray, copy: Boolean = true): Vec = ArrayRealVector(values, copy)

/** Creates a vector with the given [values]. */
fun createVec(values: List<Double>): Vec = ArrayRealVector(values.toDoubleArray(), false)

/** Creates a vector with the given [values]. */
@JvmName("createVecVararg")
fun createVec(vararg values: Double): Vec = ArrayRealVector(values, false)

/** Creates a vector filled with zeros with the given [size]. */
fun zeroVec(size: Int): Vec = ArrayRealVector(size)

/**
 * Creates a vector full of Gaussian random samples with the given [size], and using the given [random]
 */
@JvmOverloads
fun normRandVec(size: Int, random: Random = Random()): Vec = zeroVec(size).apply {
    repeat(size) {
        this[it] = random.nextGaussian()
    }
}

/**
 * Creates a vector full of Gaussian random samples with the given [size], and using the given [random]
 */
@JvmOverloads
fun normRandMat(rows: Int, cols: Int, random: Random = Random()): Mat = zeroMat(rows, cols).apply {
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = random.nextGaussian()
        }
    }
}

/** Creates a vector using the given [values]. Used for vector literals. */
@Suppress("FunctionName")
@JvmSynthetic
fun Vec(vararg values: Number): Vec {
    val doubles = DoubleArray(values.size) { values[it].toDouble() }
    return ArrayRealVector(doubles, false)
}

/**
 * Creates a matrix using the given number values, as a matrix literals.
 *
 * Number values should be separated by commas, and rows separated using the
 * infix function [to].
 *
 * For example:
 * ```kotlin
 * Mat(
 *  1, 2, 3 to
 *  4, 5, 6 to
 *  6, 7, 0
 * )
 * ```
 *
 * In java, use [createMat] instead with a 2d array literal.
 */
@Suppress("FunctionName")
@JvmSynthetic
fun Mat(vararg values: Any): Mat {
    lateinit var mat: Mat
    varargEndToArr<Number>(
        values,
        { r, c -> mat = zeroMat(r, c) },
        { r, c, e -> mat[r, c] = e.toDouble() }
    )
    return mat
}
