@file:JvmName("MatCreators")

package org.futurerobotics.jargon.linalg

import org.hipparchus.linear.Array2DRowRealMatrix
import org.hipparchus.linear.ArrayRealVector
import org.hipparchus.linear.BlockRealMatrix
import org.hipparchus.linear.MatrixUtils
import java.util.*

/** Creates a [Mat] using a 2d double array. */
@JvmOverloads
fun createMat(data: Array<DoubleArray>, copy: Boolean = true): Mat =
    if (data.size * data[0].size <= 4096)
        Array2DRowRealMatrix(data, copy)
    else
        BlockRealMatrix(data)

/** Creates a matrix with the given [rows] and [cols], and filling values with the given [func]. */
inline fun createMat(rows: Int, cols: Int, func: (r: Int, c: Int) -> Double): Mat = zeroMat(rows, cols).apply {
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = func(r, c)
        }
    }
}

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

/** Creates a vec, using the given [values]. Used for vector literals.*/
fun vec(vararg values: Number): Vec {
    val doubles = DoubleArray(values.size) { values[it].toDouble() }
    return ArrayRealVector(doubles, false)
}

/**
 * Creates a matrix using the given values; used as a DSL. Values are separated by commas, and
 * use [end] to separate rows. [end] can be either with the value [end] or the infix function [end];
 * depending on if you are in kotlin land or java land.
 *
 * For example:
 *
 * in java (static imports recommended)
 * ```
 * mat(3, -4,  5.2, end,
 *     5,  6,  7,   end,
 *     4, -1,  0)
 * ```

 */
fun mat(vararg values: Any): Mat {
    val ends = values.count { it === end }
    val pairs = values.count { it is Pair<*, *> }
    val items = values.size - ends + pairs
    val rows = ends + pairs + 1
    val cols = items / rows
    require(rows * cols == items) { "Even rows/cols not given" }
    val out = zeroMat(rows, cols)
    var curRow = 0
    var curCol = 0
    values.forEach {
        when (it) {
            is Number -> {
                require(curCol < cols) { "Even cols not given" }
                out[curRow, curCol] = it.toDouble()
                curCol++
            }

            is Pair<*, *> -> {
                val (a, b) = it
                require(a is Number && b is Number) { "Invalid value given" }
                require(curRow < rows) { "Even cols not given" }
                out[curRow, curCol] = a.toDouble()
                curRow++
                out[curRow, 0] = b.toDouble()
                curCol = 1
            }

            end -> {
                require(curRow < rows) { "Even rows not given" }
                curRow++
                curCol = 0
            }
            else -> require(false) { "Invalid value given" }
        }
    }
    return out
}

/**
 * Value used to indicate the end of a row in [mat]
 */
val end: Any = Any()

/**
 * Infix function used to indicate the end of a row in [mat].
 */
infix fun Number.end(other: Number): Pair<Number, Number> = this to other
