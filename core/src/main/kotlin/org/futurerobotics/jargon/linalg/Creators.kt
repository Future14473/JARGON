@file:Suppress("FunctionName")

package org.futurerobotics.jargon.linalg

import org.hipparchus.linear.Array2DRowRealMatrix
import org.hipparchus.linear.ArrayRealVector
import org.hipparchus.linear.BlockRealMatrix
import org.hipparchus.linear.MatrixUtils

/*
 * Utility functions to create vectors and matrices, in the style of lists in the kotlin standard library.
 *
 * This is meant to be used from kotlin. In java, use [org.hipparchus.linear.MatrixUtils]
 */

//matrices

/**
 * Creates a zero matrix with the given number of [rows] and [cols]
 */
fun Mat(rows: Int, cols: Int): Mat = MatrixUtils.createRealMatrix(rows, cols)

/**
 * Creates a matrix with the given number of [rows] and [cols], initializing values using the given [init] function.
 */
inline fun Mat(rows: Int, cols: Int, init: (r: Int, c: Int) -> Double): Mat {
    require(rows > 0) { "Rows ($rows) must be > 0" }
    require(cols > 0) { "Cols ($cols) must be > 0" }
    val data = Array(rows) { r ->
        DoubleArray(cols) { c ->
            init(r, c)
        }
    }
    return matFrom(data, false)
}

/**
 * Creates a matrix from a given 2d double array [data].
 *
 * @param copy if true, the array will be copied. If the data contains a large number of elements, the array will
 * always be copied regardless of this value.
 */
fun matFrom(data: Array<DoubleArray>, copy: Boolean = true): Mat {
    val rows = data.size
    val cols = data[0].size

    return if (rows * cols <= 4096) Array2DRowRealMatrix(data, copy) else BlockRealMatrix(data)
}

/**
 * Converts [this] 2d double array to a matrix.
 *
 * @param copy if true, the array will be copied.
 */
fun Array<DoubleArray>.toMat(copy: Boolean = true): Mat = matFrom(this, copy)

/** Creates an identity matrix with the given [size]. */
fun idenMat(size: Int): Mat = MatrixUtils.createRealIdentityMatrix(size)

/** Creates a matrix with the given [values] along the diagonal. */
fun diagMatOf(values: DoubleArray): Mat = MatrixUtils.createRealDiagonalMatrix(values)

/** Creates a matrix with the given [values] along the diagonal. */
@JvmName("diagMatOfVararg")
fun diagMatOf(vararg values: Double): Mat = MatrixUtils.createRealDiagonalMatrix(values)

/** Creates a matrix with the given [values] along the diagonal. */
fun diagMatOf(values: Vec): Mat = Mat(values.size, values.size).apply {
    values.forEachIndexed { i, d ->
        this[i, i] = d
    }
}

/** Creates a matrix with [this] vectors' values on its diagonal. */
fun Vec.toDiagMat(): Mat = diagMatOf(this)

/**
 * Creates a matrix using the given number values in the syntax described below. This is typically
 * used for matrix literals.
 *
 * Number values should be separated by commas, and rows separated using the
 * infix function [to].
 *
 * For example:
 * ```kotlin
 * matOf(
 *  1, 2, 3 to
 *  4, 5, 6 to
 *  6, 7, 0
 * )
 * ```
 *
 * In java, use [MatrixUtils.createRealMatrix] instead with a 2d array literal.
 */
@Suppress("FunctionName")
@JvmSynthetic
fun matOf(vararg values: Any): Mat {
    return varargBuilderToArray<Number, Mat>(
        values,
        { r, c -> Mat(r, c) },
        { r, c, e -> this[r, c] = e.toDouble() }
    )
}

//vectors

/**
 * Creates a zero vector with the given [size].
 */
fun Vec(size: Int): Vec = ArrayRealVector(size)

/**
 * Creates a size with a given [size], initializing using the given [init] function.
 */
inline fun Vec(size: Int, init: (i: Int) -> Double): Vec {
    val data = DoubleArray(size) { i -> init(i) }
    return vecFrom(data, false)
}

/**
 * Creates a vector from a given double array [data].
 *
 * @param copy if true, the array will be copied.
 */
fun vecFrom(data: DoubleArray, copy: Boolean = true): Vec = ArrayRealVector(data, copy)

/**
 * Converts this double array to a vector.
 *
 * @param copy if true, the array will be copied.
 */
fun DoubleArray.toVec(copy: Boolean = true): Vec = ArrayRealVector(this, copy)

/**
 * Creates a vector from the given [values].
 */
fun vecOf(vararg values: Double): Vec = ArrayRealVector(values, false)
