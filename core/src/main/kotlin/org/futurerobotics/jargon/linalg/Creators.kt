@file:Suppress("FunctionName")

package org.futurerobotics.jargon.linalg

import org.hipparchus.linear.Array2DRowRealMatrix
import org.hipparchus.linear.ArrayRealVector
import org.hipparchus.linear.BlockRealMatrix
import org.hipparchus.linear.MatrixUtils

/**
 * Creates a zero matrix with the given number of [rows] and [cols]
 */
fun Mat(rows: Int, cols: Int): Mat {
    require(rows > 0) { "Rows ($rows) must be > 0" }
    require(cols > 0) { "Cols ($cols) must be > 0" }
    return MatrixUtils.createRealMatrix(rows, cols)
}

/**
 * Creates a matrix with the given number of [rows] and [cols], initializing using the given [init] function.
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
 * @param copy if true, will copy array.
 */
fun matFrom(data: Array<DoubleArray>, copy: Boolean = true): Mat {
    val rows = data.size
    val cols = data[0].size

    return if (rows * cols <= 4096) Array2DRowRealMatrix(data, copy) else BlockRealMatrix(data)
}

/**
 * Converts [this] 2d double array to a matrix.
 *
 * @param copy if true, will copy array, else will not.
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
fun diagMatFrom(values: Vec): Mat = Mat(values.size, values.size).apply {
    values.forEachIndexed { i, d ->
        this[i, i] = d
    }
}

/** Creates a [Mat] with [this] vectors' values on its diagonal. */
fun Vec.toDiagMat(): Mat = diagMatFrom(this)

/**
 * Creates a zero vector with the given [size].
 */
fun Vec(size: Int): Vec {
    require(size > 0) { "Size ($size) must be > 0" }
    return ArrayRealVector(size)
}

/**
 * Creates a size with a given [size], initializing using the given [init] function.
 */
inline fun Vec(size: Int, init: (i: Int) -> Double): Vec {
    require(size > 0) { "Rows ($size) must be > 0" }
    val data = DoubleArray(size) { i -> init(i) }
    return vecFrom(data, false)
}

/**
 * Creates a vector from a given double array [data].
 *
 * @param copy if true, will copy mat.
 */
fun vecFrom(data: DoubleArray, copy: Boolean = true): Vec = ArrayRealVector(data, copy)

/**
 * Converts this double array to a vector.
 *
 * @param copy if true, will copy mat.
 */
fun DoubleArray.toVec(copy: Boolean = true): Vec = ArrayRealVector(this, copy)

/**
 * Creates a vector from the given [values].
 */
fun vecOf(vararg values: Double): Vec = ArrayRealVector(values, false)

/**
 * Creates a matrix using the given number values, used for matrix literals.
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
fun matOf(vararg values: Any): Mat {
    lateinit var mat: Mat
    varargEndToArr<Number>(
        values,
        { r, c -> mat = Mat(r, c) },
        { r, c, e -> mat[r, c] = e.toDouble() }
    )
    return mat
}

//ALL OF below is going to replace

/** Creates a matrix with the given [rows] and [cols], filling with values given by [init]. */
@Deprecated("Use new", ReplaceWith("Mat(rows, cols, init)"))
inline fun genMat(rows: Int, cols: Int, init: (r: Int, c: Int) -> Double): Mat = Mat(rows, cols, init)

/** Creates a [matFrom] from a 2d double array. */
@JvmOverloads
@Deprecated("Use new", ReplaceWith("Mat(data, copy)"))
fun createMat(data: Array<DoubleArray>, copy: Boolean = true): Mat =
    matFrom(data, copy)

/** Creates a matrix filled with zeros with the given [rows] and [cols]. */
@Deprecated("Use new", ReplaceWith("Mat(rows, cols)"))
fun zeroMat(rows: Int, cols: Int): Mat = Mat(rows, cols)

/** Creates a vector with the given [size], and filling values given by [init]. */
@Deprecated("Use new", ReplaceWith("Vec(size, init)"))
inline fun genVec(size: Int, init: (Int) -> Double): Vec = Vec(size, init)

/** Creates a vector with the given [values], optionally [copy]ing the array. */
@JvmOverloads
@Deprecated("Use new", ReplaceWith("vecFrom(values, copy)"))
fun createVec(values: DoubleArray, copy: Boolean = true): Vec = vecFrom(values, copy)

/** Creates a vector with the given [values]. */
@JvmName("createVecVararg")
@Deprecated("Use new", ReplaceWith("vecOf(*values)"))
fun createVec(vararg values: Double): Vec = vecOf(*values)

/** Creates a vector filled with zeros with the given [size]. */
@Deprecated("Use new", ReplaceWith("Vec(size)"))
fun zeroVec(size: Int): Vec = Vec(size)

/**
 * Creates a matrix using the given number values, used for matrix literals.
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
@Deprecated("Use new", ReplaceWith("mat(*values)"))
fun Mat(vararg values: Any): Mat = matOf(*values)

/** Creates a matrix with the given [values] along the diagonal. */
@Deprecated("Use new", ReplaceWith("diagMatOf(values)"))
fun diagMat(values: DoubleArray): Mat = diagMatOf(values)

/** Creates a matrix with the given [values] along the diagonal. */
@JvmName("diagMatVararg")
@Deprecated("Use new", ReplaceWith("diagMatOf(*values)"))
fun diagMat(vararg values: Double): Mat = diagMatOf(*values)
