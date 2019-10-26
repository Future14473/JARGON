@file:Suppress("NOTHING_TO_INLINE", "KDocMissingDocumentation")

package org.futurerobotics.jargon.linalg

import org.futurerobotics.jargon.math.epsEq
import org.hipparchus.linear.*
import kotlin.math.abs

//get set
inline operator fun Mat.get(row: Int, col: Int): Double = getEntry(row, col)

inline operator fun Vec.get(ind: Int): Double = getEntry(ind)

inline operator fun Mat.get(rows: IntRange, cols: IntRange): Mat =
    this.getSubMatrix(rows.first, rows.last, cols.first, cols.last)

inline operator fun Vec.get(indices: IntRange): Vec =
    getSubVector(indices.first, indices.last - indices.first + 1)

inline operator fun Mat.set(row: Int, col: Int, subMat: Double): Unit = setEntry(row, col, subMat)
inline operator fun Mat.set(row: Int, col: Int, value: Mat): Unit = setSubMatrix(value.data, row, col)

inline operator fun Vec.set(ind: Int, value: Double): Unit = setEntry(ind, value)
inline operator fun Vec.set(ind: Int, subVec: Vec): Unit = setSubVector(ind, subVec)

//times
inline operator fun Mat.times(mat: Mat): Mat = multiply(mat)

inline operator fun Mat.times(vec: Vec): Vec = this.operate(vec)
inline operator fun Mat.times(vec: DoubleArray): DoubleArray = this.operate(vec)
inline operator fun Vec.times(mat: Mat): Vec = mat.preMultiply(this)

inline operator fun Mat.times(scalar: Double): Mat = scalarMultiply(scalar)
inline operator fun Vec.times(scalar: Double): Vec = mapMultiply(scalar)
inline operator fun Double.times(mat: Mat): Mat = mat.scalarMultiply(this)
inline operator fun Double.times(vec: Vec): Vec = vec.mapMultiply(this)

inline operator fun Mat.invoke(mat: Mat): Mat = this * mat
inline operator fun Mat.invoke(vec: Vec): Vec = this * vec
inline operator fun Mat.invoke(vec: DoubleArray): DoubleArray = this * vec
inline operator fun Vec.invoke(mat: Mat): Vec = this * mat

inline operator fun Mat.invoke(scalar: Double): Mat = this * scalar
inline operator fun Vec.invoke(scalar: Double): Vec = this * scalar
inline operator fun Double.invoke(mat: Mat): Mat = this * mat
inline operator fun Double.invoke(vec: Vec): Vec = this * vec

inline operator fun Mat.div(scalar: Double): Mat = this * (1 / scalar)
inline operator fun Vec.div(scalar: Double): Vec = mapDivide(scalar)

//plus/minus
inline operator fun Mat.plus(mat: Mat): Mat = add(mat)

inline operator fun Vec.plus(vec: Vec): Vec = add(vec)

inline operator fun Mat.minus(mat: Mat): Mat = this.subtract(mat)
inline operator fun Vec.minus(vec: Vec): Vec = this.subtract(vec)

inline operator fun Mat.unaryMinus(): Mat = this * -1.0
inline operator fun Vec.unaryMinus(): Vec = this * -1.0

infix fun Vec.setTo(vec: Vec) {
    require(size == vec.size) { "Dimension mismatch" }
    repeat(size) {
        this[it] = vec[it]
    }
}

infix fun Mat.setTo(mat: Mat) {
    require(rows == mat.rows && cols == mat.cols) { "Dimension mismatch" }
    repeat(rows) { i ->
        repeat(cols) { j ->
            this[i, j] = mat[i, j]
        }
    }
}

fun Mat.epsEq(mat: Mat, epsilon: Double): Boolean {
    require(rows == mat.rows && cols == mat.cols) { "Dimension mismatch" }
    repeat(rows) { i ->
        repeat(cols) { j ->
            if (abs(this[i, j] - mat[i, j]) >= epsilon) return false
        }
    }
    return true
}

infix fun Vec.epsEq(vec: Vec): Boolean {
    require(size == vec.size) { "Dimension mismatch" }
    repeat(size) { i ->
        if (!(this[i] epsEq vec[i])) return false
    }
    return true
}

// +=, -=

infix fun Mat.addI(mat: Mat): Mat = apply { combineToSelf(mat) { this + it } }
inline operator fun Mat.plusAssign(mat: Mat) {
    this addI mat
}

infix fun Mat.subI(mat: Mat): Mat = apply { combineToSelf(mat) { this - it } }
inline operator fun Mat.minusAssign(mat: Mat) {
    this subI mat
}

infix fun Mat.timesI(scalar: Double): Mat = apply { mapToSelf { it * scalar } }
inline operator fun Mat.timesAssign(scalar: Double) {
    this timesI scalar
}

infix fun Mat.divI(scalar: Double): Mat = apply { mapToSelf { it / scalar } }
inline operator fun Mat.divAssign(scalar: Double) {
    this divI scalar
}

infix fun Vec.addI(vec: Vec): Vec = apply { combineToSelf(vec) { this + it } }
inline operator fun Vec.plusAssign(mat: Vec) {
    this addI mat
}

infix fun Vec.subI(vec: Vec): Vec = apply { combineToSelf(vec) { this - it } }
inline operator fun Vec.minusAssign(mat: Vec) {
    this subI mat
}

infix fun Vec.timesI(scalar: Double): Vec = apply { mapToSelf { it / scalar } }
inline operator fun Vec.timesAssign(scalar: Double) {
    this timesI scalar
}

infix fun Vec.divI(scalar: Double): Vec = apply { mapToSelf { it / scalar } }
inline operator fun Vec.divAssign(scalar: Double) {
    this divI scalar
}

//other
infix fun Vec.emul(other: Vec): Vec = ebeMultiply(other)

inline val Mat.T: Mat get() = transpose()

inline val Mat.cols: Int get() = columnDimension
inline val Mat.rows: Int get() = rowDimension

inline val Vec.size: Int get() = dimension

fun Mat.solve(vec: Vec): Vec = getSolver().solve(vec)
fun Mat.solve(mat: Mat): Mat = getSolver().solve(mat)

fun Mat.getSolver(): DecompositionSolver = when {
    isSquare -> LUDecomposition(this).solver
    else -> QRDecomposition(this).solver
}

fun Mat.inv(): Mat = MatrixUtils.inverse(this)

fun Mat.pinv(): Mat = SingularValueDecomposition(this).solver.inverse
