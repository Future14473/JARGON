@file:Suppress("NOTHING_TO_INLINE", "KDocMissingDocumentation")

package org.futurerobotics.jargon.linalg

import org.futurerobotics.jargon.math.epsEq
import org.hipparchus.linear.*
import kotlin.math.abs


//get set
inline operator fun Mat.get(row: Int, col: Int): Double = this.getEntry(row, col)

inline operator fun Mat.get(rows: IntRange, cols: IntRange): Mat =
    this.getSubMatrix(rows.first, rows.last, cols.first, cols.last)

inline operator fun Vec.get(row: Int): Double = this.getEntry(row)
inline operator fun Mat.set(row: Int, col: Int, value: Double): Unit = this.setEntry(row, col, value)
inline operator fun Mat.set(row: Int, col: Int, value: Mat): Unit = setSubMatrix(value.data, row, col)
inline operator fun Vec.set(row: Int, value: Double): Unit = this.setEntry(row, value)

//times
inline operator fun Mat.times(mat: Mat): Mat = this.multiply(mat)

inline operator fun Mat.times(vec: Vec): Vec = this.operate(vec)
inline operator fun Mat.times(vec: DoubleArray): DoubleArray = this.operate(vec)
inline operator fun Vec.times(mat: Mat): Vec = mat.preMultiply(this)

inline operator fun Mat.times(scalar: Double): Mat = this.scalarMultiply(scalar)
inline operator fun Vec.times(scalar: Double): Vec = this.mapMultiply(scalar)
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
inline operator fun Vec.div(scalar: Double): Vec = this.mapDivide(scalar)


//plus/minus
inline operator fun Mat.plus(mat: Mat): Mat = this.add(mat)

inline operator fun Vec.plus(vec: Vec): Vec = this.add(vec)


inline operator fun Mat.minus(mat: Mat): Mat = this.subtract(mat)
inline operator fun Vec.minus(vec: Vec): Vec = this.subtract(vec)

inline operator fun Mat.unaryMinus(): Mat = this * -1.0
inline operator fun Vec.unaryMinus(): Vec = this * -1.0

infix fun Vec.setTo(vec: Vec) {
    require(dimension == vec.dimension) { "Dimension mismatch" }
    repeat(dimension) {
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
    require(dimension == vec.dimension) { "Dimension mismatch" }
    repeat(dimension) { i ->
        if (!(this[i] epsEq vec[i])) return false
    }
    return true
}

// +=, -=
@PublishedApi
internal inline fun go(action: () -> Unit) = action()

inline fun Mat.combineToSelf(mat: Mat, func: Double.(Double) -> Double) {
    require(rows == mat.rows && cols == mat.cols) { "Dimension mismatch" }
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = this[r, c].func(mat[r, c])
        }
    }
}

inline fun Mat.inlineMapToSelf(func: (Double) -> Double) {
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = func(this[r, c])
        }
    }
}

inline fun Vec.combineToSelf(other: Vec, func: Double.(Double) -> Double) {
    require(dimension == other.dimension) { "Dimension mismatch" }
    repeat(dimension) {
        this[it] = this[it].func(other[it])
    }
}

inline fun Vec.inlineMapToSelf(func: (Double) -> Double) {
    repeat(dimension) { i ->
        this[i] = func(this[i])
    }
}

infix fun Mat.addI(mat: Mat): Mat = apply { combineToSelf(mat) { this + it } }
inline operator fun Mat.plusAssign(mat: Mat): Unit = go { this addI mat }

infix fun Mat.subI(mat: Mat): Mat = apply { combineToSelf(mat) { this - it } }
inline operator fun Mat.minusAssign(mat: Mat): Unit = go { this subI mat }

infix fun Mat.timesI(scalar: Double): Mat = apply { inlineMapToSelf { it * scalar } }
inline operator fun Mat.timesAssign(scalar: Double): Unit = go { this timesI scalar }

infix fun Mat.divI(scalar: Double): Mat = apply { inlineMapToSelf { it / scalar } }
inline operator fun Mat.divAssign(scalar: Double): Unit = go { this divI scalar }


infix fun Vec.addI(vec: Vec): Vec = apply { combineToSelf(vec) { this + it } }
inline operator fun Vec.plusAssign(mat: Vec): Unit = go { this addI mat }

infix fun Vec.subI(vec: Vec): Vec = apply { combineToSelf(vec) { this - it } }
inline operator fun Vec.minusAssign(mat: Vec): Unit = go { this subI mat }

infix fun Vec.timesI(scalar: Double): Vec = apply { inlineMapToSelf { it / scalar } }
inline operator fun Vec.timesAssign(scalar: Double): Unit = go { this timesI scalar }

infix fun Vec.divI(scalar: Double): Vec = apply { inlineMapToSelf { it / scalar } }
inline operator fun Vec.divAssign(scalar: Double): Unit = go { this divI scalar }

//other
infix fun Vec.emul(other: Vec): Vec = this.ebeMultiply(other)

inline val Mat.T: Mat get() = this.transpose()

inline val Mat.cols: Int get() = columnDimension
inline val Mat.rows: Int get() = rowDimension

fun Mat.solve(vec: Vec): Vec = getSolver().solve(vec)
fun Mat.solve(mat: Mat): Mat = getSolver().solve(mat)

fun Mat.getSolver(): DecompositionSolver = when {
    isSquare -> LUDecomposition(this).solver
    else -> QRDecomposition(this).solver
}

fun Mat.inv(): Mat = MatrixUtils.inverse(this)

fun Mat.pinv(): Mat = SingularValueDecomposition(this).solver.inverse