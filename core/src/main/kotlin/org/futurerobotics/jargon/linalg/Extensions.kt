@file:Suppress("NOTHING_TO_INLINE")

package org.futurerobotics.jargon.linalg

import org.hipparchus.linear.*


//getset
inline operator fun Mat.get(row: Int, col: Int): Double = this.getEntry(row, col)

inline operator fun Mat.get(rows: IntRange, cols: IntRange) =
    this.getSubMatrix(rows.first, rows.last, cols.first, cols.last)

inline operator fun Vec.get(row: Int): Double = this.getEntry(row)

inline operator fun Mat.set(row: Int, col: Int, value: Double): Unit = this.setEntry(row, col, value)
inline operator fun Mat.set(row: Int, col: Int, value: Mat) = setSubMatrix(value.data, row, col)
inline operator fun Vec.set(row: Int, value: Double): Unit = this.setEntry(row, value)

//times
inline operator fun Mat.times(mat: Mat): Mat = this.multiply(mat)

inline operator fun Mat.times(scalar: Double): Mat = this.scalarMultiply(scalar)
inline operator fun Double.times(mat: Mat): Mat = mat.scalarMultiply(this)
inline operator fun Mat.times(vec: Vec): Vec = this.operate(vec)

inline operator fun Mat.times(vec: DoubleArray): DoubleArray = this.operate(vec)
inline operator fun Vec.times(scalar: Double): Vec = this.mapMultiply(scalar)

inline operator fun Double.times(vec: Vec): Vec = vec.mapMultiply(this)
inline operator fun Mat.invoke(mat: Mat): Mat = this.multiply(mat)
inline operator fun Mat.invoke(scalar: Double): Mat = this.scalarMultiply(scalar)
inline operator fun Double.invoke(mat: Mat): Mat = mat.scalarMultiply(this)
inline operator fun Mat.invoke(vec: Vec): Vec = this.operate(vec)

inline operator fun Mat.invoke(vec: DoubleArray): DoubleArray = this.operate(vec)
inline operator fun Vec.invoke(scalar: Double): Vec = this.mapMultiply(scalar)

inline operator fun Double.invoke(vec: Vec): Vec = vec.mapMultiply(this)

//add/sub
inline operator fun Mat.plus(mat: Mat): Mat = this.add(mat)
inline operator fun Vec.plus(vec: Vec): Vec = this.add(vec)

inline operator fun Mat.minus(mat: Mat): Mat = this.subtract(mat)
inline operator fun Vec.minus(vec: Vec): Vec = this.subtract(vec)


inline operator fun Mat.unaryMinus(): Mat = this * -1.0
inline operator fun Vec.unaryMinus(): Vec = this * -1.0

infix fun Vec.emul(other: Vec): Vec = this.ebeMultiply(other)


inline val Mat.T: Mat get() = this.transpose()

inline val Mat.cols: Int get() = columnDimension
inline val Mat.rows: Int get() = rowDimension

fun Mat.solve(vec: Vec): Vec = generalSolver().solve(vec)
fun Mat.solve(mat: Mat): Mat = generalSolver().solve(mat)

private fun Mat.generalSolver(): DecompositionSolver = when {
    isSquare -> LUDecomposition(this).solver
    else -> QRDecomposition(this).solver
}

fun Mat.inv(): RealMatrix {
    return MatrixUtils.inverse(this)
}

fun Mat.pinv(): RealMatrix {
    return SingularValueDecomposition(this).solver.inverse
}