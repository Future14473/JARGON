@file:Suppress("NOTHING_TO_INLINE", "KDocMissingDocumentation")

package org.futurerobotics.jargon.linalg

//get set

inline operator fun Mat.get(row: Int, col: Int): Double = getEntry(row, col)
inline operator fun Mat.get(rows: IntRange, cols: IntRange): Mat =
    getSubMatrix(rows.first, rows.last, cols.first, cols.last)

inline operator fun Vec.get(ind: Int): Double = getEntry(ind)
inline operator fun Vec.get(indices: IntRange): Vec =
    getSubVector(indices.first, indices.last - indices.first + 1)

inline operator fun Mat.set(row: Int, col: Int, subMat: Double): Unit = setEntry(row, col, subMat)
inline operator fun Mat.set(row: Int, col: Int, value: Mat): Unit = setSubMatrix(value.data, row, col)

inline operator fun Vec.set(ind: Int, value: Double): Unit = setEntry(ind, value)
inline operator fun Vec.set(ind: Int, subVec: Vec): Unit = setSubVector(ind, subVec)

//times

inline operator fun Mat.times(vec: Vec): Vec = operate(vec)
inline operator fun Mat.times(mat: Mat): Mat = multiply(mat)
inline operator fun Mat.times(vec: DoubleArray): DoubleArray = operate(vec)
inline operator fun Mat.times(scalar: Double): Mat = scalarMultiply(scalar)

inline operator fun Vec.times(scalar: Double): Vec = mapMultiply(scalar)

inline operator fun Double.times(mat: Mat): Mat = mat.scalarMultiply(this)
inline operator fun Double.times(vec: Vec): Vec = vec.mapMultiply(this)

inline operator fun Mat.invoke(mat: Mat): Mat = this * mat
inline operator fun Mat.invoke(vec: Vec): Vec = this * vec
inline operator fun Mat.invoke(vec: DoubleArray): DoubleArray = this * vec
inline operator fun Mat.invoke(scalar: Double): Mat = this * scalar

inline operator fun Vec.invoke(scalar: Double): Vec = this * scalar

inline operator fun Double.invoke(mat: Mat): Mat = this * mat
inline operator fun Double.invoke(vec: Vec): Vec = this * vec

inline operator fun Mat.div(scalar: Double): Mat = this * (1 / scalar)
inline operator fun Vec.div(scalar: Double): Vec = mapDivide(scalar)

//plus/minus

inline operator fun Mat.plus(mat: Mat): Mat = add(mat)
inline operator fun Vec.plus(vec: Vec): Vec = add(vec)

inline operator fun Mat.minus(mat: Mat): Mat = subtract(mat)
inline operator fun Vec.minus(vec: Vec): Vec = subtract(vec)

inline operator fun Mat.unaryMinus(): Mat = this * -1.0
inline operator fun Vec.unaryMinus(): Vec = this * -1.0

// +=, -=

inline operator fun Mat.plusAssign(mat: Mat) {
    this addI mat
}

inline operator fun Mat.minusAssign(mat: Mat) {
    this subI mat
}

inline operator fun Mat.timesAssign(scalar: Double) {
    this multI scalar
}

inline operator fun Mat.divAssign(scalar: Double) {
    this divI scalar
}

inline operator fun Vec.plusAssign(mat: Vec) {
    this addI mat
}

inline operator fun Vec.minusAssign(mat: Vec) {
    this subI mat
}

inline operator fun Vec.timesAssign(scalar: Double) {
    this multI scalar
}

inline operator fun Vec.divAssign(scalar: Double) {
    this divI scalar
}

//other
infix fun Vec.emul(other: Vec): Vec = ebeMultiply(other)

inline val Mat.T: Mat get() = transpose()
val Vec.T: Mat get() = toRowMatrix()

inline val Mat.cols: Int get() = columnDimension
inline val Mat.rows: Int get() = rowDimension

inline val Vec.size: Int get() = dimension
