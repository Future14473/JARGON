@file:JvmName("MatrixFuncs")

package org.futurerobotics.jargon.linalg

import org.futurerobotics.jargon.math.EPSILON
import org.futurerobotics.jargon.math.distTo
import org.futurerobotics.jargon.math.epsEq
import org.hipparchus.linear.*
import java.text.DecimalFormat
import kotlin.math.*

/** Solves for x in `Ax=b`, with a = this matrix, and b = the given [vec]. */
fun Mat.solve(vec: Vec): Vec = getSolver().solve(vec)

/** Solves for X in `AX = B`, with a = this matrix, and B = the given [mat]. */
fun Mat.solve(mat: Mat): Mat = getSolver().solve(mat)

/** Gets a general [DecompositionSolver] for the given matrix. */
fun Mat.getSolver(): DecompositionSolver = when {
    isSquare -> LUDecomposition(this).solver
    else -> QRDecomposition(this).solver
}

/** Finds the inverse of this matrix. */
fun Mat.inv(): Mat = MatrixUtils.inverse(this)

/** Finds the pseudo-inverse of this matrix. */
fun Mat.pinv(): Mat = SingularValueDecomposition(this).solver.inverse

/** Sets this vector to the given [vec]. */
infix fun Vec.setTo(vec: Vec) {
    require(size == vec.size) { "Dimension mismatch" }
    repeat(size) {
        this[it] = vec[it]
    }
}

/** Sets this matrix to the given [mat]. */
infix fun Mat.setTo(mat: Mat) {
    require(rows == mat.rows && cols == mat.cols) { "Dimension mismatch" }
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = mat[r, c]
        }
    }
}

/** Checks if all elements of this matrix match the other [mat], with a tolerance of [epsilon]. */
fun Mat.epsEq(mat: Mat, epsilon: Double): Boolean {
    require(rows == mat.rows && cols == mat.cols) { "Dimension mismatch" }
    repeat(rows) { r ->
        repeat(cols) { c ->
            if (!this[r, c].epsEq(mat[r, c], epsilon)) return false
        }
    }
    return true
}

/** Checks if all elements of this matrix match the other [vec], with a tolerance of [epsilon]. */
fun Vec.epsEq(vec: Vec, epsilon: Double): Boolean {
    require(size == vec.size) { "Dimension mismatch" }
    repeat(size) { i ->
        if (!this[i].epsEq(vec[i], epsilon)) return false
    }
    return true
}

/** Returns true if this matrix is square and symmetric with a tolerance of [epsilon]. */
fun Mat.isSymmetric(epsilon: Double = EPSILON): Boolean {
    if (!isSquare) return false
    for (r in 0 until rows) {
        for (c in r + 1 until cols) {
            if (this[r, c] distTo this[c, r] >= epsilon) return false
        }
    }
    return true
}

/** Adds this matrix with the other [mat], in place. */
infix fun Mat.addI(mat: Mat): Mat = apply {
    require(rows == mat.rows && cols == mat.cols) { "Dimension mismatch" }
    repeat(rows) { r ->
        repeat(cols) { c ->
            this.addToEntry(r, c, mat[r, c])
        }
    }
}

/** Subtracts this matrix with the other [mat], in place. */
infix fun Mat.subI(mat: Mat): Mat = apply {
    require(rows == mat.rows && cols == mat.cols) { "Dimension mismatch" }
    repeat(rows) { r ->
        repeat(cols) { c ->
            this.addToEntry(r, c, -mat[r, c])
        }
    }
}

/** Multiplies this matrix with the given [scalar], in place. */
infix fun Mat.multI(scalar: Double): Mat = apply {
    repeat(rows) { r ->
        repeat(cols) { c ->
            this.multiplyEntry(r, c, scalar)
        }
    }
}

/**
 * Divides this matrix with the given [scalar], in place.
 */
infix fun Mat.divI(scalar: Double): Mat = multI(1 / scalar)

/**
 * Adds this vector with the other [vec], in place.
 */
infix fun Vec.addI(vec: Vec): Vec = apply {
    require(size == vec.size) { "Dimension mismatch" }
    repeat(size) {
        this.addToEntry(it, vec[it])
    }
}

/**
 * Subtracts this vector with the other [vec], in place.
 */
infix fun Vec.subI(vec: Vec): Vec = apply {
    require(size == vec.size) { "Dimension mismatch" }
    repeat(size) {
        this.addToEntry(it, -vec[it])
    }
}

/**
 * Multiplies this vector with the given [scalar], in place.
 */
infix fun Vec.multI(scalar: Double): Vec = apply { mapToSelf { it * scalar } }

/**
 * Divides this vector with the given [scalar], in place.
 */
infix fun Vec.divI(scalar: Double): Vec = apply { mapToSelf { it / scalar } }

/** Converts this vector to a column matrix. */
fun Vec.toColumnMat(): Mat = createMat(Array(size) { doubleArrayOf(this[it]) }, false)

/** Converts this vector to a row matrix. */
fun Vec.toRowMatrix(): Mat = createMat(arrayOf(toArray()), false)

/** Maps each element of this vector to its sign. */
fun sign(vec: Vec): Vec = vec.map { sign(it) }

/** Gets the maximum magnitude entry in this matrix. */
fun Mat.normMax(): Double = walkInOptimizedOrder(object : RealMatrixPreservingVisitor {
    private var m = 0.0
    override fun visit(row: Int, column: Int, value: Double) {
        m = max(m, abs(value))
    }

    override fun end(): Double = m
    override fun start(rows: Int, columns: Int, startRow: Int, endRow: Int, startColumn: Int, endColumn: Int) {
    }
})

/** Returns a copy of this matrix if [copyMat] is true, else returns this. */
fun Mat.copyIf(copyMat: Boolean): Mat = if (copyMat) copy() else this

/**
 * Computes the matrix exponential (`e^X`) of the given matrix [mat].
 *
 * This implementation is translated from `MatrixFunctions.expm(DoubleMatrix)` in [jBlas][http://jblas.org/], which is
 * licensed under a [BSD - style license][https://raw.githubusercontent.com/jblas-project/jblas/jblas-1.2.4/COPYING]
 */
@Suppress("LocalVariableName")
fun expm(mat: Mat): Mat {
    require(mat.isSquare) { "Matrix must be square" }
    val c0 = 1.0
    val c1 = 0.5
    val c2 = 0.12
    val c3 = 0.01833333333333333
    val c4 = 0.0019927536231884053
    val c5 = 1.630434782608695E-4
    val c6 = 1.0351966873706E-5
    val c7 = 5.175983436853E-7
    val c8 = 2.0431513566525E-8
    val c9 = 6.306022705717593E-10
    val c10 = 1.4837700484041396E-11
    val c11 = 2.5291534915979653E-13
    val c12 = 2.8101705462199615E-15
    val c13 = 1.5440497506703084E-17

    val j = max(0, 1 + floor(log(mat.normMax(), 2.0)).toInt())
    val As = mat / (2.0.pow(j)) // scaled version of A
    val n = mat.rows

    // calculate D and N using special Horner techniques
    val As_2 = As * As
    val As_4 = As_2 * As_2
    val As_6 = As_4 * As_2
    // U = c0*I + c2*A^2 + c4*A^4 + (c6*I + c8*A^2 + c10*A^4 + c12*A^6)*A^6
    val U = (idenMat(n) * c0) addI (c2 * As_2) addI (c4 * As_4) addI
            ((idenMat(n) * c6) addI (c8 * As_2) addI (c10 * As_4) addI (c12 * As_6)) * As_6

    // V = c1*I + c3*A^2 + c5*A^4 + (c7*I + c9*A^2 + c11*A^4 + c13*A^6)*A^6
    val V = (idenMat(n) * c1) addI (As_2 * c3) addI (As_4 * c5) addI
            ((idenMat(n) * c7) addI (c9 * As_2) addI (c11 * As_4) addI (c13 * As_6)) * As_6

    val AV = As * V
    val N = U + AV
    val D = U subI AV

    // solve DF = N for F
    var F = D.solve(N)

    // now square j times
    repeat(j) {
        F *= F
    }

    return F
}

/**
 * Gets a quadrant of the matrix in `[m11, m12; m21, m22]` order,
 * splitting square at [splitIndex]
 */
fun Mat.getQuad(splitIndex: Int, row: Int, col: Int): Mat {
    require(isSquare) { "Matrix must be square" }
    val size = rows
    require(splitIndex < size) { "Split index must be less than size" }
    require(row in 0..1) { "row index must be 0 or 1, got $row" }
    require(col in 0..1) { "col index mst be 0 or 1, got $col" }
    val rows =
        if (row == 0) 0 until splitIndex
        else splitIndex until size

    val cols =
        if (col == 0) 0 until splitIndex
        else splitIndex until size

    return this[rows, cols]
}

/**
 * Formats this matrix in a more human readable format.
 */
fun Mat.formatReadable(): String = buildString {
    repeat(rows) { r ->
        append("| ")
        repeat(cols - 1) { c ->
            append(this@formatReadable[r, c].format())
            append(' ')
        }
        append(this@formatReadable[r, cols - 1].format())
        appendln(" |")
    }
}

private val decimalFormat = DecimalFormat(" 00.0000;-#")
private fun Double.format(): String = "%-8s".format(decimalFormat.format(this))
