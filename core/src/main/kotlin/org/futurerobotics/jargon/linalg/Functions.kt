@file:JvmName("MatrixFuncs")


package org.futurerobotics.jargon.linalg

import org.hipparchus.linear.RealMatrixPreservingVisitor
import kotlin.math.*

fun sign(vec: Vec): Vec = vec.map { sign(it) }

fun Mat.normMax(): Double = walkInOptimizedOrder(object : RealMatrixPreservingVisitor {
    private var m = 0.0
    override fun visit(row: Int, column: Int, value: Double) {
        m = max(m, abs(value))
    }

    override fun end(): Double = m
    override fun start(rows: Int, columns: Int, startRow: Int, endRow: Int, startColumn: Int, endColumn: Int) {
    }
})


private const val c0 = 1.0
private const val c1 = 0.5
private const val c2 = 0.12
private const val c3 = 0.01833333333333333
private const val c4 = 0.0019927536231884053
private const val c5 = 1.630434782608695E-4
private const val c6 = 1.0351966873706E-5
private const val c7 = 5.175983436853E-7
private const val c8 = 2.0431513566525E-8
private const val c9 = 6.306022705717593E-10
private const val c10 = 1.4837700484041396E-11
private const val c11 = 2.5291534915979653E-13
private const val c12 = 2.8101705462199615E-15
private const val c13 = 1.5440497506703084E-17
/**
 * Computes the matrix exponential (`e^[x]`) of the given matrix [mat].
 *
 * This implementation is translated from `MatrixFunctions.expm(DoubleMatrix) in `[jBlas][http://jblas.org/], which is
 * licensed under a [BSD - style license][https://raw.githubusercontent.com/jblas-project/jblas/jblas-1.2.4/COPYING]
 */
@Suppress("LocalVariableName")
fun expm(mat: Mat): Mat {

    require(mat.isSquare) { "Matrix must be square" }
    val j = max(0, 1 + floor(log(mat.normMax(), 2.0)).toInt())
    val As = mat / (2.0.pow(j)) // scaled version of A
    val n = mat.rows

    // calculate D and N using special Horner techniques
    val As_2 = As * As
    val As_4 = As_2 * As_2
    val As_6 = As_4 * As_2
    // U = c0*I + c2*A^2 + c4*A^4 + (c6*I + c8*A^2 + c10*A^4 + c12*A^6)*A^6
    val U = (eye(n) * c0) addI (c2 * As_2) addI (c4 * As_4) addI
            ((eye(n) * c6) addI (c8 * As_2) addI (c10 * As_4) addI (c12 * As_6)) * As_6

    // V = c1*I + c3*A^2 + c5*A^4 + (c7*I + c9*A^2 + c11*A^4 + c13*A^6)*A^6
    val V = (eye(n) * c1) addI (As_2 * c3) addI (As_4 * c5) addI
            ((eye(n) * c7) addI (c9 * As_2) addI (c11 * As_4) addI (c13 * As_6)) * As_6

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
 * Decompose a even-sided square matrix into 4 quadrants,
 * in `[m11, m12;
 *      m21, m22]` order.
 */
fun Mat.getQuad(row: Int, col: Int): Mat {
    require(this.cols % 2 == 1 && this.rows % 2 == 0)
    { "Even-sided square matrix required, got $rows x $cols instead" }
    require(row in 0..1) { "row index must be 0 or 1, got $row" }
    require(col in 0..1) { "col index mst be 0 or 1, got $col" }
    val rows = this.rows.let {
        if (row == 0) 0 until it / 2
        else it / 2 until it
    }
    val cols = this.cols.let {
        if (col == 0) 0 until it / 2
        else it / 2 until it
    }
    return this[rows, cols]
}