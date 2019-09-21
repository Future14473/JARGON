package org.futurerobotics.jargon.linalg

import org.apache.commons.math3.linear.ArrayRealVector
import org.apache.commons.math3.linear.DiagonalMatrix
import org.apache.commons.math3.linear.MatrixUtils


/**
 * Creates a matrix with zeros given the [numRows] and [numCols]
 */
fun zeros(numRows: Int, numCols: Int): Mat = MatrixUtils.createRealMatrix(numRows, numCols)

fun create(data: Array<DoubleArray>): Mat = MatrixUtils.createRealMatrix(data)

fun createVec(v: DoubleArray): Vec = ArrayRealVector(v, true)
fun createVec(v: List<Double>): Vec = ArrayRealVector(v.toDoubleArray(), false)
@JvmName("createVecVararg")
fun createVec(vararg v: Double): Vec = ArrayRealVector(v, false)

fun createVec(vararg v: Int): Vec = ArrayRealVector(DoubleArray(v.size) { it.toDouble() }, false)


fun diag(v: DoubleArray): Mat = DiagonalMatrix(v)
fun diag(v: List<Double>): Mat = DiagonalMatrix(v.toDoubleArray(), false)
@JvmName("createDiagVararg")
fun diag(vararg v: Double): Mat = DiagonalMatrix(v, false)


fun eye(size: Int): Mat = MatrixUtils.createRealIdentityMatrix(size)