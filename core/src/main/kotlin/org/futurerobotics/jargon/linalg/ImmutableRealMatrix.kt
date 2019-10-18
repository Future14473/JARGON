package org.futurerobotics.jargon.linalg

import org.hipparchus.linear.RealMatrix
import org.hipparchus.linear.RealMatrixChangingVisitor
import org.hipparchus.linear.RealVector

/**
 * A dense immutable real matrix, wrapped around a copy off of another matrix.
 *
 * Operations on this matrix create a new mutable matrix.
 */
@Suppress("KDocMissingDocumentation")
class ImmutableRealMatrix private constructor(private val mat: Mat) : RealMatrix by mat {
    private val no: Nothing get() = throw UnsupportedOperationException("Matrix is immutable")

    override fun setEntry(row: Int, column: Int, value: Double): Unit = no
    override fun setColumnMatrix(column: Int, matrix: RealMatrix?): Unit = no
    override fun setColumnVector(column: Int, vector: RealVector?): Unit = no
    override fun setRow(row: Int, array: DoubleArray?): Unit = no
    override fun setRowMatrix(row: Int, matrix: RealMatrix?): Unit = no
    override fun setRowVector(row: Int, vector: RealVector?): Unit = no
    override fun setColumn(column: Int, array: DoubleArray?): Unit = no
    override fun setSubMatrix(subMatrix: Array<out DoubleArray>?, row: Int, column: Int): Unit = no
    override fun walkInOptimizedOrder(visitor: RealMatrixChangingVisitor?): Double = no
    override fun walkInOptimizedOrder(
        visitor: RealMatrixChangingVisitor?,
        startRow: Int,
        endRow: Int,
        startColumn: Int,
        endColumn: Int
    ): Double = no

    override fun walkInRowOrder(
        visitor: RealMatrixChangingVisitor?,
        startRow: Int,
        endRow: Int,
        startColumn: Int,
        endColumn: Int
    ): Double = no

    override fun walkInRowOrder(visitor: RealMatrixChangingVisitor?): Double = no
    override fun toString(): String = "Immutable $mat"

    companion object {
        /**
         * Creates an immutable matrix wrapping the given [mat].
         */
        fun of(mat: Mat): ImmutableRealMatrix =
            if (mat is ImmutableRealMatrix) mat
            else ImmutableRealMatrix(mat.copy())
    }
}

/**
 * Returns this matrix as an immutable matrix.
 */
fun Mat.toImmutableMat(): ImmutableRealMatrix = ImmutableRealMatrix.of(this)

