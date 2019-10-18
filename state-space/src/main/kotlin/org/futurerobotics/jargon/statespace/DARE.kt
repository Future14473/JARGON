/*
 * The classes in this file are derivatives of similar classes in the Hipparchus project v1.5.
 * They were copied, converted to kotlin, and modified so that they solve the
 * *Discrete* Algebraic Ricatti Equation (approximately).
 * License for the Hipparchus project can be found in "Third Party Licences/Hipparchus"
 *
 * The original Hipparchus project files have the following copyright notice:
 *
 *
 * Licensed to the Hipparchus project under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The Hipparchus project licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
@file:Suppress("LocalVariableName", "PrivatePropertyName", "KDocMissingDocumentation")

package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.hipparchus.complex.Complex
import org.hipparchus.exception.LocalizedCoreFormats
import org.hipparchus.exception.MathIllegalArgumentException
import org.hipparchus.exception.MathRuntimeException
import org.hipparchus.linear.*
import org.hipparchus.util.FastMath
import java.util.*

/**
 * Given a matrix A, it computes a complex eigen decomposition A = VDV^{T}.
 *
 * It ensures that eigen values in the diagonal of D are in ascending order by magnitude.
 *
 * Except where said is modified, this is copied from [OrderedComplexEigenDecomposition] and converted to kotlin
 * using J2K
 */
class OrderedByMagComplexEigenDecomposition(matrix: RealMatrix) : ComplexEigenDecomposition(matrix) {

    init {

        val D = this.d
        val V = this.v

        val eigenValues = TreeSet(compareBy<Complex> { it.abs() }) //MODIFIED: use new comparator

        for (ij in 0 until matrix.rowDimension) {
            eigenValues.add(D.getEntry(ij, ij))
        }

        // ordering
        for (ij in 0 until matrix.rowDimension - 1) {
            val eigValue = eigenValues.pollFirst()
            var currentIndex = ij
            // searching the current index
            while (currentIndex < matrix.rowDimension) {
                val compCurrent = D.getEntry(currentIndex, currentIndex)
                if (eigValue == compCurrent) {
                    break
                }
                currentIndex++
            }

            if (ij == currentIndex) {
                continue
            }

            // exchanging D
            val previousValue = D.getEntry(ij, ij)
            D.setEntry(ij, ij, eigValue)
            D.setEntry(currentIndex, currentIndex, previousValue)

            // exchanging V
            val previousColumnV = V.getColumn(ij)
            V.setColumn(ij, V.getColumn(currentIndex))
            V.setColumn(currentIndex, previousColumnV)
        }

        checkDefinition(matrix)
    }

    /** {@inheritDoc}  */
    override fun getVT(): FieldMatrix<Complex> = v.transpose()
}


/**
 *
 * This solver computes the solution to the _DARE_ using the following approach:
 *
 * 1. Compute the Hamiltonian matrix 2. Extract its complex eigen vectors (not
 * the best solution, a better solution would be ordered Schur transformation)
 *
 * A and B should be compatible. B and R must be
 * multiplicative compatible. A and Q must be multiplicative compatible. R and A
 * must be invertible.
 *
 * @param A state transition matrix
 * @param B control multipliers matrix
 * @param Q state cost matrix
 * @param R control cost matrix
 *
 * Except where said is modified, this is copied from [LenientRiccatiEquationSolverImpl] and converted to kotlin.
 * Unused methods were removed.
 */
class DiscreteRicattiEquationSolverImpl(
    A: RealMatrix, B: RealMatrix,
    Q: RealMatrix, R: RealMatrix
) : RiccatiEquationSolver {

    /** The solution of the algebraic Riccati equation.  */
    private val P: RealMatrix

    /** The computed K.  */
    private val K: RealMatrix

    init {

        // checking A
        if (!A.isSquare) throw MathIllegalArgumentException(
            LocalizedCoreFormats.NON_SQUARE_MATRIX,
            A.rowDimension, A.columnDimension
        )
        if (A.columnDimension != B.rowDimension) throw MathIllegalArgumentException(
            LocalizedCoreFormats.DIMENSIONS_MISMATCH,
            A.rowDimension, B.rowDimension
        )
        MatrixUtils.checkMultiplicationCompatible(B, R)
        MatrixUtils.checkMultiplicationCompatible(A, Q)

        // checking R
        var svd = SingularValueDecomposition(R)
        if (!svd.solver.isNonSingular) {
            throw MathIllegalArgumentException(LocalizedCoreFormats.SINGULAR_MATRIX)
        }
        val R_inv = svd.solver.inverse

        svd = SingularValueDecomposition(A)
        if (!svd.solver.isNonSingular) {
            throw MathIllegalArgumentException(LocalizedCoreFormats.SINGULAR_MATRIX)
        }
        val A_inv = svd.solver.inverse

        P = computeInitialP(A, B, Q, A_inv, R_inv)
        //formula here changed.
        val BT = B.T
        K = (R + BT * P * B).solve(BT * P * A)
    }

    /**
     * Compute initial P using a Symplectic matrix and the ordered eigen value (by magnitude)
     * decomposition.
     *
     * @param A state transition matrix
     * @param B control multipliers matrix
     * @param Q state cost matrix
     * @param A_inv state cost matrix
     * @param R_inv inverse of matrix R
     * @return initial solution
     */
    private fun computeInitialP(
        A: RealMatrix, B: RealMatrix,
        Q: RealMatrix, A_inv: RealMatrix, R_inv: RealMatrix
    ): RealMatrix {
        val BT = B.transpose()
        val AiT = A_inv.T
        //formula changed for DARE instead
        //use kotlin extensions & func to build the z matrix instead
        val z = MatConcat.concat2x2(
            A + B * R_inv * BT * AiT * Q, -B * R_inv * BT * AiT,
            -AiT * Q, AiT
        )


        val eigenDecomposition = OrderedByMagComplexEigenDecomposition(z)
        val u = eigenDecomposition.v

        val size = A_inv.cols
        val u11 = u.getSubMatrix(
            0, size - 1, 0, size - 1
        )
        val u21 = u.getSubMatrix(
            size, 2 * size - 1, 0, size - 1
        )

        val solver = FieldLUDecomposition(u11).solver

        if (!solver.isNonSingular) throw MathRuntimeException(LocalizedCoreFormats.SINGULAR_MATRIX)

        val p = u21.multiply(solver.inverse)

        return convertToRealMatrix(p, Double.MAX_VALUE)

    }

    override fun getP(): RealMatrix = P

    override fun getK(): RealMatrix = K

    /**
     * Converts a given complex matrix into a real matrix taking into account a
     * precision for the imaginary components.
     *
     * @param matrix complex field matrix
     * @param tolerance tolerance on the imaginary part
     * @return real matrix.
     */
    private fun convertToRealMatrix(matrix: FieldMatrix<Complex>, tolerance: Double): RealMatrix {
        val toRet = MatrixUtils.createRealMatrix(matrix.rowDimension, matrix.rowDimension)
        for (i in 0 until toRet.rowDimension) {
            for (j in 0 until toRet.columnDimension) {
                val c = matrix.getEntry(i, j)
                if (c.imaginary != 0.0 && FastMath.abs(c.imaginary) > tolerance) {
                    throw MathRuntimeException(
                        LocalizedCoreFormats.COMPLEX_CANNOT_BE_CONSIDERED_A_REAL_NUMBER,
                        c.real, c.imaginary
                    )
                }
                toRet.setEntry(i, j, c.real)
            }
        }
        return toRet
    }

}
