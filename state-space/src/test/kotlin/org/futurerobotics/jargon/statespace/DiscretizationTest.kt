package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.isEpsEqTo
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import kotlin.random.Random

internal class DiscretizationTest {

    @Suppress("LocalVariableName")
    @Test
    fun `discretize test`() {
        val random = Random("discretization test".hashCode())
        repeat(5) {
            val A = Mat(3, 3) { _, _ ->
                random.nextDouble(-5.0, 5.0)
            }
            val B = Mat(3, 2) { _, _ ->
                random.nextDouble(-5.0, 5.0)
            }
            val period = 0.5

            val model = ContinuousStateSpaceMatrices(A, B, Mat(3, 3))
            val discrete = discretizeZeroOrderHold(model, period)
            expectThat(discrete.A).isEpsEqTo(expm(A * period))
            val expectedB = A.inv() * (discrete.A - idenMat(3)) * B
            expectThat(discrete.B).isEpsEqTo(expectedB)
        }
    }
}
