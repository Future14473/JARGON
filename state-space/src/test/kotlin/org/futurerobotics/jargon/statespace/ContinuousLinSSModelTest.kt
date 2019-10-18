package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.isEpsEqTo
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import kotlin.random.Random

internal class ContinuousLinSSModelTest {

    @Test
    fun discretize() {
        val random = Random("discretization test".hashCode())
        repeat(5) {
            val A = create(3, 3) { _, _ ->
                random.nextDouble(-5.0, 5.0)
            }
            val B = create(3, 2) { _, _ ->
                random.nextDouble(-5.0, 5.0)
            }
            val period = 0.5

            val model = ContinuousLinSSModel(A, B, zeros(3, 3), zeros(3, 2))
            val discrete = model.discretize(period)
            expectThat(discrete.A).isEpsEqTo(expm(A * period))
            val expectedB = A.inv()(discrete.A - eye(3)) * B
            expectThat(discrete.B).isEpsEqTo(expectedB)
        }
    }
}