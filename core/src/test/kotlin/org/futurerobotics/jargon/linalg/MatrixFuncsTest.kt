package org.futurerobotics.jargon.linalg

import org.futurerobotics.jargon.math.isEpsEqTo
import org.futurerobotics.jargon.of
import org.junit.jupiter.api.Test
import strikt.api.expectThat

internal class MatrixFuncsTest {
    @Test
    fun expm() {
        val aMat = expm(
            mat[of the
                    3, 2, -1, 3 end
                    4, -2, 0, -4 end
                    6, 5, -2, 3 end
                    0, 1, -1, 2]
        )
        val expected = mat[of the
                9.10682, 2.21611, -4.9011, 11.3573 end
                15.6455, 4.16685, -4.24013, 8.00694 end
                27.063, 7.42418, -9.48559, 20.5082 end
                -8.25672, -2.30821, 1.01201, -1.03677]
        expectThat(aMat).isEpsEqTo(expected, 1e-4)
    }
}