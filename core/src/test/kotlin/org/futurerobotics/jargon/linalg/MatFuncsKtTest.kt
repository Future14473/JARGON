package org.futurerobotics.jargon.linalg

import org.futurerobotics.jargon.math.isEpsEqTo
import org.futurerobotics.jargon.of
import org.junit.jupiter.api.Test
import strikt.api.expectThat

internal class MatFuncsKtTest {
    @Test
    fun expm() {
        val aMat = expm(
            matOf(
                of the
                        3, 2, -1, 3 to
                        4, -2, 0, -4 to
                        6, 5, -2, 3 to
                        0, 1, -1, 2
            )
        )
        val expected = matOf(
            of the
                    9.10682, 2.21611, -4.9011, 11.3573 to
                    15.6455, 4.16685, -4.24013, 8.00694 to
                    27.063, 7.42418, -9.48559, 20.5082 to
                    -8.25672, -2.30821, 1.01201, -1.03677
        )
        expectThat(aMat).isEpsEqTo(expected, 1e-4)
    }
}
