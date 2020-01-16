package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.isEpsEqTo
import org.junit.jupiter.api.Test
import strikt.api.expectThat

@ExperimentalStateSpace
internal class DiscreteRicattiEquationSolverImplTest {

    @Test
    fun test1() {
        val (a, b, q, r) = listOf(
            Mat(1, -1, 1 to 0, 1, 1 to 0, 0, 1),
            Mat(1, 0 to 1, 0 to 0, 1),
            Mat(10, 0, 0 to 0, 1, 0 to 0, 0, 0.1),
            Mat(10, 0 to 0, 0.1)
        )
        val test = DiscreteRiccatiEquationSolverImpl(a, b, q, r).p
        val expect = Mat(
            42.2835, -68.5247, -3.94783 to
                    -68.5247, 154.043, 16.0017 to
                    -3.94783, 16.0017, 8.33197
        )
        println(test)
        expectThat(test).isEpsEqTo(expect, 1e-4)
    }
}
