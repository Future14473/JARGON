package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.of
import org.junit.jupiter.api.Test
import strikt.api.expectCatching
import strikt.api.expectThat
import strikt.assertions.failed
import strikt.assertions.isA
import strikt.assertions.isEqualTo
import java.math.BigInteger

class MatConcatTest {

    @Test
    fun get1() {
        val expect = mat(
            of the
                    3, 4, 2 end
                    5, 6, 3 end
                    2, 3, 3 end
                    8, 7, 4 end
                    12, 14, 5
        )
        val test = MatConcat[ //@formatter:off
                mat(3, 4 end
                        5, 6),    mat(2, 3).T to
                mat(2, 3 end
                        8, 7 end
                                  12, 14), mat(3, 4, 5).T //@formatter:on
        ]
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun get2() {
        val expect = mat(
            of the
                    3, 4, 5, 6, 7 end
                    8, 1, -1, -2, -3 end
                    7, 2, -4, -5, -6
        )
        val test = MatConcat[//@formatter:off
                    3,BigInteger.valueOf(4), mat(5, 6, 7) to
                    mat(8, 7).T, mat(1, 2).T, mat(-1, -2, -3 end
                            -4, -5, -6)
            ]//@formatter:on
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun getThrows() {
        expectCatching {
            MatConcat[//@formatter:off
                    3,           4,         mat(5, 6, 7) to
                    mat(8, 7).T, mat(1, 2), mat(-1, -2, -3 end
                            -4, -5, -6)
            ]//@formatter:on
        }.failed().isA<Exception>()
        expectCatching {
            MatConcat[//@formatter:off
                    3,           Any(),       mat(5, 6, 7) to
                    mat(8, 7).T, mat(1, 2).T, mat(-1, -2, -3 end
                            -4, -5, -6)
            ]//@formatter:on
        }.failed().isA<Exception>()
    }

    @Test
    fun concat1() {
        val expect = mat(
            of the
                    1, 2, 3, 4, 5, 6 end
                    7, 8, 9, 8, 7, 6 end
                    1, 3, 5, 4, 6, 8
        )
        val test = MatConcat.concat(
            arrayOf(
                arrayOf(1.m, 2.m, 3.m, mat(4, 5, 6)),
                arrayOf(mat(7, 1).T, mat(8, 3).T, mat(9, 5).T, mat(8, 7, 6 end 4, 6, 8))
            )
        )
        expectThat(test).isEqualTo(expect)
    }

    private val Number.m get() = mat(this)

    @Test
    fun `concat 2x2 1`() {
        val expect = mat(
            of the
                    3, 4, 2, 5, 6, 1, 0 end
                    1, 2, 3, 4, 9, 3, -1 end
                    2, 5, 6, 3, 2, 3, 2
        )
        val test = MatConcat.concat2x2(
            mat(3, 4, 2, 5 end 1, 2, 3, 4),
            mat(6, 1, 0 end 9, 3, -1),
            mat(2, 5, 6, 3),
            mat(2, 3, 2)
        )
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun `dynamic 2x2 square 1`() {
        val expect = mat(
            of the
                    1, 0, 0, 3, 4 end
                    0, 1, 0, 1, 5 end
                    0, 0, 1, 2, 1 end
                    2, 3, 2, 0, 0 end
                    1, 3, 5, 0, 0 end
                    7, 8, 4, 0, 0
        )
        val test = MatConcat.dynamic2x2Square(
            1,
            mat(3, 4 end 1, 5 end 2, 1),
            mat(2, 3, 2 end 1, 3, 5 end 7, 8, 4),
            0
        )
        expectThat(test).isEqualTo(expect)
    }
}
