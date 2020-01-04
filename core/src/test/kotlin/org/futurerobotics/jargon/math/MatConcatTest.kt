package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.of
import org.junit.jupiter.api.Test
import strikt.api.expectCatching
import strikt.api.expectThat
import strikt.assertions.failed
import strikt.assertions.isA
import strikt.assertions.isEqualTo

class MatConcatTest {

    @Test
    fun get1() {
        val expect = Mat(
            of the
                    3, 4, 2 to
                    5, 6, 3 to
                    2, 3, 3 to
                    8, 7, 4 to
                    12, 14, 5
        )
        val test = concat(//@formatter:off
                    Mat(3, 4 to
                     5, 6), Mat(2, 3).T to
                              Mat(2, 3 to
                               8, 7 to
                                12, 14
                              ), Mat(3, 4, 5).T //@formatter:on
        )
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun get2() {
        val expect = Mat(
            of the
                    3, 4, 5, 6, 7 to
                    8, 1, -1, -2, -3 to
                    7, 2, -4, -5, -6
        )
        val test = concat(//@formatter:off
                        Mat(3), Mat(4), Mat(5, 6, 7) to
                Mat(8, 7).T, Mat(1, 2).T, Mat(-1, -2, -3 to
                 -4, -5, -6)
     )//@formatter:on
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun getThrows() {
        expectCatching {
            concat(//@formatter:off
                            Mat(3), Mat(4), Mat(5, 6, 7) to
                    Mat(8, 7).T, Mat(1, 2), Mat(-1, -2, -3 to -4, -5, -6)
         )//@formatter:on
        }.failed().isA<Exception>()
        expectCatching {
            concat(//@formatter:off
                            3, Any(), Mat(5, 6, 7) to
                    Mat(8, 7).T, Mat(1, 2).T, Mat(-1, -2, -3 to
                     -4, -5, -6)
         )//@formatter:on
        }.failed().isA<Exception>()
    }

    @Test
    fun concat1() {
        val expect = Mat(
            of the
                    1, 2, 3, 4, 5, 6 to
                    7, 8, 9, 8, 7, 6 to
                    1, 3, 5, 4, 6, 8
        )
        val test = concat(
            arrayOf(
                arrayOf(1.m, 2.m, 3.m, Mat(4, 5, 6)),
                arrayOf(
                    Mat(7, 1).T, Mat(8, 3).T, Mat(9, 5).T,
                    Mat(8, 7, 6 to 4, 6, 8)
                )
            )
        )
        expectThat(test).isEqualTo(expect)
    }

    private val Number.m get() = Mat(this)

    @Test
    fun `concat 2x2 1`() {
        val expect = Mat(
            of the
                    3, 4, 2, 5, 6, 1, 0 to
                    1, 2, 3, 4, 9, 3, -1 to
                    2, 5, 6, 3, 2, 3, 2
        )
        val test = concat2x2(
            Mat(3, 4, 2, 5 to 1, 2, 3, 4),
            Mat(6, 1, 0 to 9, 3, -1),
            Mat(2, 5, 6, 3),
            Mat(2, 3, 2)
        )
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun `concat 2x2 dynamic 1`() {
        val expect = Mat(
            of the
                    1, 0, 0, 3, 4 to
                    0, 1, 0, 1, 5 to
                    0, 0, 1, 2, 1 to
                    2, 3, 2, 0, 0 to
                    1, 3, 5, 0, 0 to
                    7, 8, 4, 0, 0
        )
        val test = concat2x2dynamic(
            1,
            Mat(3, 4 to 1, 5 to 2, 1),
            Mat(2, 3, 2 to 1, 3, 5 to 7, 8, 4),
            0
        )
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun `concat dynamic 1`() {
        val expect = Mat(
            of the
                    1, 0, 0, 3, 4, 0 to
                    0, 1, 0, 1, 5, 0 to
                    0, 0, 1, 2, 1, 0 to

                    2, 3, 2, 0, 0, 1 to
                    1, 3, 5, 0, 0, 2 to

                    0, 0, 0, 1, 0, 0 to
                    0, 0, 0, 0, 1, 0
        )
        val test = concatDynamic(
            of the
                    1, Mat(3, 4 to 1, 5 to 2, 1), 0 to
                    Mat(2, 3, 2 to 1, 3, 5), 0, Mat(1, 2).T to
                    0, 1, 0
        )
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun `concat col 2`() {
        val expect = Mat(
            of the
                    0, 1, 2 to
                    3, 4, 5 to
                    6, 7, 8 to
                    9, 0, 1
        )
        val test = concatCol(
            Mat(0, 1, 2 to 3, 4, 5), Mat(6, 7, 8 to 9, 0, 1)
        )
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun `concat col many`() {
        val expect = Mat(
            of the
                    0, 1, 2 to
                    3, 4, 5 to
                    6, 7, 8 to
                    9, 0, 1 to
                    7, 8, 9
        )
        val test = concatCol(
            Mat(0, 1, 2 to 3, 4, 5), Mat(6, 7, 8 to 9, 0, 1), Mat(7, 8, 9)
        )
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun `concat row 2`() {
        val expect = Mat(
            of the
                    0, 1, 2, 5 to
                    3, 4, 5, 8 to
                    6, 7, 8, 1
        )
        val test = concatRow(
            Mat(0, 1 to 3, 4 to 6, 7), Mat(2, 5 to 5, 8 to 8, 1)
        )
        expectThat(test).isEqualTo(expect)
    }

    @Test
    fun `concat row many`() {
        val expect = Mat(
            of the
                    0, 1, 2, 5, 3 to
                    3, 4, 5, 8, 2 to
                    6, 7, 8, 1, 1
        )
        val test = concatRow(
            Mat(0, 1 to 3, 4 to 6, 7), Mat(2, 5 to 5, 8 to 8, 1), Mat(3, 2, 1).T
        )
        expectThat(test).isEqualTo(expect)
    }
}
