package org.futurerobotics.jargon.math

import koma.end
import koma.mat
import org.futurerobotics.jargon.util.of
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.math.BigInteger

class MatConcatTest {

    @Test
    fun get1() {
        val expect = mat[of the
                3, 4, 2 end
                5, 6, 3 end
                2, 3, 3 end
                8, 7, 4 end
                12, 14, 5]
        val test = MatConcat[ //@formatter:off
                mat[3, 4 end
                    5, 6],    mat[2, 3].T to
                mat[2, 3 end
                    8, 7 end
                    12, 14], mat[3, 4, 5].T //@formatter:on
        ]
        assertEquals(expect, test)
    }

    @Test
    fun get2() {
        val expect = mat[of the
                3, 4, 5, 6, 7 end
                8, 1, -1, -2, -3 end
                7, 2, -4, -5, -6]
        val test = MatConcat[//@formatter:off
                    3,BigInteger.valueOf(4), mat[ 5,  6,  7] to
                    mat[8, 7].T, mat[1, 2].T, mat[-1, -2, -3 end
                                                  -4, -5, -6]
            ]//@formatter:on
        assertEquals(expect, test)
    }

    @Test
    fun getThrows() {
        var thrown = false
        try {
            MatConcat[//@formatter:off
                    3,           4,           mat[ 5,  6,  7] to
                    mat[8, 7].T, mat[1, 2], mat[-1, -2, -3 end
                                                  -4, -5, -6]
            ]//@formatter:on
        } catch (e: Exception) {
            e.printStackTrace()
            thrown = true
        }
        assertTrue(thrown)
        thrown = false
        try {
            MatConcat[//@formatter:off
                    3,           Any(),       mat[ 5,  6,  7] to
                    mat[8, 7].T, mat[1, 2].T, mat[-1, -2, -3 end
                                                  -4, -5, -6]
            ]//@formatter:on
        } catch (e: Exception) {
            e.printStackTrace()
            thrown = true
        }
        assertTrue(thrown)
    }

    @Test
    fun concat1() {
        val expect = mat[of the
                1, 2, 3, 4, 5, 6 end
                7, 8, 9, 8, 7, 6 end
                1, 3, 5, 4, 6, 8]
        val test = MatConcat.concat(
            arrayOf<Array<Any?>>(
                arrayOf(1, 2, 3, mat[4, 5, 6]),
                arrayOf(mat[7, 1].T, mat[8, 3].T, mat[9, 5].T, mat[8, 7, 6 end 4, 6, 8])
            )
        )
        assertEquals(expect, test)
    }
}