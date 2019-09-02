package org.futurerobotics.temporaryname.math

import koma.mat
import org.junit.Test

internal class VectorNamingTest {
    @Test
    fun naming() {
        val mat = mat[3, 4, 6, 7]
        val naming = VectorNaming(
            listOf(
                "speed" to "m/s",
                "size" to "m",
                "dance" to "duck",
                "race" to "fine"
            )
        )
        println(naming.toString(mat))
    }
}