package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*
import org.junit.Assert
import org.junit.Test

internal class VectorStructureTest {
    @Test
    fun naming() {
        val vec = createVec(3.0, 4.0, 6.0, 7.0)
        val naming = VectorStructure(
            listOf(
                "speed" to "m/s",
                "length" to "m",
                "coolness" to "rads",
                "absurdity" to "level"
            )
        )
        val expected = """
            [
            speed: 3.0 m/s
            length: 4.0 m
            coolness: 6.0 rads
            absurdity: 7.0 level
            ]
            """.trimIndent()
        Assert.assertEquals(expected, naming.toString(vec).replace("\r\n","\n"))
    }
}