package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.normalize
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isEqualTo

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
            
            """.trimIndent().normalize()
        expectThat(naming.format(vec).normalize()).isEqualTo(expected)
    }
}