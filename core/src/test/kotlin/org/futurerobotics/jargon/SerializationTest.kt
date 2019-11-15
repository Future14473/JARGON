package org.futurerobotics.jargon

import org.futurerobotics.jargon.math.Vector2d
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isEqualTo
import java.io.File
import java.io.ObjectInputStream
import java.io.ObjectOutputStream
import java.nio.file.Paths
import java.security.SecureRandom

internal class SerializationTest {
    @Test
    fun vec() {
        val file = File(Paths.get("./tmp/test/serialize/vec.txt").toString())
        file.parentFile.mkdirs()
        println(file.absolutePath)
        val vec = Vector2d(9, 2)
        ObjectOutputStream(file.outputStream()).use {
            it.writeObject(vec)
        }

        val read = ObjectInputStream(file.inputStream())
        val vecr = read.readObject() as Vector2d

        expectThat(vec).isEqualTo(vecr)
    }

    @Test
    fun gen() {
        val random = SecureRandom()
        repeat(10) {
            println(
                """
            companion object {
                private const val serialVersionUID: Long = ${random.nextLong()}
            }
           """.trimIndent()
            )
        }
    }
}
