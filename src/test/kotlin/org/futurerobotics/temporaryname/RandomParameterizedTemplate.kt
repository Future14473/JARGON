@file:Suppress("RemoveRedundantBackticks")

package org.futurerobotics.temporaryname

import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

private typealias Thing = Any

@RunWith(Parameterized::class)
internal class Template(private val thing: Thing) {

    @Test
    fun `test`() {
    }

    companion object {
        private val random = Random(2346)
        private const val range = 10.0
        @JvmStatic
        @Parameterized.Parameters
        fun getParams(): List<Array<Any>> {
            return listOf(arrayOf(Any()))
        }
    }
}