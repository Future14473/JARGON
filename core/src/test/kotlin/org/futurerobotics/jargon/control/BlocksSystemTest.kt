package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.control.Block.Processing.OUT_FIRST_ALWAYS
import org.futurerobotics.jargon.system.SimpleLoopSystemDriver
import org.junit.jupiter.api.Test
import strikt.api.expectCatching
import strikt.api.expectThat
import strikt.assertions.failed
import strikt.assertions.isA
import strikt.assertions.isEqualTo

internal abstract class AbstractBlockSystemTest {
    protected fun testBlock(
        name: String,
        numInputs: Int,
        numOutputs: Int,
        processing: Block.Processing = Block.Processing.IN_FIRST_LAZY,
        requireAllInputs: Boolean = true
    ): TestBlock {
        return TestBlock(name, numInputs, numOutputs, processing, requireAllInputs)
    }

    protected fun emptyBlock(
        processing: Block.Processing = Block.Processing.IN_FIRST_LAZY
    ) = testBlock("Empty", 0, 0, processing)
}

internal class BlocksSystemTest : AbstractBlockSystemTest() {

    @Test
    fun `update order test`() {
        val monitor: Monitor<String>

        val system = buildBlocksSystem {
            val b = testBlock("B", 3, 2, requireAllInputs = false)
            val c = testBlock("C", 1, 2)
            val d = testBlock("D", 1, 1, OUT_FIRST_ALWAYS)
            val e = testBlock("E", 1, 1, OUT_FIRST_ALWAYS)
            val f = testBlock("F", 2, 2)
            val g = testBlock("G", 2, 1, IN_FIRST_ALWAYS)
            val h = testBlock("H", 2, 1)

            b.connectFromAll(Constant("A"), e.output(0))
            c.connectFromAll(b.output(0))
            d.connectFromAll(h.output(0))
            e.connectFromAll(f.output(1))
            f.connectFromAll(b.output(1), d.output(0))
            g.connectFromAll(c.output(0), f.output(0))
            h.connectFromAll(c.output(0), g.output(0))
            monitor = d.input().monitor()
        }

        expectThat(monitor) {
            repeat(4) { i ->
                system.init()
                system.loop(Double.NaN)
                get { value }.describedAs("restart #$i")
                    .isEqualTo("H0[C0[B0[A, E0-, null]], G0[C0[B0[A, E0-, null]], F0[B1[A, E0-, null], D0-]]]")

                repeat(4) {
                    system.loop(Double.NaN)
                    get { value }.describedAs("run #$it")
                        .isEqualTo("H0[C0[B0[A, E0[F1], null]], G0[C0[B0[A, E0[F1], null]], F0[B1[A, E0[F1], null], D0[H0]]]]")
                }
            }
        }
    }

    @Test
    fun `loop no good`() {
        expectCatching {
            buildBlocksSystem {
                val a = testBlock("1", 1, 1, IN_FIRST_ALWAYS)
                val b = testBlock("1", 1, 1)
                val c = testBlock("1", 1, 1)
                val d = testBlock("1", 1, 1)
                a.connectFromAll(b.output(0))
                b.connectFromAll(c.output(0))
                c.connectFromAll(d.output(0))
                d.connectFromAll(a.output(0))
            }
        }.failed().isA<IllegalBlockConfigurationException>()
    }

    @Test
    fun `it actually shuts down`() {
        val monitor: Monitor<Int>
        val externalConstant = ExternalValue(4)
        val system = buildBlocksSystem {

            systemValues.shutdown connectFrom CombineBlock<Int, Int, Boolean> { a, b -> a == b }.apply {
                first connectFrom externalConstant; second connectFrom systemValues.loopNumber
            }

            monitor = systemValues.loopNumber.monitor()
        }
        repeat(10) { i ->
            externalConstant.value = i
            SimpleLoopSystemDriver(system).run()
            expectThat(monitor.value).isEqualTo(i)
        }
    }
}

