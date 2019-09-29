package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.InOutOrder.OUT_FIRST
import org.futurerobotics.jargon.control.Block.Processing.ALWAYS
import org.futurerobotics.jargon.system.SimpleLoopSystemDriver
import org.junit.jupiter.api.Test
import strikt.api.expectCatching
import strikt.api.expectThat
import strikt.assertions.failed
import strikt.assertions.isA
import strikt.assertions.isEqualTo

internal abstract class AbstractBlockSystemTest {
    protected lateinit var monitor: Monitor<*>
    protected fun BlocksBuilder.testBlock(
        name: String,
        numInputs: Int,
        numOutputs: Int,
        inOutOrder: Block.InOutOrder = Block.InOutOrder.IN_FIRST,
        processing: Block.Processing = Block.Processing.LAZY
    ): BlocksBuilder.Block {
        return add(TestBlock(name, numInputs, numOutputs, inOutOrder, processing))
    }

    protected fun BlocksBuilder.emptyBlock(
        inOutOrder: Block.InOutOrder = Block.InOutOrder.IN_FIRST,
        processing: Block.Processing = Block.Processing.LAZY
    ) = testBlock("Empty", 0, 0, inOutOrder, processing)
}

internal class BlockSystemTest : AbstractBlockSystemTest() {

    @Test
    fun `update order test`() {
        val system = buildBlockSystem {
            val b = testBlock("B", 2, 2)
            val c = testBlock("C", 1, 2)
            val d = testBlock("D", 1, 1, OUT_FIRST, ALWAYS)
            val e = testBlock("E", 1, 1, OUT_FIRST)
            val f = testBlock("F", 2, 2)
            val g = testBlock("G", 2, 1)
            val h = testBlock("H", 2, 1)

            b.connectAll("A".constant(), e(0))
            c.connectAll(b<Any?>(0).pipe { "p{$it}" })
            d.connectAll(h(0))
            e.connectAll(f(1))
            f.connectAll(b(1), d(0))
            g.connectAll(c(0), f(0))
            h.connectAll(c(0), g(0))
            monitor = d.input<String>().monitor()
        }

        expectThat(monitor) {
            repeat(4) { i ->
                system.init()
                system.loop(Double.NaN)
                get { value }.describedAs("restart #$i")
                    .isEqualTo("H0[C0[p{B0[A, E0-]}], G0[C0[p{B0[A, E0-]}], F0[B1[A, E0-], D0-]]]")

                repeat(4) {
                    system.loop(Double.NaN)
                    get { value }.describedAs("run #$it")
                        .isEqualTo("H0[C0[p{B0[A, E0[F1]]}], G0[C0[p{B0[A, E0[F1]]}], F0[B1[A, E0[F1]], D0[H0]]]]")
                }
            }
        }
    }

    @Test
    fun `loop no good`() {
        expectCatching {
            buildBlockSystem {
                val a = testBlock("1", 1, 1)
                val b = testBlock("1", 1, 1)
                val c = testBlock("1", 1, 1)
                val d = testBlock("1", 1, 1)
                a.connectAll(b())
                b.connectAll(c())
                c.connectAll(d())
                d.connectAll(a())
                a.input<Any>().monitor()
            }
        }.failed().isA<IllegalBlockConfigurationException>()
    }

    @Test
    fun `it actually shuts down`() {
        val monitor: Monitor<Int>
        val externalConstant = ExternalInput(4)
        val system = buildBlockSystem {

            val externalIn = externalConstant.add().output<Int>()

            shutdown connectFrom
                    combine(loopNumber, externalIn) { a, b -> a == b }

            monitor = loopNumber.monitor()
        }
        repeat(10) { i ->
            externalConstant.value = i
            SimpleLoopSystemDriver(system).run()
            expectThat(monitor.value).isEqualTo(i)
        }
    }
}

