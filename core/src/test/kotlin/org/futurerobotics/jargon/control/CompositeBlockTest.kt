package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.control.Block.Processing.OUT_FIRST_ALWAYS
import org.futurerobotics.jargon.util.forEachZipped
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isEqualTo

internal class TestCompositeBlock : CompositeBlock(2, 2, IN_FIRST_ALWAYS) {
    override fun BlocksConfig.configureSystem(sources: List<BlockOutput<Any?>>, outputs: List<BlockInput<Any?>>) {
        forEachZipped(sources, outputs) { a, b ->
            a connectTo b
        }
    }

    fun input(index: Int) = inputIndex<Any?>(index)
    fun output(index: Int) = outputIndex<Any?>(index)
}


internal class CompositeBlockTest : AbstractBlockSystemTest() {
    @Test
    fun `it works in the middle`() {
        val monitor: Monitor<String>

        val system = buildBlocksSystem {
            val b = testBlock("B", 3, 2, requireAllInputs = false)
            val c = testBlock("C", 1, 2)
            val d = testBlock("D", 1, 1, OUT_FIRST_ALWAYS)
            val e = testBlock("E", 1, 1, OUT_FIRST_ALWAYS)
            val f = testBlock("F", 2, 2)
            val g = testBlock("G", 2, 1, IN_FIRST_ALWAYS)
            val h = testBlock("H", 2, 1)

            val pipe = TestCompositeBlock()
            e.output(0) connectTo pipe.input(0)
            d.output(0) connectTo pipe.input(1)

            b.connectFromAll(Constant("A"), pipe.output(0))
            c.connectFromAll(b.output(0))
            d.connectFromAll(h.output(0))
            e.connectFromAll(f.output(1))
            f.connectFromAll(b.output(1), pipe.output(1))
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
}