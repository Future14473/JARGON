package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_ALWAYS
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST_ALWAYS
import org.futurerobotics.jargon.util.forEachZipped
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isEqualTo

internal class TestCompositeBlock : CompositeBlock(2, 2, IN_FIRST_ALWAYS) {
    override fun BlocksConfig.buildSubsystem(
        sources: List<BlocksConfig.Output<Any?>>,
        outputs: List<BlocksConfig.Input<Any?>>
    ) {
        forEachZipped(sources, outputs) { a, b ->
            a into b
        }
    }

    fun input(index: Int) = configInput<Any?>(index)
    fun output(index: Int) = configOutput<Any?>(index)
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
            e.output(0) into pipe.input(0)
            d.output(0) into pipe.input(1)

            b.fromAll(Constant("A"), pipe.output(0))
            c.fromAll(b.output(0))
            d.fromAll(h.output(0))
            e.fromAll(f.output(1))
            f.fromAll(b.output(1), pipe.output(1))
            g.fromAll(c.output(0), f.output(0))
            h.fromAll(c.output(0), g.output(0))
            monitor = d.input().monitor()
        }

        expectThat(monitor) {
            repeat(4) { i ->
                system.init()
                system.loop()
                get { value }.describedAs("restart #$i")
                    .isEqualTo("H0[C0[B0[A, E0-, null]], G0[C0[B0[A, E0-, null]], F0[B1[A, E0-, null], D0-]]]")

                repeat(4) {
                    system.loop()
                    get { value }.describedAs("run #$it")
                        .isEqualTo("H0[C0[B0[A, E0[F1], null]], G0[C0[B0[A, E0[F1], null]], F0[B1[A, E0[F1], null], D0[H0]]]]")
                }
            }
        }
    }
}