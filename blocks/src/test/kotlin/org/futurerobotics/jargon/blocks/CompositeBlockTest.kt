package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST
import org.futurerobotics.jargon.blocks.functional.Constant
import org.futurerobotics.jargon.blocks.functional.Monitor
import org.futurerobotics.jargon.util.uncheckedCast
import org.futurerobotics.jargon.util.zipForEach
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.isEqualTo

internal class TestCompositeBlock : CompositeBlock(ALWAYS) {
    init {
        repeat(2) {
            newInput<String?>()
            newOutput<String?>()
        }
    }

    override fun SubsystemMapper.configSubsystem(): BlockArrangement = BlockArrangementBuilder().build {
        inputs.zipForEach(outputs) { i, o ->
            i.subOutput into o.subInput.uncheckedCast<Input<Any?>>()
        }
    }

    fun input(index: Int): Input<String?> = inputs[index].uncheckedCast()
    fun output(index: Int): Output<String?> = outputs[index].uncheckedCast()
}

internal class CompositeBlockTest {
    @Test
    fun `it works in the middle`() {
        val monitor: Monitor<out String?>
        val system = buildBlockSystem {
            val b = testBlock("B", 3, 2, requireAllInputs = false)
            val c = testBlock("C", 1, 2)
            val d = testBlock("D", 1, 1, OUT_FIRST)
            val e = testBlock("E", 1, 1, OUT_FIRST)
            val f = testBlock("F", 2, 2)
            val g = testBlock("G", 2, 1, ALWAYS)
            val h = testBlock("H", 2, 1)
            val pipe = TestCompositeBlock()
            e.output(0) into pipe.input(0)
            d.output(0) into pipe.input(1)

            b.fromAll(this, Constant("A").output, pipe.output(0))
            c.fromAll(this, b.output(0))
            d.fromAll(this, h.output(0))
            e.fromAll(this, f.output(1))
            f.fromAll(this, b.output(1), pipe.output(1))
            g.fromAll(this, c.output(0), f.output(0))
            h.fromAll(this, c.output(0), g.output(0))
            monitor = d.input(0).source()!!.monitor()
        }

        expectThat(monitor) {
            repeat(4) { i ->
                system.start()
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
