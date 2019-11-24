package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST
import org.futurerobotics.jargon.blocks.functional.Monitor
import org.futurerobotics.jargon.system.looping.LoopAsFastAsPossible
import org.futurerobotics.jargon.system.looping.LoopSystemRunner
import org.junit.jupiter.api.Test
import strikt.api.expectCatching
import strikt.api.expectThat
import strikt.assertions.failed
import strikt.assertions.isEqualTo

internal fun BlockArrangementBuilder.testBlock(
    name: String,
    numInputs: Int,
    numOutputs: Int,
    processing: Block.Processing = Block.Processing.LAZY,
    requireAllInputs: Boolean = true
): TestBlock = TestBlock(name, numInputs, numOutputs, processing, requireAllInputs).also { add(it) }

internal fun BlockArrangementBuilder.emptyBlock(
    processing: Block.Processing = Block.Processing.LAZY
) = testBlock("Empty", 0, 0, processing)

internal class BlockSystemTest {
    @Test
    fun `update order test`() {
        val monitor: Monitor<out String?>
        val system = buildBlockSystem {
            val b = testBlock("B", 3, 2, requireAllInputs = false)
            val c = testBlock("C", 1, 2)
            val d = testBlock("D", 1, 1, OUT_FIRST)
            val e = testBlock("E", 1, 1, OUT_FIRST)
            val f = testBlock("F", 2, 2)
            val g = testBlock("G", 2, 1, ALWAYS)
            val h = testBlock("H", 2, 1)

            b.fromAll(constant("A"), e.output(0))
            c.fromAll(b.output(0))
            d.fromAll(h.output(0))
            e.fromAll(f.output(1))
            f.fromAll(b.output(1), d.output(0))
            g.fromAll(c.output(0), f.output(0))
            h.fromAll(c.output(0), g.output(0))
            monitor = d.input(0).source!!.monitor()
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

    @Test
    fun `loop no good`() {
        expectCatching {
            val system = buildBlockSystem {
                val a = testBlock("1", 1, 1, ALWAYS)
                val b = testBlock("1", 1, 1)
                val c = testBlock("1", 1, 1)
                val d = testBlock("1", 1, 1)
                a.fromAll(b.output(0))
                b.fromAll(c.output(0))
                c.fromAll(d.output(0))
                d.fromAll(a.output(0))
            }
            system.start()
            system.loop(0)
        }.failed()
    }

    @Test
    fun `it actually shuts down`() {
        val monitor: Monitor<Int>
        var externalConstant = 0
        val system = buildBlockSystem {
            Shutdown().config { signal from generate { externalConstant == loopNumber } }

            monitor = generate { loopNumber }.monitor().also { add(it) }
        }
        repeat(10) { i ->
            externalConstant = i
            LoopSystemRunner(system, LoopAsFastAsPossible()).run()
            expectThat(monitor.value).isEqualTo(i)
        }
    }
}

