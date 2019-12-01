package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.ALWAYS
import org.futurerobotics.jargon.blocks.Block.Processing.OUT_FIRST
import org.futurerobotics.jargon.blocks.functional.Monitor
import org.futurerobotics.jargon.running.LoopSystemRunner
import org.futurerobotics.jargon.running.UnregulatedRegulator
import org.junit.jupiter.api.Test
import strikt.api.expectCatching
import strikt.api.expectThat
import strikt.assertions.failed
import strikt.assertions.isEqualTo

internal fun testBlock(
    name: String,
    numInputs: Int,
    numOutputs: Int,
    processing: Block.Processing = Block.Processing.LAZY,
    requireAllInputs: Boolean = true
): TestBlock = TestBlock(name, numInputs, numOutputs, processing, requireAllInputs)

internal fun emptyBlock(
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

            b.fromAll(this, constant("A"), e.output(0))
            c.fromAll(this, b.output(0))
            d.fromAll(this, h.output(0))
            e.fromAll(this, f.output(1))
            f.fromAll(this, b.output(1), d.output(0))
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

    @Test
    fun `loop no good`() {
        expectCatching {
            val system = buildBlockSystem {
                val a = testBlock("1", 1, 1, ALWAYS)
                val b = testBlock("1", 1, 1)
                val c = testBlock("1", 1, 1)
                val d = testBlock("1", 1, 1)
                a.fromAll(this, b.output(0))
                b.fromAll(this, c.output(0))
                c.fromAll(this, d.output(0))
                d.fromAll(this, a.output(0))
            }
            system.start()
            system.loop(0)
        }.failed()
    }

    @Test
    fun `it actually shuts down`() {
        val monitor: Monitor<Int>
        var externalConstant = 4
        val system = buildBlockSystem {
            Shutdown().signal from generate { externalConstant == loopNumber }

            monitor = generate { loopNumber }.monitor()
        }
        repeat(10) { i ->
            externalConstant = i
            LoopSystemRunner(system, UnregulatedRegulator()).run()
            expectThat(monitor.value).isEqualTo(i)
        }
    }
}

