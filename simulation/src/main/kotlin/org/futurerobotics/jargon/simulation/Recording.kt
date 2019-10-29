package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.BaseBlocksConfig
import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.BlocksSystem
import org.futurerobotics.jargon.blocks.SystemValuesBlock
import org.futurerobotics.jargon.blocks.functional.RecordingBlock
import org.futurerobotics.jargon.math.Vector2d
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.markers.None
import kotlin.collections.set
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract

private typealias RecordingsGroup<T> = MutableMap<String, RecordingBlock<T>>
private typealias RecordingsMap<T> = MutableMap<String, RecordingsGroup<T>>

/**
 * Stores recordings of y or xy values from a system, and eventually can display them on a graph.
 * Recordings can be put in _groups_, and every group corresponds to a graph. Every group/graph can have multiple
 * recordings with different names.
 *
 * Obtained with [RecordingBlocksConfig].
 */
class Recordings internal constructor(
    private val yRecordingsMap: RecordingsMap<Double>,
    private val xyRecordingsMap: RecordingsMap<Vector2d>,
    private val times: RecordingBlock<Double>
) {
    /**
     * Gets a [XYChart] for the given [groupName] of y recordings. Time will be graphed on the x-axis, and values on the
     * y-axis.
     *
     * Uses the given [builder] to create the base graph.
     */
    fun getYGraph(groupName: String, builder: XYChartBuilder = XYChartBuilder()): XYChart {
        require(groupName in yRecordingsMap) { "Group $groupName does not exist" }
        return builder.build().apply {
            title = groupName
            xAxisTitle = "Time"
            yRecordingsMap.getValue(groupName).forEach { (name, recording) ->
                addSeries(name, times.values, recording.values).apply {
                    marker = None()
                }
            }
        }
    }

    /**
     * Gets a list of pairs of (group name, [XYChart]) of y recordings for all groups of y recordings.
     * Time will be graphed on the x-axis, and values on the y-axis.
     *
     * Uses the given [builder] to create the base graph.
     */
    fun getAllYGraphs(builder: XYChartBuilder = XYChartBuilder()): List<Pair<String, XYChart>> =
        yRecordingsMap.map { (groupName, _) -> groupName to getYGraph(groupName, builder) }

    /**
     * Gets a [XYChart] for the given [groupName] of xy recordings. The values of xy will be plotted and connected.
     *
     * Uses the given [builder] to create the base graph.
     */
    fun getXYGraph(groupName: String, builder: XYChartBuilder = XYChartBuilder()): XYChart {
        require(groupName in xyRecordingsMap) { "Group $groupName does not exist" }
        return builder.build().apply {
            title = groupName
            xyRecordingsMap.getValue(groupName).forEach { (name, recording) ->
                addSeries(name, recording.values.map { it.x }, recording.values.map { it.y }).apply {
                    marker = None()
                }
            }
        }
    }

    /**
     * Gets a list of pairs of (group name, [XYChart]) of xy recordings for all groups of xy recordings.
     * The values of xy will be plotted and connected.
     *
     * Uses the given [builder] to create the base graph.
     */
    fun getAllXYGraphs(builder: XYChartBuilder = XYChartBuilder()): List<Pair<String, XYChart>> =
        xyRecordingsMap.map { (groupName, _) -> groupName to getXYGraph(groupName, builder) }

    /**
     * Gets a list of pairs of (group name, [XYChart]) for _all_ groups, y graphs and xy graphs.
     */
    fun getAllGraphs(): List<Pair<String, XYChart>> = getAllXYGraphs() + getAllYGraphs()
}

/**
 * A [BlocksConfig] that provides additional DSL functions for recording values of blocks.
 */
class RecordingBlocksConfig : BaseBlocksConfig() {
    private val yRecordingsMap: RecordingsMap<Double> = HashMap()
    private val xyRecordingsMap: RecordingsMap<Vector2d> = HashMap()

    /**
     * The [Recordings] that are maintained by this [RecordingBlocksConfig]. Any newly added recordings will
     * be reflected in this Records.
     */
    val recordings: Recordings

    init {
        val times = RecordingBlock<Double>()() { this from SystemValuesBlock().totalTime }
        recordings = Recordings(yRecordingsMap, xyRecordingsMap, times)
    }

    /**
     * Adds a new recording for the given double [BlocksConfig.Output]; in the given [group] and the given [name].
     * This is for a [Recordings]
     *
     * The group and name can later be used to generate a graph with time on the x axis and value on the y.
     */
    fun Output<Double>.recordY(group: String, name: String) {
        val recordingsGroup = yRecordingsMap.getOrPut(group) { HashMap() }
        check(!recordingsGroup.contains(name)) { """group "$group" name "$name" already used!""" }
        recordingsGroup[name] = RecordingBlock<Double>().also { this into it }
    }

    /**
     * Adds a new recording for the given [Vector2d] [BlocksConfig.Output]; in the given [group] and the given [name].
     *
     * The group and name can later be used to generate a graph of the vector values in 2d space.
     */
    fun Output<Vector2d>.recordXY(group: String, name: String) {
        val recordingsGroup = xyRecordingsMap.getOrPut(group) { HashMap() }
        check(!recordingsGroup.contains(name)) { """group "$group" name "$name" already used!""" }
        recordingsGroup[name] = RecordingBlock<Vector2d>().also { this into it }
    }
}

/**
 * DSL to build a block system.
 * Runs the [configuration] block on a [RecordingBlocksConfig] then returns the built [BlocksSystem] and [Recordings].
 */
@UseExperimental(ExperimentalContracts::class)
inline fun buildRecordingBlocksSystem(
    configuration: RecordingBlocksConfig.() -> Unit
): Pair<BlocksSystem, Recordings> {
    contract {
        callsInPlace(configuration, InvocationKind.EXACTLY_ONCE)
    }
    return RecordingBlocksConfig().run {
        configuration()
        verifyConfig()
        BlocksSystem(this) to recordings
    }
}
