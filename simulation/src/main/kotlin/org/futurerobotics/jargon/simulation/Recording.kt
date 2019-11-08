package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.config.BCBuilder
import org.futurerobotics.jargon.blocks.functional.Recording
import org.futurerobotics.jargon.math.Vector2d
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.markers.None

private typealias RecordingsGroup<T> = MutableMap<String, Recording<T>>
private typealias RecordingsMap<T> = MutableMap<String, RecordingsGroup<T>>

/**
 * Stores recordings of y or xy values from a system, and eventually can display them on a graph.
 * Recordings can be put in _groups_, and every group corresponds to a graph. Every group/graph can have multiple
 * recordings with different names.
 */
class Recordings internal constructor(
    builder: BCBuilder
) {

    private val xyRecordingsMap: RecordingsMap<Vector2d> = HashMap()
    private val yRecordingsMap: RecordingsMap<Double> = HashMap()
    private val times: Recording<Double>

    init {
        with(builder) {
            times = Recording<Double>()() { input from generate { totalTime } }
        }
    }

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

    /**
     * Adds a new recording for the given double [Block.Output]; in the given [group] and the given [name].
     * This is for a [Recordings]
     *
     * The group and name can later be used to generate a graph with time on the x axis and value on the y.
     */
    fun BCBuilder.recordY(output: Block.Output<Double>, group: String, name: String) {
        val recordingsGroup = yRecordingsMap.getOrPut(group) { HashMap() }
        check(!recordingsGroup.contains(name)) { """group "$group" name "$name" already used!""" }
        recordingsGroup[name] = output.recording()
    }

    /**
     * Adds a new recording for the given [Vector2d] [Block.Output]; in the given [group] and the given [name].
     *
     * The group and name can later be used to generate a graph of the vector values in 2d space.
     */
    fun BCBuilder.recordXY(output: Block.Output<Vector2d>, group: String, name: String) {
        val recordingsGroup = xyRecordingsMap.getOrPut(group) { HashMap() }
        check(!recordingsGroup.contains(name)) { """group "$group" name "$name" already used!""" }
        recordingsGroup[name] = output.recording()
    }
}
