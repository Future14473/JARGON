package org.futurerobotics.jargon.pathing.graph

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.pathing.*
import org.futurerobotics.jargon.pathing.graph.PathGraph.Edge
import org.futurerobotics.jargon.pathing.graph.PathGraph.Node
import org.futurerobotics.jargon.util.builder
import org.futurerobotics.jargon.util.replaceIf
import org.futurerobotics.jargon.util.uncheckedCast
import org.futurerobotics.jargon.util.zipWithNextTo
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.PI

/**
 * Represents a graph paths, using [Node]s (at specific points) connected with [Edge]s (paths between
 * those locations).
 *
 * Then, this will use good ol' dijkstra's algorithm to find a path between to nodes, and use that to generate paths.
 *
 * Nodes can also be given names for later retrieval. Nodes can be named directly using an edge, and it will
 * name the "ending" node. This is useful in builder usage patterns.
 *
 * Edges have a 'forward/backward' direction depending on the order they were connected from. Edges can optionally
 * have incoming and outgoing [WaypointConstraint]s, forward/backward weights, and edges that represent curves only must
 * be supplied with a [HeadingInterpolator] (or else a [defaultInterpolator] will be used). If a path requires
 * the bot to go from forward to backward (a point_turn will be required), then the [Node.turnAroundWeight] is added
 * to the cost.
 *
 * By default, when traversing an edge backwards, the heading will stay the same. This is the only option for now
 *
 * This is so in the end you can say "Get me from point 'A' to 'B' and it figures out a path for you.
 *
 * @property defaultInterpolator The default heading interpolator to use for curves, if none is supplied.
 */
class PathGraph
@JvmOverloads constructor(
    var defaultInterpolator: HeadingInterpolator = TangentInterpolator
) {

    private val nodes = mutableListOf<Node>()
    private val nodesByName = hashMapOf<String, Node>()
    /** Gets a node with the given [name], or `null` if it does not exist. */
    fun getNodeOrNull(name: String): Node? = nodesByName[name]

    /** Gets a node with the given [name], and throws an exception if it does not exist. */
    fun getNode(name: String): Node =
        getNodeOrNull(name) ?: throw IllegalArgumentException("Node with name $name does not exist.")

    /** Creates a node with at the given [location]. */
    fun newNode(location: Vector2d): Node = Node(location)

    /** Creates a node with at the given ([x], [y]) location */
    fun newNode(x: Double, y: Double): Node = newNode(Vector2d(x, y))

    /**
     * Attempts to create a path between the node named [start] and the node name [end], using the given
     * [curveGenParams], or `null` if not possible.
     */
    @JvmOverloads
    fun getPath(start: String, end: String, curveGenParams: CurveGenParams = CurveGenParams.DEFAULT): Path? =
        getPaths(start, end, curveGenParams)?.let { multiplePath(it) }

    /**
     * Attempts to create a path between the node named [start] and the node name [end], using the given
     * [curveGenParams], or `null` if not possible.
     */
    @JvmOverloads
    fun getPaths(start: String, end: String, curveGenParams: CurveGenParams = CurveGenParams.DEFAULT): List<Path>? {
        val edges = dijkstras(getNode(start).index, getNode(end).index)?.takeIf { it.isNotEmpty() } ?: return null
        return createPaths(edges, curveGenParams)
    }

    //rather ad-hoc...
    private fun createPaths(edges: List<EdgeTraversal>, curveGenParams: CurveGenParams): List<Path> {
        //in waypoints have priority.
        val waypoints = ArrayList<Waypoint>(edges.size)
        waypoints += edges.first().inner.inWaypoint

        edges.zipWithNextTo(waypoints) { edge1, edge2 ->
            edge1.inner.outWaypoint.mergeFrom(edge2.inner.inConstraint)
        }
        waypoints += edges.last().inner.outWaypoint

        //create paths/curves.
        val curSplineWaypoints = arrayListOf(waypoints[0])
        val genericPaths = ArrayList<GenericPath<*>>(edges.size)

        edges.forEachIndexed { i, edge ->
            val inner = edge.inner
            if (inner !is FreeEdge) {
                //Only FreeEdges get curves, so turn any queued splines into curves.
                if (curSplineWaypoints.size > 1)
                    genericPaths += heuristicSplineCurves(curSplineWaypoints, curveGenParams)
                curSplineWaypoints.clear()
                //is either curve or path edge
                genericPaths += inner.path
                    ?: throw NotImplementedError("Edge type ${inner.javaClass} is not implemented")
            }


            curSplineWaypoints += waypoints[i + 1]
        }
        //fencepost
        if (curSplineWaypoints.size > 1)
            genericPaths.addAll(heuristicSplineCurves(curSplineWaypoints, curveGenParams))

        return genericPaths.mapIndexed { i, genPath ->
            if (genPath is Path) genPath
            else {
                val curve = genPath as Curve
                val edge = edges[i]
                val interpolator = (edge.inner as EdgeWithHeading).interpolator

                val startHeading = waypoints[i].constraint.let { (direction, heading) ->
                    heading ?: (direction ?: curve.startPoint().tanAngle).replaceIf(edge.isBackwards) {
                        angleNorm(it + PI)
                    }
                }
                val endHeading = waypoints[i + 1].constraint.let { (direction, heading) ->
                    heading ?: (direction ?: curve.endPoint().tanAngle).replaceIf(edge.isBackwards) {
                        angleNorm(it + PI)
                    }
                }

                curve.addHeading(interpolator, startHeading, endHeading)
            }
        }
    }

    /** Edge including _backwards heading_ of traversal. */
    internal class EdgeTraversal(val inner: Edge<*>, val isBackwards: Boolean)

    private fun dijkstras(start: Int, goal: Int): List<EdgeTraversal>? {
        if (start == goal) return emptyList()
        val numNodes = nodes.size

        data class MarkedNode(val nodeIndex: Int, val distance: Int, val backwards: Boolean) {

            val visIndex get() = nodeIndex.replaceIf(backwards) { it + numNodes }
        }

        var found = false

        val distances = IntArray(nodes.size * 2) { Int.MAX_VALUE }
        val visited = BooleanArray(nodes.size * 2)
        val from = arrayOfNulls<EdgeTraversal>(nodes.size)

        val queue = PriorityQueue<MarkedNode>(compareBy { it.distance })
        queue += MarkedNode(start, 0, false)
        queue += MarkedNode(start, 0, true)
        while (queue.isNotEmpty()) {
            val next = queue.remove()
            val (nodeIndex, dist, backwards) = next
            val visIndex = next.visIndex
            if (visited[visIndex]) continue
            visited[visIndex] = true
            if (nodeIndex == goal) { //found?
                found = true
                break
            }
            //expand
            nodes[nodeIndex].edges.forEach { edge ->
                assert(edge.fromNode.index == nodeIndex)
                val nextNodeIndex = edge.toNode.index
                val nextVisIndex = nextNodeIndex.replaceIf(backwards) { it + numNodes }
                if (visited[nextVisIndex]) return@forEach

                if (edge.isDirectional && backwards != edge.isReversed) return@forEach //need turnaround.

                val weight = edge.weight.also {
                    if (it == Int.MAX_VALUE) return@forEach
                }

                val nextDist = (dist.toLong() + weight).also {
                    if (it !in Int.MIN_VALUE until Int.MAX_VALUE) throw ArithmeticException("Integer overflow")
                }.toInt()
                if (nextDist < distances[nextVisIndex]) {
                    from[nextNodeIndex] = EdgeTraversal(edge, backwards)
                    distances[nextVisIndex] = nextDist
                    queue += MarkedNode(nextNodeIndex, nextDist, backwards)
                }
            }
            //also expand backwards.
            val reverseVisIndex = nodeIndex.replaceIf(!backwards) { it + numNodes }
            if (!visited[reverseVisIndex]) {
                visited[reverseVisIndex] = true
                val nextDist = (dist.toLong() + nodes[nodeIndex].turnAroundWeight).also {
                    if (it !in Int.MIN_VALUE until Int.MAX_VALUE) throw ArithmeticException("Integer overflow")
                }.toInt()
                if (nextDist < distances[reverseVisIndex]) {
                    distances[reverseVisIndex] = nextDist
                    queue += MarkedNode(nodeIndex, nextDist, !backwards)
                }
            }
        }
        if (!found) return null

        var cur = goal
        val list = mutableListOf<EdgeTraversal>()
        while (cur != start) {
            val edge = from[cur]!!
            list += edge
            cur = edge.inner.fromNode.index
        }
        list.reverse()
        return list
    }

    /** Represents a node or the end of the node where you can continue adding edges. */
    interface NodeContinuation<Return> {

        /**
         * The name of this node, or `null` if none. Must be unique, and can only be set once.
         *
         * Can also be set via [setName] builder function.
         */
        var name: String?
        /** Sets the weight assigned when the bot needs to go from forward to reverse at this node. */
        var turnAroundWeight: Int

        /** Builder method to set [name] */
        @JvmDefault
        fun setName(name: String): Return = builder {
            this.name = name
        }

        /** Builder method to set [turnAroundWeight] */
        @JvmDefault
        fun setTurnAroundWeight(weight: Int): Return = builder {
            this.turnAroundWeight = weight
        }

        /** Connects this node to the given [node], then returns the resulting [FreeEdge]. */
        fun to(node: Node): FreeEdge

        /** Connects this node to the node with the given [name], then returns the resulting [FreeEdge]. */
        fun to(name: String): FreeEdge

        /** Connects this node to a new node with the given [location], then returns the resulting [FreeEdge]. */
        fun to(location: Vector2d): FreeEdge

        /**
         * Connects this node to a new node with the given ([x], [y]) location, then returns the resulting
         * [FreeEdge].
         */
        fun to(x: Double, y: Double): FreeEdge

        /** Connects this node to the given [waypoint], with its constraints, then returns the resulting [FreeEdge]. */
        fun to(waypoint: Waypoint): FreeEdge

        /** Connects this node to the given [node] using a line. */
        fun lineTo(node: Node): CurveEdge

        /** Connects this node to the node with the given [name] using a line. */
        fun lineTo(name: String): CurveEdge

        /** Connects this node to a new node at the given [location] with a line. */
        fun lineTo(location: Vector2d): CurveEdge

        /** Connects this node to a new node at the given ([x], [y]) location, using a line. */
        fun lineTo(x: Double, y: Double): CurveEdge

        /**
         * Adds a [curve] as an edge, creating a new node at the end of the curve, and which must start
         * from this node.
         */
        fun addCurve(curve: Curve): CurveEdge

        /**
         * Adds a [path] as an edge, creating a new node at the end of the path, and which must start
         * from this node.
         */
        fun addPath(path: Path): PathEdge
    }

    /**
     * A [Node] in a [PathGraph]. Has a specific [position], and can also have a [name].
     *
     * This can then be connected to other nodes via the many given functions, mainly [to].
     *
     * @property position the position of this node.
     */
    inner class Node internal constructor(val position: Vector2d) : NodeContinuation<Node> {

        internal val index: Int = nodes.size
        internal val edges = hashSetOf<Edge<*>>()

        override var name: String? = null
            set(value) {
                check(value != null) { "Cannot set name to null." }
                require(field == null) {
                    """This node has already been named "$field", cannot set to "$value."""
                }
                require(value !in nodesByName) { """Name "$value" already used.""" }
                nodesByName[value] = this
                field = value
            }

        override var turnAroundWeight: Int = DEFAULT_TURNAROUND_WEIGHT

        init {
            nodes += this
        }

        override fun to(node: Node): FreeEdge = FreeEdge(this, node, defaultInterpolator)
        override fun to(name: String): FreeEdge = to(getNode(name))
        override fun to(location: Vector2d): FreeEdge = to(Node(location))
        override fun to(x: Double, y: Double): FreeEdge = to(Vector2d(x, y))
        override fun to(waypoint: Waypoint): FreeEdge = to(waypoint.position).addOutConstraint(waypoint.constraint)
        override fun lineTo(node: Node): CurveEdge =
            CurveEdge(this, node, Line(position, node.position), defaultInterpolator)

        override fun lineTo(name: String): CurveEdge = lineTo(getNode(name))
        override fun lineTo(location: Vector2d): CurveEdge = lineTo(Node(location))
        override fun lineTo(x: Double, y: Double): CurveEdge = lineTo(Vector2d(x, y))
        override fun addCurve(curve: Curve): CurveEdge {
            require(position epsEq curve.startPoint().position) { "Start position must match." }
            return CurveEdge(this, Node(curve.endPoint().position), curve, defaultInterpolator)
        }

        override fun addPath(path: Path): PathEdge {
            require(position epsEq path.startPoint().position) { "Start position must match." }
            return PathEdge(this, Node(path.endPoint().position), path)
        }
    }

    /**
     * Represents an edge between two nodes, with continuation to continue adding edges.
     *
     * This is for more idiomatic building.
     *
     * @property fromNode the from/start node of this edge.
     * @property toNode the from/start node of this edge.
     */
    abstract class Edge<Self : Edge<Self>>
    internal constructor(continuation: Node, val fromNode: Node, val toNode: Node) :
        NodeContinuation<Self> by continuation.uncheckedCast() {

        /** The weight of this edge. */
        var weight: Int = DEFAULT_WEIGHT

        /** Builder method to set [weight]. */
        fun setWeight(weight: Int): Self = builder {
            this.weight = weight
        }

        /** Sets both this's [weight] and [reverse]'s [weight]. */
        fun setWeights(weight: Int): Self = builder {
            this.weight = weight
            reverse.weight = weight
        }

        /** The constraint applied on the [fromNode], when incoming this edge. */
        var inConstraint: WaypointConstraint = WaypointConstraint.NONE
            private set

        /** The constraint applied on the [toNode], when outgoing this edge. */
        var outConstraint: WaypointConstraint = WaypointConstraint.NONE
            private set

        /** Adds additional constraints to the [inConstraint]. */
        fun addInConstraint(constraint: WaypointConstraint): Self = builder {
            inConstraint = inConstraint.mergeFrom(constraint)
        }

        /** Adds additional constraints to the [outConstraint]. */
        fun addOutConstraint(constraint: WaypointConstraint): Self = builder {
            outConstraint = outConstraint.mergeFrom(constraint)
        }

        /** Adds additional constraints to both [inConstraint] and the [reverse]'s [outConstraint]. */
        fun addStartConstraints(constraint: WaypointConstraint): Self = builder {
            addInConstraint(constraint)
            reverse.addOutConstraint(constraint.reverseDirection())
        }

        /** Adds additional constraints to both [outConstraint] and the [reverse]'s [inConstraint]. */
        fun addEndConstraints(constraint: WaypointConstraint): Self = builder {
            addOutConstraint(constraint)
            reverse.addInConstraint(constraint.reverseDirection())
        }

        internal val inWaypoint: Waypoint get() = Waypoint(fromNode.position, inConstraint)
        internal val outWaypoint: Waypoint get() = Waypoint(toNode.position, outConstraint)

        /** The corresponding reverse edge. */
        lateinit var reverse: Self
            protected set
        internal var isReversed: Boolean = false

        internal val isDirectional get() = inConstraint.heading != null || outConstraint.heading != null

        internal fun setup(reverse: Self) {
            this.reverse = reverse
            reverse.reverse = this.uncheckedCast()

            reverse.isReversed = true

            fromNode.edges += this
            toNode.edges += reverse
        }
    }

    /**
     * A [EdgeWithHeading] that has an [HeadingInterpolator].
     *
     * @property interpolator The [HeadingInterpolator] used for this [EdgeWithHeading].
     */
    abstract class EdgeWithHeading<Self : EdgeWithHeading<Self>>
    internal constructor(
        continuation: Node, fromNode: Node, toNode: Node, var interpolator: HeadingInterpolator
    ) : Edge<Self>(continuation, fromNode, toNode)

    /** An edge that will use splines to connect points, using [heuristicSplineCurves]. */
    class FreeEdge private constructor(
        continuation: Node,
        fromNode: Node,
        toNode: Node,
        interpolator: HeadingInterpolator
    ) : EdgeWithHeading<FreeEdge>(continuation, fromNode, toNode, interpolator) {

        constructor(
            fromNode: Node,
            toNode: Node,
            interpolator: HeadingInterpolator
        ) : this(toNode, fromNode, toNode, interpolator) {
            setup(FreeEdge(toNode, toNode, fromNode, interpolator))
        }
    }

    /**
     * A edge that uses a [curve] to connect nodes.
     *
     * @property curve the [Curve] of this [CurveEdge]
     */
    class CurveEdge private constructor(
        continuation: Node, fromNode: Node, toNode: Node,
        val curve: Curve, interpolator: HeadingInterpolator
    ) : EdgeWithHeading<CurveEdge>(continuation, fromNode, toNode, interpolator) {

        constructor(
            fromNode: Node,
            toNode: Node,
            curve: Curve,
            interpolator: HeadingInterpolator
        ) : this(toNode, fromNode, toNode, curve, interpolator) {
            setup(CurveEdge(toNode, toNode, fromNode, curve.reversed(), interpolator))
        }
    }

    /**
     * A edge that uses a [path] to connect nodes.
     *
     * @property path the [Path] of this [PathEdge]
     */
    class PathEdge private constructor(
        continuation: Node, fromNode: Node, toNode: Node,
        /** The path of this [PathEdge]. */
        val path: Path
    ) : Edge<PathEdge>(continuation, fromNode, toNode) {

        constructor(
            fromNode: Node,
            toNode: Node,
            path: Path
        ) : this(toNode, fromNode, toNode, path) {
            setup(PathEdge(toNode, toNode, fromNode, path.reversed()))
        }
    }

    private val Edge<*>.path: GenericPath<*>?
        get() = when (this) {
            is CurveEdge -> curve
            is PathEdge -> path
            else -> null
        }

    companion object {
        /** The default weight assigned to edges (128). */
        const val DEFAULT_WEIGHT: Int = 128
        /** The default weight assigned to node turnaround (256). */
        const val DEFAULT_TURNAROUND_WEIGHT: Int = 256
    }
}
