package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.pathing.Curve
import org.futurerobotics.jargon.pathing.HeadingProvider
import org.futurerobotics.jargon.pathing.Path
import org.futurerobotics.jargon.pathing.PathPoint

/**
 * Provider for a [HeadingProvider], that can first get info from the previous [Path] and current [Curve], so as
 * to "continue" the previous path.
 *
 * Not to be confused with [HeadingProvider].
 *
 * @see HeadingProvider
 */
interface ContinuationHeadingProvider {

    /**
     * Gets a [HeadingProvider] given the [lastPoint] of the previous path, and the curCurve [curve], the
     * heading will be given to.
     */
    fun getHeadingProvider(lastPoint: PathPoint, curve: Curve): HeadingProvider
}
