@file:JvmMultifileClass
@file:JvmName("Paths")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.VectorFunction

/**
 * Something that reparameterizes a [VectorFunction] on th domain from (0,1) to a [ReparamMapping].
 */
interface Reparameterizer {

    /**
     * Reparameterizes the given [function] on the interval (0,1), by arc length, and gets
     * a [ReparamMapping].
     */
    fun getReparamMappingFor(function: VectorFunction): ReparamMapping

    companion object {
        /**
         * The default [Reparameterizer]
         */
        @JvmField
        val DEFAULT: Reparameterizer = IntegrationReparameterizer()
    }
}

/**
 * Reparameterize a [VectorFunction] into a [ReparamCurve] using a the given [reparameterizer].
 */
@JvmOverloads
fun VectorFunction.toCurve(reparameterizer: Reparameterizer = Reparameterizer.DEFAULT): ReparamCurve =
    ReparamCurve(this, reparameterizer)
