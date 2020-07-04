package org.futurerobotics.jargon.util

import java.util.*

/** Wraps this list with [Collections.unmodifiableList]. */
fun <T> List<T>.asUnmodifiableList(): List<T> = Collections.unmodifiableList(this)

/** Wraps this map with [Collections.unmodifiableMap]. */
fun <T, R> Map<T, R>.asUnmodifiableMap(): Map<T, R> = Collections.unmodifiableMap(this)

/** Wraps this set with [Collections.unmodifiableSet]. */
fun <T> Set<T>.asUnmodifiableSet(): Set<T> = Collections.unmodifiableSet(this)

/** Wraps this collection with [Collections.unmodifiableCollection] */
fun <T> Collection<T>.asUnmodifiableCollection(): Collection<T> = Collections.unmodifiableCollection(this)

