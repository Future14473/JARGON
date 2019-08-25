package org.futurerobotics.temporaryname.util

import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicReference

/**
 * The value held by this AtomicInteger, as a property.
 */
inline var AtomicInteger.value: Int
    get() = get()
    set(value) = set(value)
/**
 * The value held by this AtomicBoolean, as a property.
 */
inline var AtomicBoolean.value: Boolean
    get() = get()
    set(value) = set(value)
/**
 * The value held by this AtomicReference, as a property.
 */
inline var <V> AtomicReference<V>.value: V
    get() = get()
    set(value) = set(value)