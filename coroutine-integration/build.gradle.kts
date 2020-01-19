@file:Suppress("KDocMissingDocumentation", "PublicApiImplicitType", "SpellCheckingInspection")

val ext = project.rootProject.extra
val coroutinesVersion: String by ext

plugins {
    kotlin("jvm")
}
apply(plugin = "kotlinx-atomicfu")

dependencies {
    api(project(":core"))
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-core:$coroutinesVersion")
}

extra["publish"] = true
