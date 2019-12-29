@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

plugins {
    kotlin("jvm")
}
apply(plugin = "kotlinx-atomicfu")

dependencies {
    api(project(":core"))
    api(project(":state-space"))
    compile(kotlin("reflect"))
}

extra["publish"] = true
