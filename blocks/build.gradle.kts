@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

plugins {
    kotlin("jvm")
}
apply(plugin = "kotlinx-atomicfu")

dependencies {
    api(project(":core"))
    compile(kotlin("reflect"))
}

extra["publish"] = true
