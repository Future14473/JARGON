@file:Suppress("KDocMissingDocumentation", "PublicApiImplicitType", "SpellCheckingInspection")

val ext = project.rootProject.extra

val hipparchus: ((String) -> String) by ext

plugins {
    kotlin("jvm")
}

dependencies {
    api(project(":core"))
    api(hipparchus("filtering"))
}
extra["publish"] = true
