@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

plugins {
    kotlin("jvm")
}

dependencies {
    api(project(":core"))
    compile(kotlin("reflect"))
}

extra["publish"] = true
