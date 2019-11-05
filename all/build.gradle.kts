@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation")

plugins {
    kotlin("jvm")
}

dependencies {
    api(project(":core"))
    api(project(":blocks"))
    api(project(":pathing"))
    api(project(":state-space"))
}

val publish by extra(true)
