@file:Suppress("KDocMissingDocumentation", "SpellCheckingInspection", "PublicApiImplicitType")

val ext = project.rootProject.extra

val xchart: String by ext

plugins {
    kotlin("jvm")
}

dependencies {
    api(project(":all"))
    testImplementation(xchart)
}

extra["publish"] = true
