@file:Suppress("KDocMissingDocumentation", "SpellCheckingInspection", "PublicApiImplicitType")

val ext = project.rootProject.extra
val junit5: String by ext
val xchart: String by ext
val strikt: String by ext
val coroutines: String by ext

plugins {
    kotlin("jvm")
}
repositories {
    mavenLocal()
}
dependencies {
    api(project(":core"))
    api(xchart)
    api(strikt)
}
