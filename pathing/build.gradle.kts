@file:Suppress("KDocMissingDocumentation", "PublicApiImplicitType", "SpellCheckingInspection")

val ext = project.rootProject.extra
val junit: String by ext
val junit5vintage: String by ext
val xchart: String by ext

plugins {
    kotlin("jvm")
}

dependencies {
    api(project(":core"))

    testImplementation(junit)
    testRuntimeOnly(junit5vintage)
    testImplementation(xchart)
}

tasks.named("cleanTest") {
    doLast {
        delete("graphs")
    }
}
val publish by extra(true)
