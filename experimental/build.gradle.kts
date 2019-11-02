@file:Suppress("KDocMissingDocumentation", "SpellCheckingInspection", "PublicApiImplicitType")

val ext = project.rootProject.extra

val junit5: String by ext
val junit5params: String by ext
val junit5engine: String by ext
val strikt: String by ext
val xchart: String by ext

plugins {
    kotlin("jvm")
}

dependencies {
    api(project(":all"))

    testImplementation(junit5)
    testImplementation(junit5params)
    testRuntimeOnly(junit5engine)
    testImplementation(project(":test-util"))
    testImplementation(strikt)
    testImplementation(xchart)
}
tasks.test {
    useJUnitPlatform()
}
