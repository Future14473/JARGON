@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

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
    api(project(":core"))
    api(project(":blocks"))
    api(project(":pathing"))
    api(project(":state-space"))
    testImplementation(junit5)
    testImplementation(junit5params)
    testRuntimeOnly(junit5engine)
    testImplementation(strikt)
    testImplementation(xchart)
    testImplementation(project(":test-util"))

}

tasks.test {
    useJUnitPlatform()
}