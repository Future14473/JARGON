@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

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
    implementation(xchart)

    testImplementation(junit5)
    testImplementation(junit5params)
    testRuntimeOnly(junit5engine)
    testImplementation(strikt)
    testImplementation(project(":test-util"))

}

tasks.withType<KotlinCompile> {
    kotlinOptions {
        @Suppress("SuspiciousCollectionReassignment")
        freeCompilerArgs += listOf(
            "-Xuse-experimental=kotlin.Experimental" //for contracts
        )
    }
}

tasks.test {
    useJUnitPlatform()
}