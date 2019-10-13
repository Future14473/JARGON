@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

val ext = project.rootProject.extra
val hipparchus: ((String) -> String) by ext
val junit5: String by ext
val junit5engine: String by ext
val junit5params: String by ext
val strikt: String by ext



plugins {
    kotlin("jvm")
}

dependencies {
    api(project(":core"))

    testImplementation(junit5)
    testImplementation(junit5params)
    testRuntimeOnly(junit5engine)
    testImplementation(project(":test-util"))
    testImplementation(strikt)
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
    dependsOn("cleanTest")
    useJUnitPlatform()
}
