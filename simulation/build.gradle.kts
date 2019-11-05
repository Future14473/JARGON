@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

val ext = project.rootProject.extra

val xchart: String by ext
plugins {
    kotlin("jvm")
}

dependencies {
    api(project(":all"))
    implementation(xchart)
}

tasks.named("cleanTest") {
    doLast {
        delete("graphs")
    }
}

tasks.withType<KotlinCompile> {
    kotlinOptions {
        @Suppress("SuspiciousCollectionReassignment")
        freeCompilerArgs += listOf(
            "-Xuse-experimental=kotlin.Experimental" //for contracts
        )
    }
}
