@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

val ext = project.rootProject.extra
val hipparchus: ((String) -> String) by ext

plugins {
    kotlin("jvm")
}

dependencies {
    api(hipparchus("core"))
}

tasks.named("cleanTest") {
    doLast {
        delete("graphs")
        delete("tmp")
    }
}

tasks.withType<KotlinCompile> {
    kotlinOptions {
        @Suppress("SuspiciousCollectionReassignment")
        freeCompilerArgs += listOf(
            "-Xmulti-platform",
            "-Xuse-experimental=kotlin.ExperimentalMultiplatform"
        )
    }
}
extra["publish"] = true
