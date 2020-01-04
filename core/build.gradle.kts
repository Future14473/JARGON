@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

val ext = project.rootProject.extra
val hipparchus: ((String) -> String) by ext

plugins {
    kotlin("jvm")
}
apply(plugin = "kotlinx-atomicfu")

dependencies {
    api(hipparchus("core"))
}

tasks.named("cleanTest") {
    doLast {
        delete("graphs")
        delete("tmp")
    }
}

//tasks.withType<KotlinCompile> {
//    kotlinOptions {
//        @Suppress("SuspiciousCollectionReassignment")
//        freeCompilerArgs += listOf(
//        )
//    }
//}
extra["publish"] = true
