@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation", "SpellCheckingInspection")

import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

val ext = project.rootProject.extra
val junit5: String by ext
val junit5engine: String by ext
val junit5params: String by ext
val strikt: String by ext



plugins {
    kotlin("jvm")
    id("org.jetbrains.dokka")
    `maven-publish`
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

tasks.dokka {
    outputFormat = "html"
    outputDirectory = "$buildDir/javadoc"
}
val sourcesJar by tasks.creating(Jar::class) {
    from(sourceSets.main.get().allSource)
    archiveClassifier.set("sources")
}
val dokkaJar by tasks.creating(Jar::class) {
    description = "Assembles Kotlin docs with dokka"
    group = JavaBasePlugin.DOCUMENTATION_GROUP
    archiveClassifier.set("javadoc")
    from(tasks.dokka)
}

publishing {
    publications {
        create<MavenPublication>("publish") {
            from(components["java"])
            artifact(dokkaJar)
            artifact(sourcesJar)
            versionMapping {
                usage("java-api") {
                    fromResolutionOf("runtimeClasspath")
                }
                usage("java-runtime") {
                    fromResolutionResult()
                }
            }
        }
    }
}
