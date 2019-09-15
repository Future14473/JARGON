@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation")

val ext = project.rootProject.extra
val junit: String by ext
val koma: String by ext
plugins {
    id("org.jetbrains.dokka")
    `maven-publish`
    kotlin("jvm")
}
dependencies {
    implementation(koma)
    testImplementation(junit)
    testImplementation(project(":test-util"))
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
        create<MavenPublication>("publishLocal") {
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