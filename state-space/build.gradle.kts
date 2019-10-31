@file:Suppress("KDocMissingDocumentation", "PublicApiImplicitType", "SpellCheckingInspection")

val ext = project.rootProject.extra

val hipparchus: ((String) -> String) by ext
val junit5: String by ext
val junit5engine: String by ext
val strikt: String by ext

plugins {
    kotlin("jvm")
    id("org.jetbrains.dokka")
    `maven-publish`
}

dependencies {
    api(project(":core"))
    api(project(":blocks"))
    implementation(hipparchus("filtering"))
    testImplementation(junit5)
    testRuntimeOnly(junit5engine)
    testImplementation(project(":test-util"))
    testImplementation(strikt)

}
tasks.test {
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
//            artifact(dokkaJar)
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

