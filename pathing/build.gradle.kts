@file:Suppress("KDocMissingDocumentation", "PublicApiImplicitType", "SpellCheckingInspection")

val ext = project.rootProject.extra
val junit: String by ext
val junit5: String by ext
val junit5params: String by ext
val junit5engine: String by ext
val junit5vintage: String by ext
val xchart: String by ext

plugins {
    kotlin("jvm")
    id("org.jetbrains.dokka")
    `maven-publish`
}

dependencies {
    api(project(":core"))

    testImplementation(project(":test-util"))
    testImplementation(junit)
    testImplementation(junit5)
    testImplementation(junit5params)
    testRuntimeOnly(junit5engine)
    testRuntimeOnly(junit5vintage)
    testImplementation(xchart)
}

tasks.test {
    useJUnitPlatform()
}
tasks.named("cleanTest") {
    doLast {
        delete("graphs")
    }
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
