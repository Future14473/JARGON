@file:Suppress("PublicApiImplicitType", "KDocMissingDocumentation")

val ext = project.rootProject.extra
val junit: String by ext
val commons: String by ext
plugins {
    kotlin("jvm")
    id("org.jetbrains.dokka")
    `maven-publish`
}
dependencies {
    implementation(commons)
    testImplementation(junit)
    testImplementation(project(":test-util"))
//    implementation( "org.ejml:ejml-all:0.38")
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