@file:Suppress("PublicApiImplicitType", "SpellCheckingInspection", "KDocMissingDocumentation")

import org.jetbrains.dokka.gradle.DokkaPlugin
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

//will later be applied to all subprojects
group = "org.futurerobotics.jargon"
version = "0.2.0-SNAPSHOT"
//---
val hipparchusVersion by extra("1.6")
val striktVersion by extra("0.23.2")
val xchartVersion by extra("3.6.0")
val junitVersion by extra("4.12")
val junit5Version by extra("5.5.2")
val coroutinesVersion by extra("1.3.3")

val hipparchus by extra<(String) -> String> { { "org.hipparchus:hipparchus-$it:$hipparchusVersion" } }

val xchart by extra("org.knowm.xchart:xchart:$xchartVersion")
val junit by extra("junit:junit:$junitVersion")
val junit5 by extra("org.junit.jupiter:junit-jupiter-api:$junit5Version")
val junit5params by extra("org.junit.jupiter:junit-jupiter-params:$junit5Version")
val junit5engine by extra("org.junit.jupiter:junit-jupiter-engine:$junit5Version")
val junit5vintage by extra("org.junit.vintage:junit-vintage-engine:$junit5Version")

val strikt by extra("io.strikt:strikt-core:$striktVersion")

buildscript {
    val kotlinVersion by extra("1.3.61")
    val atomicfuVersion by extra("0.14.1")
    repositories {
        mavenCentral()
    }
    dependencies {
        classpath("org.jetbrains.kotlinx:atomicfu-gradle-plugin:$atomicfuVersion")
        classpath(kotlin("gradle-plugin", version = kotlinVersion))
    }
}

repositories {
    mavenCentral()
    jcenter()
}

plugins {
    kotlin("jvm") version "1.3.61"
    id("org.jetbrains.dokka") version "0.10.0"
    `maven-publish`
    id("com.jfrog.bintray") version "1.8.4" apply false
    id("com.jfrog.artifactory") version "4.13.0" apply false
}

subprojects {
    group = rootProject.group
    version = rootProject.version
    repositories {
        mavenCentral()
        jcenter()
    }
    plugins.withId("org.jetbrains.kotlin.jvm") {
        configureKotlin()
        configureTests()
    }

}

fun Project.configureKotlin() {
    dependencies {
        implementation(kotlin("stdlib-jdk8"))
    }
    tasks.withType<KotlinCompile> {
        kotlinOptions {
            jvmTarget = "1.8"
            @Suppress("SuspiciousCollectionReassignment")
            freeCompilerArgs += listOf(
                "-Xuse-experimental=kotlin.Experimental" //for contracts
            )
        }
    }
}

//Tests
fun Project.configureTests() {
    dependencies {
        testImplementation(junit5)
        testImplementation(junit5params)
        testRuntimeOnly(junit5engine)
        testImplementation(strikt)
        val testUtil = ":core:test-util"
        if (parent == rootProject)
            testImplementation(project(testUtil))
    }
    tasks.withType<Test> {
        useJUnitPlatform()
    }
}

tasks.create("testAll") {
    group = "verification"
    dependsOn(
        subprojects.map { it.tasks.withType<Test>() }
    )
}
//publishing

val githubURL = "https://github.com/Future14473/JARGON"

val bintrayUser: String? by extra { System.getenv("BINTRAY_USER") }
val bintrayKey: String? by extra { System.getenv("BINTRAY_KEY") }

//may change later
val isSnapshot = version.toString().contains(Regex("[a-zA-Z]"))

val doPublish = !(bintrayUser == null || bintrayKey == null)
if (!doPublish) {
    logger.warn("Bintray user or key not found; NOT configuring upload publications")
} else {
    if (isSnapshot) {
        apply(from = "configureArtifactory.gradle")
    }
    subprojects {
        if (parent != rootProject) return@subprojects
        apply<MavenPublishPlugin>()
        afterEvaluate {
            configurePublish()
        }
        //if snapshot, artifactory on root, else bintray
        if (isSnapshot) {
            apply(plugin = "com.jfrog.artifactory")
        } else {
            apply(from = rootProject.file("configureBintray.gradle"))
        }
    }
}

fun Project.configurePublish() {
    apply<DokkaPlugin>()
    tasks.dokka {
        outputFormat = "html"
        outputDirectory = "$buildDir/javadoc"
    }
    val dokkaJar by tasks.creating(Jar::class) {
        group = JavaBasePlugin.DOCUMENTATION_GROUP
        archiveClassifier.v = "javadoc"
        from(tasks.dokka)
    }

    val sourcesJar by tasks.creating(Jar::class) {
        from(sourceSets.main.get().allSource)
        archiveClassifier.v = "sources"
    }

    val project = this
    val publicationName = if (isSnapshot) "snapshot" else "release"

    publishing.publications {
        create<MavenPublication>(publicationName) {
            artifactId = "jargon-${project.name}"
            from(components["java"])
            artifact(sourcesJar)
            if (!isSnapshot) {
                artifact(dokkaJar)
            }
            this@configurePublish.configMavenPublication(this)
        }
    }
}

fun Project.configMavenPublication(pub: MavenPublication) {
    pub.versionMapping {
        usage("java-api") {
            fromResolutionOf("runtimeClasspath")
        }
        usage("java-runtime") {
            fromResolutionResult()
        }
    }
    val project = this
    pub.pom {
        name.v = "JARGON ${project.name.replace("-", " ")}"
        description.v = project.description
        url.v = githubURL
        licenses {
            license {
                name.v = "The MIT License"
                url.v = "http://www.opensource.org/licenses/MIT"
                distribution.v = "repo"
            }
        }
        scm {
            url.v = githubURL
        }
    }
}

var <T> Property<T>.v: T?
    get() = get()
    set(value) {
        set(value)
    }
