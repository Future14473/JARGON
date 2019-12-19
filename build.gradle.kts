@file:Suppress("PublicApiImplicitType", "SpellCheckingInspection", "KDocMissingDocumentation")

import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

val hipparchusVersion by extra("1.5")
val striktVersion by extra("0.22.2")
val xchartVersion by extra("3.5.4")
val junitVersion by extra("4.12")
val junit5Version by extra("5.5.2")

val dokka by extra("org.jetbrains.dokka")

val hipparchus by extra<(String) -> String> { { "org.hipparchus:hipparchus-$it:$hipparchusVersion" } }

val coroutines by extra("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.3.2")

val xchart by extra("org.knowm.xchart:xchart:$xchartVersion")
val junit by extra("junit:junit:$junitVersion")
val junit5 by extra("org.junit.jupiter:junit-jupiter-api:$junit5Version")
val junit5params by extra("org.junit.jupiter:junit-jupiter-params:$junit5Version")
val junit5engine by extra("org.junit.jupiter:junit-jupiter-engine:$junit5Version")
val junit5vintage by extra("org.junit.vintage:junit-vintage-engine:$junit5Version")

val strikt by extra("io.strikt:strikt-core:$striktVersion")

buildscript {
    val kotlinVersion by extra("1.3.50")
    repositories {
        mavenCentral()
    }
    dependencies {
        classpath(kotlin("gradle-plugin", version = kotlinVersion))
    }
}

repositories {
    mavenCentral()
    jcenter()
}

plugins {
    kotlin("jvm") version "1.3.50"
    id("org.jetbrains.dokka") version "0.9.18"
    `maven-publish`
}
subprojects {
    group = "org.futurerobotics.jargon"
    version = "0.1.0-SNAPSHOT"
    repositories {
        mavenCentral()
        jcenter()
    }
    plugins.withId("org.jetbrains.kotlin.jvm") {
        configureKotlin()
        configureTests()
    }
    afterEvaluate {
        if (extra.has("publish") && extra["publish"] == true) {
            configurePublish()
        }
    }
}

fun Project.configureKotlin() {
    dependencies {
        implementation(kotlin("stdlib-jdk8"))
    }
    tasks.withType<KotlinCompile> {
        kotlinOptions {
            jvmTarget = "1.8"
        }
    }
}

//workaround issues with apply from kts
fun Project.configureTests() {
    dependencies {
        testImplementation(junit5)
        testImplementation(junit5params)
        testRuntimeOnly(junit5engine)
        testImplementation(strikt)
        val testUtil = "test-util"
        if (name != testUtil)
            testImplementation(project(":$testUtil"))
    }
    tasks.withType<Test> {
        useJUnitPlatform {
        }
    }
}

fun Project.configurePublish() {
    apply(plugin = "org.gradle.maven-publish")
    apply(plugin = "org.jetbrains.dokka")

    tasks.dokka {
        outputFormat = "html"
        outputDirectory = "$buildDir/javadoc"
    }
    val sourcesJar by tasks.creating(Jar::class) {
        from(sourceSets.main.get().allSource)
        archiveClassifier.set("sources")
    }
    val dokkaJar by tasks.creating(Jar::class) {
        group = JavaBasePlugin.DOCUMENTATION_GROUP
        archiveClassifier.set("javadoc")
        from(tasks.dokka)
    }

    publishing {
        publications {
            create<MavenPublication>("snapshot") {
                from(components["java"])
//                artifact(dokkaJar) //no dokka
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
}

tasks.create("testAll") {
    group = "verification"
    dependsOn(
        rootProject.subprojects.map { it.tasks.withType<Test>() }
    )
}
