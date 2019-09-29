@file:Suppress("PublicApiImplicitType", "SpellCheckingInspection", "KDocMissingDocumentation")

import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

val hipparchusVersion by extra("1.5")
val striktVersion by extra("0.22.2")
val xchartVersion by extra("3.5.4")
val junitVersion by extra("4.12")
val junit5Version by extra("5.5.2")

val dokka by extra("org.jetbrains.dokka")

val hipparchus by extra<(String) -> String> { { "org.hipparchus:hipparchus-$it:$hipparchusVersion" } }

val coroutines by extra("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.1.1")

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

plugins {
    id("org.jetbrains.dokka") version "0.9.18" apply false
}

subprojects {
    group = "org.futurerobotics.jargon"
    version = "0.1.0-SNAPSHOT"
    repositories {
        mavenCentral()
        jcenter()
    }
    plugins.withId("org.jetbrains.kotlin.jvm") {
        dependencies {
            // <3 contextual String.invoke
            "implementation"(kotlin("stdlib-jdk8"))
        }
        tasks.withType<KotlinCompile> {
            kotlinOptions.jvmTarget = "1.8"
        }
    }
    tasks.withType<Test> {
        useJUnitPlatform {
            excludeTags("Not a test")
        }
    }
}