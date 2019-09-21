import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

buildscript {
    val kotlinVersion by extra("1.3.50")
    val junitVersion by extra("4.12")
    val xchartVersion by extra("3.5.4")
    val hipparchusVersion by extra("1.5")
    val junit by extra("junit:junit:$junitVersion")
    val xchart by extra("org.knowm.xchart:xchart:$xchartVersion")
    val dokka by extra("org.jetbrains.dokka")
    val hipparchus by extra("org.hipparchus:hipparchus-core:$hipparchusVersion")
    repositories {
        mavenCentral()
    }
    dependencies {
        classpath(kotlin("gradle-plugin", version = kotlinVersion))
    }
}

plugins {
    //    `build-scan`
    id("org.jetbrains.dokka") version "0.9.18" apply false
}
//
//buildScan {
//    termsOfServiceUrl = "https://gradle.com/terms-of-service"
//    termsOfServiceAgree = "yes"
//
//}
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
    tasks.getting(Test::class) {
        useJUnit {
            excludeCategories("org.futurerobotics.jargon.NotARealTest")
        }
    }
}
