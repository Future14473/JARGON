@file:Suppress("PublicApiImplicitType", "SpellCheckingInspection", "KDocMissingDocumentation")

import com.jfrog.bintray.gradle.BintrayExtension
import com.jfrog.bintray.gradle.BintrayPlugin
import com.jfrog.bintray.gradle.tasks.BintrayPublishTask
import org.jetbrains.dokka.gradle.DokkaPlugin
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile
import java.util.*

//will later be applied to all subprojects
group = "org.futurerobotics.jargon"
version = "snapshot1"
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
    id("com.jfrog.bintray") version "1.8.4"
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
            @Suppress("SuspiciousCollectionReassignment")
            freeCompilerArgs += listOf(
                "-Xuse-experimental=kotlin.Experimental" //for contracts
            )
        }
    }
}

var <T> Property<T>.v: T?
    get() = get()
    set(value) {
        set(value)
    }

//Tests
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

tasks.create("testAll") {
    group = "verification"
    dependsOn(
        subprojects.map { it.tasks.withType<Test>() }
    )
}
//publishing
val properties = Properties()
val propertiesFile = file("local.properties")
if (propertiesFile.exists()) {
    properties.load(propertiesFile.inputStream())
}
val githubURL = "https://github.com/Future14473/JARGON"

val bintrayUser = properties.getProperty("bintray.user") ?: System.getenv("BINTRAY_USER")
val bintrayKey = properties.getProperty("bintray.key") ?: System.getenv("BINTRAY_KEY")

fun Project.configurePublish() {
    apply<MavenPublishPlugin>()
    apply<BintrayPlugin>()
    apply<DokkaPlugin>()
    val isSnapshot by extra { version.toString().contains(Regex("[a-zA-Z]")) }

    tasks.dokka {
        outputFormat = "html"
        outputDirectory = "$buildDir/javadoc"
    }
    val sourcesJar by tasks.creating(Jar::class) {
        from(sourceSets.main.get().allSource)
        archiveClassifier.v = "sources"
    }

    val dokkaJar by tasks.creating(Jar::class) {
        group = JavaBasePlugin.DOCUMENTATION_GROUP
        archiveClassifier.v = "javadoc"
        from(tasks.dokka)
    }

    val publicationName = if (isSnapshot) "snapshot" else "release"
    publishing.publications {
        create<MavenPublication>(publicationName) {
            from(components["java"])
            artifact(sourcesJar)
            if (!isSnapshot)
                artifact(dokkaJar)
            this@configurePublish.configMavenPublication(this)
        }
    }
    var doPublish = true
//    if(isSnapshot) doPublish = false
    if (doPublish && (bintrayUser == null || bintrayKey == null)) {
        logger.warn("Bintray user or key not found")
        doPublish = false
    }
    bintray {
        this.user = user
        this.key = key
        publish = doPublish
        override = isSnapshot
        setPublications(publicationName)
        pkg(delegateClosureOf<BintrayExtension.PackageConfig> {
            repo = if (isSnapshot) "maven-snapshot" else "maven"
            name = "JARGON"
            userOrg = "future14473"
            setLicenses("MIT")
            vcsUrl = "https://github.com/Future14473/JARGON"
            version(delegateClosureOf<BintrayExtension.VersionConfig> {
                name = project.version.toString()
                released = Date().toString()
                if (!isSnapshot) {
                    vcsTag = "v" + project.version
                }
            })
        })
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

tasks.withType(BintrayPublishTask::class.java).all {
    onlyIf { false }
}
