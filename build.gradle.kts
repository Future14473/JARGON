import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

buildscript {
    val kotlinVersion by extra("1.3.50")
    val junitVersion by extra("4.12")
    val xchartVersion by extra("3.5.4")
    val komaVersion by extra("0.12")
    repositories {
        mavenCentral()
    }
    dependencies {
        classpath(kotlin("gradle-plugin", version = kotlinVersion))
    }
}
val javaVersion = 8
subprojects {
    repositories {
        mavenCentral()
        maven {
            url = uri("https://dl.bintray.com/kyonifer/maven")
        }
        jcenter()
    }
    plugins.withId("org.jetbrains.kotlin.jvm") {
        println("configuring kotlin on project $name")
        dependencies {
            // <3 contextual String.invoke
            "implementation"(kotlin("stdlib-jdk$javaVersion"))
        }
        tasks.withType<KotlinCompile> {
            kotlinOptions.jvmTarget = "1.$javaVersion"
        }
    }
}

