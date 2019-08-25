val ext = project.rootProject.extra

val junitVersion: String by ext
val xchartVersion: String by ext

plugins {
    kotlin("jvm")
}

dependencies {
    implementation(project(":core"))

    compile("junit:junit:$junitVersion")
    implementation("org.knowm.xchart:xchart:$xchartVersion")
}