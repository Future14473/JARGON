val ext = project.rootProject.extra

val junitVersion: String by ext
val xchartVersion: String by ext

plugins {
    kotlin("jvm")
}

dependencies {
    implementation(project(":core"))

    testApi(project(":testUtils"))
    testImplementation("org.knowm.xchart:xchart:$xchartVersion")
}

