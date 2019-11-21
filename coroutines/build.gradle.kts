val ext = project.rootProject.extra

val coroutines: String by ext

plugins {
    kotlin("jvm")
}

dependencies {
    implementation(project(":core"))
    implementation(coroutines)
}

extra["publish"] = true
