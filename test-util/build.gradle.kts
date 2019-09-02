val ext = project.rootProject.extra
val junit: String by ext
val xchart: String by ext

plugins {
    kotlin("jvm")
}
repositories {
    mavenLocal()
}
dependencies {
    //    implementation("org.futurerobotics.temporaryname:core:0.0.1")
    implementation(project(":core"))

    implementation(junit)
    implementation(xchart)
}