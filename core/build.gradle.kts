val ext = project.rootProject.extra

val komaVersion: String by ext

plugins {
    kotlin("jvm")
}

dependencies {
    implementation("com.kyonifer:koma-core-ejml:$komaVersion")

    testApi(project(":testUtils"))
}
