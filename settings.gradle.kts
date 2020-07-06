pluginManagement {
    repositories {
        mavenCentral()
        gradlePluginPortal()
        maven("https://dl.bintray.com/kotlin/kotlin-eap")
    }
}
rootProject.name = "jargon"
include(":core", ":core:test-util")
include(":pathing")
include(":state-space")
