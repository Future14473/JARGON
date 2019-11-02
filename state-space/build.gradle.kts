@file:Suppress("KDocMissingDocumentation", "PublicApiImplicitType", "SpellCheckingInspection")

val ext = project.rootProject.extra

val hipparchus: ((String) -> String) by ext

plugins {
    kotlin("jvm")
}

dependencies {
    api(project(":core"))
    api(project(":blocks"))
    implementation(hipparchus("filtering"))

}
val publish by extra(true)
