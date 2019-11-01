plugins {
    kotlin("jvm")
    `maven-publish`
}

dependencies {
    api(project(":core"))
    api(project(":blocks"))
    api(project(":pathing"))
    api(project(":state-space"))
}

publishing {
    publications {
        create<MavenPublication>("publish") {
            versionMapping {
                usage("java-api") {
                    fromResolutionOf("runtimeClasspath")
                }
                usage("java-runtime") {
                    fromResolutionResult()
                }
            }
        }
    }
}
