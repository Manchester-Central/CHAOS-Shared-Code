![GitHub top language](https://img.shields.io/github/languages/top/Manchester-Central/CHAOS-Shared-Code?style=flat-square)
[![WPILib License](https://img.shields.io/badge/license-WPI_Lib-blue?style=flat-square)](https://github.com/Manchester-Central/CHAOS-Shared-Code/blob/main/WPILib-License.md)
![GitHub contributors](https://img.shields.io/github/contributors/Manchester-Central/CHAOS-Shared-Code?style=flat-square)
![GitHub commit activity](https://img.shields.io/github/commit-activity/w/Manchester-Central/CHAOS-Shared-Code?style=flat-square)
![GitHub issues](https://img.shields.io/github/issues/Manchester-Central/CHAOS-Shared-Code?style=flat-square)
![GitHub pull requests](https://img.shields.io/github/issues-pr/Manchester-Central/CHAOS-Shared-Code?style=flat-square)
![GitHub followers](https://img.shields.io/github/followers/Manchester-Central?style=social)


![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/Manchester-Central/CHAOS-Shared-Code/gradle.yml?style=flat-square)

# CHAOS Shared Code
This is code that our team uses for different projects.

TODO: Add more details


## Setup 
You can run this in the latest WPILib VSCode for java (last tested with 2022).

## Create a new release
When making changes to this, make sure to create a new release. Add an updated tag number and include the exported jar file as an asset with the release.


## Add to your project

You can either add this to your project via jitpack or by manually including the jar

### Jit Pack
Add these to the build gradle:

Into dependencies (change `1.4.0` with the latest release or another commit SHA)
```
implementation 'com.github.Manchester-Central:CHAOS-Shared-Code:1.4.0'
```

And make sure to have this at the root:
```
repositories { 
    jcenter() 
    maven { url "https://jitpack.io" }
}
```

### Manually add

Download the shared code, run the "WPILib: Build Robot Code" task, and then run the "Java: Export Jar" task with all defaults. Copy the produced jar file into a libs folder in your target project.
**You should also be able to grab the latest jar files from the Release section on github.**

In your build.gradle, add this to your dependencies.
```
implementation files('libs/CHAOS-Shared-Code.jar')
```
