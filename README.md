# CHAOS Shared Code
This is code that our team uses for different projects.

TODO: Add more details


## Setup 
TBD

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

Download the shared code and run the "Java: Export Jar" task with all defaults. Copy the produced jar file into a libs folder in your target project.

In your build.gradle, add this to your dependencies.
```
implementation files('libs/CHAOS-Shared-Code.jar')
```
