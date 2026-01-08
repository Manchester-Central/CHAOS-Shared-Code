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

Example features:
* Special gamepad wrapper around `CommandXboxController` that adds deadband to axes and stick angles and magnitude.
* Our Autobuilder system, used for defining auto commands and running our auto scripts
* Logging to USB and NetworkTables
* PID Tuners, used for tuning PIDs from the dashboard
* Our base swerve code (pulled/modified from our 2023 code)
* DashboardNumber, a tool for creating numbers that can be easily used in the code and also updated on a dashboard

## Setup
You can run this in the latest WPILib VSCode for java (last tested with 2025).

## Releases & Packages
New releases and packages will be published automatically when changes are pushed to the `main` branch (if the build succeeds).

Beta packages (but not releases) will be published when pushing changes to a Merge Request.

All releases can be found here: https://github.com/Manchester-Central/CHAOS-Shared-Code/releases

All package versions (official and beta) can be found here: https://github.com/Manchester-Central/CHAOS-Shared-Code/packages/2033853/versions


# Add to your project

You can either add this to your project via maven, manually importing a JAR file, or by locally including the unbuilt project.


## Include project locally (side folder or submodule)

You can also manually add the CHAOS Shared Code project to yours, which is helpful when trying to make changes to the shared code project and quickly testing them.
This expects you to put the shared project in the same parent folder as your target project (but doesn't need to be if you change the file path in `settings.gradle`)

### Git Submodule Setup (Optional)

If including this repo as a git submodule, go to the workspace root in a command prompt and type in
```bash
git clone https://github.com/Manchester-Central/CHAOS-Shared-Code.git
git submodule add ./CHAOS-Shared-Code
```

You can then type in `git status` and you should see a red line "CHAOS-Shared-Code" indicating uncommitted changes. You can now `git add CHAOS-Shared-Code` now or later. If you have a preferred GUI, any normal git clone will accomplish the same thing. Proceed with the next two steps to tell gradle to also include the submodule.

#### Managing The Submodule

If developing with VS Code, all changes to this submodule will show up in the git/version control section. You can make changes to both libraries at once on the fly. Make sure to push changes to the submodule before pushing changes to the main workspace, since submodules are referenced _by hash_. The submodule must be generate a new hash for that commit before the main workspace can know that hash. Keep in mind that you probably want to choose the main branch of Shared-Code when choosing a commit hash!

### build.gradle

Add this to the dependencies section of your `build.gradle` file:
```groovy
    implementation project(':chaos-shared-code')
```

### settings.gradle
Add this VERY CASE-SENSITITVE block to the end of your `settings.gradle` file:
```groovy
include ':chaos-shared-code'
project(':chaos-shared-code').projectDir = new File("CHAOS-Shared-Code")
```

If you elect to place the Shared-Code repo alongside the workspace root, instead of inside it, change the last line to.

```groovy
project(':chaos-shared-code').projectDir = new File("../CHAOS-Shared-Code")
```

## Maven (preferred)

### Secrets.json
Create a file called `secrets.json` at the root level of your WPILib project and this text to it:
```json
{
    "github_package_auth" : {
        "username" : "my_username",
        "personal_access_token" : "github_pat_redacted"
    }
}
```
Replace `my_username` with your github username.

Replace `github_pat_redacted` with your github personal access token with readonly permission to all public repos. See [this guide](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#creating-a-fine-grained-personal-access-token) for how to make one.

**IF ADDING THIS TO A NEW PROJECT, MAKE SURE TO ADD THE `secrets.json` FILE TO `.gitignore` SO YOUR TOKEN IS NOT PUBLISHED TO GITHUB.**

If you are on a CHAOS shared computer, look at [this private discussion](https://github.com/orgs/Manchester-Central/discussions/1) for how to configure the shared token in your project.

### build.gradle
Add these to the `build.gradle` file:

Make sure to add this above the dependencies sections:
```groovy
def secretsFile = file("secrets.json")
if (secretsFile.exists()) {
    def secrets = new groovy.json.JsonSlurper().parseText(secretsFile.text)
    project.ext.sharedCodeUsername = secrets.github_package_auth.username
    project.ext.sharedCodePassword = secrets.github_package_auth.personal_access_token
}
else {
    project.ext.sharedCodeUsername = System.getenv('CHAOS_PACKAGE_USERNAME')
    project.ext.sharedCodePassword = System.getenv('CHAOS_PACKAGE_PASSWORD')
}

repositories {
    maven {
        url = uri("https://maven.pkg.github.com/Manchester-Central/CHAOS-Shared-Code")
        credentials {
            username = sharedCodeUsername
            password = sharedCodePassword
        }
    }
}
```

Add this line inside the dependencies section (change `2025.0.14` with the latest package version)
```groovy
    implementation 'com.chaos131:shared:2025.0.14'
```

You can see a live example here:
https://github.com/Manchester-Central/2025-offseason-bot/blob/main/build.gradle

If runnning a build on Github, you'll also need to add this step before the build step:
```yml
      - name: Set environment variable from context
        run: |
          echo "CHAOS_PACKAGE_USERNAME=${{ vars.CHAOS_PACKAGE_USERNAME }}" >> $GITHUB_ENV
          echo "CHAOS_PACKAGE_PASSWORD=${{ vars.CHAOS_PACKAGE_PASSWORD }}" >> $GITHUB_ENV
```

You can see an example of the build script here:
https://github.com/Manchester-Central/2025-offseason-bot/blob/main/.github/workflows/build.yml
(This also expects a valid username and token stored in the [organizational](https://github.com/organizations/Manchester-Central/settings/variables/actions) or project variables)

## Manually add JAR

Download (or clone) the shared code, run the "WPILib: Build Robot Code" task, and then run the "Java: Export Jar" task with all defaults. Copy the produced jar file into a libs folder in your target project.
**You should also be able to grab the latest jar files from the Release section on github.**

In your build.gradle, add this to your dependencies (you might need to double check the file name matches).
```
implementation files('libs/CHAOS-Shared-Code.jar')
```
