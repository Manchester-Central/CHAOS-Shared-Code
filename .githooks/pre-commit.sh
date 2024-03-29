#!/bin/bash

URL="https://github.com/google/google-java-format/releases/download/v1.19.2/google-java-format-1.19.2-all-deps.jar"
JARFILE=".format/google-java-format.jar"

if ! shopt -q globstar; then
    shopt -s globstar
fi

downloadJar() {
    echo "Downloading formatter"
    curl --create-dirs -L -o $JARFILE $URL
}

runFormatter() {
    java -jar $JARFILE --replace src/**/*.java
}

if [ -f $jarfile ] && [ "$(java -jar "$JARFILE" --version 2>&1 | cut -d ' ' -f 3-)" != "1.9" ]; then
    echo "Old formatter version detected, updating it..."
    downloadJar || echo >&2 "Error updating jar"
fi

if [ -d $JARFILE ]; then
    echo >&2 "Error: $JARFILE already exists but is a directory."
    exit 1
elif [ -f $JARFILE ]; then
    echo "$JARFILE found"
else
    if ! downloadJar; then
        echo >&2 "Error downloading formatter, skipping format"
        exit 0
    fi
fi

runFormatter
