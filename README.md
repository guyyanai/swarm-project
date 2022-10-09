# swarm-project

## Introduction

This repository contains our Drone Swarm project which we developed as a part of Real Time Data Processing course. More information can be obtained in the [problem specification](ProblemSpecification.pdf) (note: it is in Hebrew).

## Compile

Under the 'arucoReader' directory located in the root directory of the project, execute the following commands to compile the source code.

```zsh
mkdir build
cd build
cmake ..
make
```

## Execute

After compiling the source code, you can find the binary file 'runAruco' under the 'build' directory.
Execute the binary with the following command:

```zsh
./runAruco
```
