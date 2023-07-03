# Introduction

`kindyngen` is a toolchain to transform [_composable models_](https://github.com/comp-rob2b/modelling-tutorial) of kinematic chains and queries thereupon into _correct-by-construction_ solver code. Example solvers are forward position/velocity/acceleration kinematics or forward/inverse/hybrid dynamics. Additionally, the `kindyngen` toolchain enables to _interleave_ those familiar algorithms with additional computations, including control or estimation that reuses the kinematic chain state as the most simple _world model_ available to a robot.

Structurally, the `kindyngen` toolchain consists of two parts. First, the `kindynsyn` synthesizer to compute a kinematics or dynamics algorithm given a kinematic chain model. Second, the code generator that transforms the `kindynsyn` output to code. For now the target language for the code generator is C (supported by various software libraries).


## Composability

Both, the synthesizer and the code generator are composable. On the one hand, they require the composable building blocks which we call _steps_ (for the synthesizer) and _fragments_ (for the code generator). On the other hand, a top-level context represents the overall composition. This composition includes the solver _configuration_ (for the synthesizer) and an application template (for the code generator) that contains, for example, the `main` function.


## Tutorials

In following tutorials we will provide a developer with the basic understanding of the `kindyngen` toolchain. This includes an overview of the terminology as well as the architecture. Afterwards we start with configuring solvers using the built-in functionality of `kindyngen`. Finally, the objective is to demonstrate how a developer can extend the toolchain with custom functionality. For this tutorial we focus on the specification of a Cartesian level controller and how it maps to the robot's joint-level hardware interface.
