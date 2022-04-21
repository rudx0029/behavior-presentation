# Behavior Re-Design Presentation

This repository contains a sample application which demonstrates the behavior redesign.

## `main.cpp`

Contains the implementations for the Motion Elements `WalkToPosition` and `Stop`, implementing the compile-time behavior/reaction contract.

In `main`, an example behavior is created using composition utilizing the `Sequence` element to sequentially execute a `WalkToPosition` element and a `Stop` element. The `Sequence` element can be found in `elements.hpp`.

The example behavior is then executed asynchronously while the 'strategy' waits via a `std::future`.

## `elements.hpp`

Implementations for the `Sequence` element, the interface `BehaviorElement`, and the CRTP base class `MotionElement<T>` can be found here.

The most interesting class in the file, `MotionElement<T>`, specifies the behavior/reaction contract for a Motion Element, e.g. walk to position. The base class implements the `BehaviorElement` interface and then requires any derivatives to implement `MotionElement<T>`'s static interface.

## `executor.hpp`

A light weight example of how to execute a `BehaviorElement`.

## `types.hpp`

Dependencies used in the design.

## Build and Run via CMake

```cmake
cmake -B build -S .
cmake --build build
./build/dfs
```
