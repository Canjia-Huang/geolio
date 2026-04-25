# Geogram-Mesh-Utils

**GeogramMeshUtils** is a lightweight, header-only C++ library designed to perform common surface and volumetric mesh operations within the Geogram ecosystem. It allows for seamless integration into existing projects that depend on Geogram.

## :thinking: Why I built this?

[**Geogram**](https://github.com/BrunoLevy/geogram) utilizes a highly optimized mesh data structure based on incidence and adjacency arrays. 
While this design provides exceptional performance for processing massive meshes, it sacrifices the convenience found in Half-edge-based data structures. 
Consequently, performing even simple local mesh operations (such as edge swap) becomes tedious, as maintaining the consistency of adjacency relationships often requires expensive global `connect()` calls after every modification.

**GeogramMeshUtils** bridges this gap. It provides a suite of efficient, mesh manipulation utilities that work directly with **Geogram**'s mesh data structures, allowing you to perform operations without the overhead of rebuilding the entire mesh connectivity.

## :link: Requirements / dependencies

All you need is [**Geogram**](https://github.com/BrunoLevy/geogram).

[//]: # (## :butterfly: Features)