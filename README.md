NanoRT OSPRay Module
====================

An OSPRay module which forwards ray intersections to 
NanoRT(https://github.com/lighttransport/nanort). This module
is written to work with OSPRay's ```devel``` branch.

Build/Run Instructions
----------------------

Build:

-   Clone the repo into modules/ directory in your OSPRay source tree, the
module will be enabled by default.

Run:

-   ```./ospGlutViewer [triangle mesh file] -m nanort --trianglemesh-type nanort```
