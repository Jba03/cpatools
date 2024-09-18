**cpatools** is a single-header interface for various games created with the CPA engine.
It aims to provide a complete, documented interface of the engine, as well as adding the necessary tools to enable realtime modification.

At the cost of runtime version compatibility, the interface of cpatools is static - meaning it may only be compiled for use with _one_ specific platform/version of the engine.
This also means it operates directly on the memory of the game, which is much faster than reading all structures and types dynamically.


**Usage**
```cpp
// Implementing the static interface.
#define CPATOOLS_IMPLEMENTATION
#include <cpatools/cpatools.hpp>
using namespace cpa;
```

```cpp
// Loading context from live memory
cpa::global::loadMemory(memory, memorySize);

// Do stuff
pointer<stSuperObject> player = global::g_stEngineStructure->currentMainPlayers[0];
player->position() = stVector3D(-12.0f, 24.0f, 6.0f);
```
