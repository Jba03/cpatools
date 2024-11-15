**cpatools** is a single-header library for various games created with the CPA engine.
It aims to provide a complete, documented interface of the engine, as well as adding the tools necessary to enable modding.

```cpp
// Implementing the static interface.
// If building for emulator, define CPA_TARGET_EMULATOR here.
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
