#pragma once

#ifdef CPATOOLS_IMPLEMENTATION

namespace cpa::memory {
memory::host_address_type baseAddress = nullptr;
size_t size = 0;
bool readonly = true;
};

namespace cpa::global {
  
pointer<stAlways> g_stAlways = nullptr;
pointer<stEngineStructure> g_stEngineStructure = nullptr;
pointer<stObjectType> g_stObjectTypes = nullptr;
pointer<stInputStructure> g_stInputStructure = nullptr;
pointer<stRandom> g_stRandomStructure = nullptr;

pointer<stSuperObject> p_stActualWorld = nullptr;
pointer<stSuperObject> p_stDynamicWorld = nullptr;
pointer<stSuperObject> p_stInactiveDynamicWorld = nullptr;
pointer<stSuperObject> p_stFatherSector = nullptr;

pointer<uint8> g_bGhostMode = nullptr;

static void cacheObjectTypes();

static bool isValidState() {
  if (!g_stEngineStructure) return false;
  return !g_stEngineStructure->engineFrozen
  &&      g_stEngineStructure->mode != 5
  &&      g_stEngineStructure->mode != 6
  &&      p_stActualWorld
  &&      p_stDynamicWorld
  &&      p_stInactiveDynamicWorld
  &&      p_stFatherSector;
}

static void load() {
  
  // Global
  g_stEngineStructure = pointer<stSuperObject>    (GCN_POINTER_ENGINE_STRUCTURE);
  g_stInputStructure  = pointer<stInputStructure> (GCN_POINTER_INPUT_STRUCTURE);
  g_stRandomStructure = pointer<stRandom>         (GCN_POINTER_RND);
  g_bGhostMode        = pointer<uint8>            (GCN_POINTER_GHOST_MODE);
  
  // World
  p_stActualWorld          = *doublepointer<stSuperObject>(GCN_POINTER_ACTUAL_WORLD);
  p_stDynamicWorld         = *doublepointer<stSuperObject>(GCN_POINTER_DYNAMIC_WORLD);
  p_stInactiveDynamicWorld = *doublepointer<stSuperObject>(GCN_POINTER_INACTIVE_DYNAMIC_WORLD);
  p_stFatherSector         = *doublepointer<stSuperObject>(GCN_POINTER_FATHER_SECTOR);
  
  if (isValidState()) {
    pointer<uint8> fix = *doublepointer<uint8>(GCN_POINTER_FIX);
    pointer<uint8> lvl = *doublepointer<uint8>(GCN_POINTER_LVL);
    
    #pragma mark FIX
    fix += (32 + 4); // 4=identity matrix
    fix += 4; // localizationStructure
    uint32 levelNameCount = *(uint32*)fix;
    fix += 4;
    uint32 demoNameCount = *(uint32*)fix;
    fix += 4;
    fix += 12 * demoNameCount;
    fix += 12 * demoNameCount;
    fix += 30 * levelNameCount;
    fix += 30 + 2; // First level name + padding
    fix += 4 + 4; // Language count + language offset
    uint32 fixTextureCount = *(uint32*)fix;
  
    #pragma mark LVL
    lvl += 4 * 4; // ?
    lvl += 24; // text
    lvl += 4 * 60; // ?
    uint32 lvlTextureCount = *(uint32*)lvl;
    lvl += 4;
    lvl += (lvlTextureCount - fixTextureCount) * 4 * 2;
    lvl += 4 * 5; // actualWorld (0), dynamicWorld (0), inactiveDynamicWorld (0), fatherSector (0), firstSubmapPosition
    g_stAlways = lvl;
    lvl += sizeof *g_stAlways;
    g_stObjectTypes = lvl;
    
    cacheObjectTypes();
  }
}

#pragma mark - Object type names

struct objectNameCache {
  std::vector<std::string> familyNames;
  std::vector<std::string> modelNames;
  std::vector<std::string> instanceNames;
};

std::map<std::string, objectNameCache> objectNameCacheTable;

static void cacheObjectTypes() {
  if (objectNameCacheTable.find(g_stEngineStructure->currentLevelName) == objectNameCacheTable.end()) {
    objectNameCache& cache = objectNameCacheTable[g_stEngineStructure->currentLevelName];
    g_stObjectTypes->family.forEach([&](stObjectTypeElement* e, void*) { cache.familyNames.push_back(std::string(e->name)); });
    g_stObjectTypes->model.forEach([&](stObjectTypeElement* e, void*) { cache.modelNames.push_back(std::string(e->name)); });
    g_stObjectTypes->instance.forEach([&](stObjectTypeElement* e, void*) { cache.instanceNames.push_back(std::string(e->name)); });
  }
}

auto objectTypeNameLookup(int type, int idx) -> std::string {
  if (objectNameCacheTable.find(g_stEngineStructure->currentLevelName) != objectNameCacheTable.end()) {
    objectNameCache& cache = objectNameCacheTable[g_stEngineStructure->currentLevelName];
    try {
      if (type == objectTypeFamily) return cache.familyNames.at(idx);
      if (type == objectTypeModel) return cache.modelNames.at(idx);
      if (type == objectTypeInstance) return cache.instanceNames.at(idx);
    } catch (std::out_of_range& e) {
      return "Invalid name";
    }
  }
  return "Invalid name";
}

};

#endif
