#pragma once

namespace cpa::global {
auto objectTypeNameLookup(int type, int idx) -> std::string;
};

namespace cpa {

#pragma mark EngineStructure

auto structure::stEngineStructure::loadLevel(std::string levelName) -> void {
  nextLevelName = levelName;
  mode = engineModeChangeLevel;
}

#pragma mark EngineObject

auto structure::stEngineObject::name(int16_t type) -> std::string {
  std::string name;
  for (int i : {stdGame->instanceType, stdGame->modelType, stdGame->familyType})
    if ((name = global::objectTypeNameLookup(type, i)) != "Invalid name") break;
  return name;
}
/// Get the superobject associated with this actor
auto structure::stEngineObject::superobject() -> pointer<stSuperObject> {
  return stdGame->superObject;
}

/// Return the AI model of this actor
auto structure::stEngineObject::aiModel() -> pointer<stAIModel> {
  return brain->mind->aiModel;
}

/// Get the dsg variable memory
auto structure::stEngineObject::dsgMem() -> pointer<stDsgMem> {
  return brain->mind->dsgMem;
}

auto structure::stEngineObject::dsgVar(int idx, uint32_t* type) -> pointer<> {
  try {
    pointer<stDsgMem> mem = brain->mind->dsgMem;
    if (idx > (*mem->dsgVars)->infoLength) return nullptr;
    pointer<stDsgVarInfo> info = mem->dsgVarInfo(idx);
    if (type) *type = info->type;
    return (uint8_t*)mem->currentBuffer + info->memoryOffset;
  } catch (bad_pointer& e) {
    return nullptr;
  }
}

auto structure::stEngineObject::speed() -> stVector3D {
  try {
    return dynam->dynamics->base.previousSpeed;
  } catch (bad_pointer& e) {
    return stVector3D(0.0f, 0.0f, 0.0f);
  }
}

auto structure::stEngineObject::horizontalSpeed() -> float {
  auto s = speed();
  return sqrt(s.x() * s.x() + s.y() * s.y());
}

auto structure::stEngineObject::verticalSpeed() -> float {
  return speed().z();
}

#pragma mark - ZdxList

auto structure::stZdxList::all() -> std::vector<pointer<stCollideObject>> {
  assert(list.numEntries == numZdx); //should never happen
  std::vector<pointer<stCollideObject>> objects;
  list.forEach([&](pointer<stZdxListEntry> entry, void*) { objects.emplace_back(entry->data); });
  return objects;
}

#pragma mark SuperObject

auto structure::stSuperObject::typeName() -> std::string {
  switch (type) {
    case superobjectTypeNone:
      return "Dummy SuperObject";
    case superobjectTypeWorld:
      return "World";
    case superobjectTypeActor:
      return "Actor";
    case superobjectTypeSector:
      return "Sector";
    case superobjectTypePhysicalObject:
      return "PhysicalObject";
    case superobjectTypePhysicalObjectMirror:
      return "PhysicalObject.Mirror";
    case superobjectTypeIPO:
      return "IPO";
    case superobjectTypeIPOMirror:
      return "IPO.Mirror";
    case superobjectTypeSpecialEffect:
      return "SpecialEffect";
    case superobjectTypeNoAction:
      return "NoAction";
    case superobjectTypeMirror:
      return "Mirror";
    default:
      return "Invalid";
  }
}

auto structure::stSuperObject::name(bool fullname) -> std::string {
  try {
    switch (type) {
      case superobjectTypeActor:
        return actor->name();
      case superobjectTypeIPO:
        return fullname ? ipo->name : ipo->name.lastPathComponent();
      case superobjectTypeSector:
        return fullname ? sector->name : sector->name.lastPathComponent();
      default:
        return typeName();
    }
  } catch (bad_pointer& e) {
    return "";
  }
}

auto structure::stSuperObject::position() -> stVector3D& {
  try {
    return globalTransform->translation();
  } catch (bad_pointer& e) {
    return globalTransform->translation();
  }
}

#pragma mark - Static functions

/// Determine the sector of a world-space point
static inline auto sectorSearch(pointer<structure::stSuperObject> fatherSector, structure::stVector3D point) -> pointer<structure::stSuperObject> {
  try {
    float dNear = INFINITY;
    float dCurrent = INFINITY;
    float dVirtual = INFINITY;
    int8 p = sectorPriorityMin;
    int8 v = sectorPriorityMax;
    
    pointer<structure::stSuperObject> targetSector = nullptr;
    pointer<structure::stSuperObject> targetSectorVirtual = nullptr;
    
    fatherSector->forEachChild([&](pointer<structure::stSuperObject> object, void*) {
      pointer<structure::stSector> sector = object->sector;
      structure::stVector3D min = sector->min;
      structure::stVector3D max = sector->max;
      
      if (point >= min && point <= max) {
        structure::stVector3D distance = (min + max) / 2.0f - point;
        dNear = distance.length();
        
        if (!sector->isVirtual) {
          if (sector->priority > p) {
            targetSector = object;
            dCurrent = dNear;
            p = sector->priority;
          } else if (sector->priority == p && dNear < dCurrent) {
            targetSector = object;
            dCurrent = dNear;
          }
        } else {
          if (sector->priority > v) {
            targetSectorVirtual = object;
            dVirtual = dNear;
            v = sector->priority;
          } else if (sector->priority == v && dNear < dVirtual) {
            targetSectorVirtual = object;
            dVirtual = dNear;
          }
        }
      }
    });
    
    if (!targetSector) targetSector = targetSectorVirtual;
    if (!targetSector) targetSector = fatherSector->lastChild; // UNIVERS
    return targetSector;
  } catch (...) {
    return nullptr;
  }
}

/**********************/
/** ``FUNCTION END`` **/
/**********************/

#undef s

};
