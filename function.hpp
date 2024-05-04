#pragma once

namespace cpa {

/// Serialization
#define s(what) s.add(#what, what);

#pragma mark EngineStructure

auto structure::stEngineStructure::loadLevel(std::string levelName) -> void {
  nextLevelName = levelName;
  mode = engineModeChangeLevel;
}

#pragma mark EngineObject

auto structure::stEngineObject::name(int16_t type) -> std::string {
  std::string name;
  extern ObjectNameResolver nameResolver;
  for (int i : {stdGame->instanceType, stdGame->modelType, stdGame->familyType}) {
    name = nameResolver(type, &i);
    if (name != "Invalid name") break;
  }
  
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
    return stVector3D();
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
//
//auto structure::stSuperObject::serialize(serializer::node& s) {
//  s.type("stSuperObject");
//  s(type)
//  s(data)
//  s(firstChild)
//  s(lastChild)
//  s(numChildren);
//  for (auto child : *this) s(child);
//  s.add("next", next, false);
//  s.add("prev", prev, false);
//  s.add("parent", parent, false);
//  s(localTransform);
//  s(globalTransform);
//  s(prevFrameProcessed);
//  s(drawFlags);
//  s(flags);
//  s(visualBBox);
//  s(collideBBox);
//  s(semiLookAt);
//  s(transparency);
//  s(outlineColor);
//  s(displayPriority);
//  s(ilstatus);
//  s(ambientColor);
//  s(parallelDirection);
//  s(parallelColor);
//  s(superimpose);
//  s(isSuperObject);
//  s(transition);
//}

#pragma mark - Static functions

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

static auto rayTriangleIntersect(structure::stVector3D &origin, structure::stVector3D &direction, structure::stVector3D& A, structure::stVector3D& B, structure::stVector3D& C, bool cullBackface, double &t) -> bool {
  structure::stVector3D E1 = B - A;
  structure::stVector3D E2 = C - A;
  structure::stVector3D H = direction.cross(E2);
  
  const double det = E1.dot(H);
  const double invdet = 1.0f / det;
  
  if (det < EPSILON && cullBackface) return false;
  if (fabs(det) < EPSILON && !cullBackface) return false;
  
  structure::stVector3D S = origin - A;
  const double u = S.dot(H) * invdet;
  if (u < 0.0f || u > 1.0f) return false;
  
  structure::stVector3D Q = S.cross(E1);
  const double v = direction.dot(Q) * invdet;
  if (v < 0.0f || u + v > 1.0f) return false;
  
  t = invdet * E2.dot(Q);
  return t > EPSILON;
}

template<bool UseOctreeOptimization = false>
static auto segmentCollideObjectIntersect(pointer<structure::stCollideObject> collObj, structure::stMatrix4D T, structure::stVector3D start, structure::stVector3D end, structure::stVector3D& intersectionPoint) -> bool {
  try {
    if (!collObj) return false;
    
    structure::stVector3D origin = start;
    structure::stVector3D direction = (end - start).normalize();
    const double l = (end - start).length();
    
    for (int i = 0; i < collObj->numElements; i++) {
      int16_t type = collObj->elementTypes[i];
      if (type == collideObjectTypeIndexedTriangles) {
        pointer<structure::stCollideElementIndexedTriangles> element = ((structure::stCollideElementIndexedTriangles**)collObj->elements)[i];
        structure::stVector3D* vertices = collObj->vertices;
        uint16* indices = element->faceIndices;
        
        for (int16_t index = 0; index < element->numFaces; index++) {
          uint16 idx0 = *(indices + index * 3 + 0);
          uint16 idx1 = *(indices + index * 3 + 1);
          uint16 idx2 = *(indices + index * 3 + 2);
          
          structure::stVector3D A = *(vertices + idx0);
          structure::stVector3D B = *(vertices + idx1);
          structure::stVector3D C = *(vertices + idx2);
          
          structure::stVector3D TA = (T*A).xyz();
          structure::stVector3D TB = (T*B).xyz();
          structure::stVector3D TC = (T*C).xyz();
          
          double t;
          if (rayTriangleIntersect(origin, direction, TA, TB, TC, true, t)) {
            if (t < l) { // constrain to segment
              intersectionPoint = origin + direction * t;
              return true;
            }
          }
        }
      }
    }
    return false;
  } catch (...) {
    return false;
  }
}

/************************/
/** ``FUNCTION END`` **/
/************************/

#undef s

};
