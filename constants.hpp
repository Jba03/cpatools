#pragma once

#pragma mark Base dynamics flags
#define dynamicsFlagAnimation                 (1 <<  0) ///< Use animation speed?
#define dynamicsFlagCollide                   (1 <<  1) ///< Enable geometry collision?
#define dynamicsFlagGravity                   (1 <<  2) ///< Enable gravity?
#define dynamicsFlagTilt                      (1 <<  3) ///< Tilt
#define dynamicsFlagGi                        (1 <<  4) ///< Hanging from ceiling
#define dynamicsFlagOnGround                  (1 <<  5) ///< Is on ground
#define dynamicsFlagClimb                     (1 <<  6) ///< Climbing
#define dynamicsFlagCollisionControl          (1 <<  7) ///< Use dynamics param when colliding?
#define dynamicsFlagKeepWallZSpeed            (1 <<  8) ///< Preseve Z-axis momentum when colliding with a wall?
#define dynamicsFlagSpeedLimit                (1 <<  9) ///< Limit speed
#define dynamicsFlagInertia                   (1 << 10) ///< Has inertia?
#define dynamicsFlagStream                    (1 << 11) ///< Is affected by a stream?
#define dynamicsFlagStuckToPlatform           (1 << 12) ///< No slide on platform
#define dynamicsFlagIsScale                   (1 << 13) ///< Use scale parameters
#define dynamicsFlagSpeedImposeAbsolute       (1 << 14) ///< Impose absolute speed
#define dynamicsFlagSpeedProposeAbsolute      (1 << 15) ///< Propose absolute speed
#define dynamicsFlagSpeedAddAbsolute          (1 << 16) ///< Add absolute speed
#define dynamicsFlagSpeedImposeX              (1 << 17) ///< Impose absolute X-speed (after inertia & gravity)
#define dynamicsFlagSpeedImposeY              (1 << 18) ///< Impose absolute Y-speed (after inertia & gravity)
#define dynamicsFlagSpeedImposeZ              (1 << 19) ///< Impose absolute Z-speed (after inertia & gravity)
#define dynamicsFlagSpeedProposeX             (1 << 20) ///< Propose absolute X-speed (before inertia & gravity)
#define dynamicsFlagSpeedProposeY             (1 << 21) ///< Propose absolute Y-speed (before inertia & gravity)
#define dynamicsFlagSpeedProposeZ             (1 << 22) ///< Propose absolute Z-speed (before inertia & gravity)
#define dynamicsFlagSpeedAddX                 (1 << 23) ///< Add absolute X-speed
#define dynamicsFlagSpeedAddY                 (1 << 24) ///< Add absolute Y-speed
#define dynamicsFlagSpeedAddZ                 (1 << 25) ///< Add absolute Z-speed
#define dynamicsFlagLimitX                    (1 << 26)
#define dynamicsFlagLimitY                    (1 << 27)
#define dynamicsFlagLimitZ                    (1 << 28)
#define dynamicsFlagImposeRotation            (1 << 29) ///< Impose axis rotation
#define dynamicsFlagPlatformLock              (1 << 30) ///< Keep on platform
#define dynamicsFlagImposeTranslation         (1 << 31) ///< Impose translation

#pragma mark Base dynamics endflags
#define dynamicsEndFlagSizeBase              (1 <<  0) ///< Base size dynamics
#define dynamicsEndFlagSizeAdvanced          (1 <<  1) ///< Advanced size dynamics
#define dynamicsEndFlagSizeComplex           (1 <<  2) ///< Complex size dynamics
#define dynamicsEndFlagReserved              (1 <<  3)
#define dynamicsEndFlagMechanicsChanged      (1 <<  4)
#define dynamicsEndFlagPlatformCrash         (1 <<  5)
#define dynamicsEndFlagCanFall               (1 <<  6)
#define dynamicsEndFlagIsInit                (1 <<  7)
#define dynamicsEndFlagSpiderMechanic        (1 <<  8)
#define dynamicsEndFlagIsShoot               (1 <<  9)
#define dynamicsEndFlagSafeValid             (1 << 10)
#define dynamicsEndFlagComputeInvertMatrix   (1 << 11)
#define dynamicsEndFlagChangeScale           (1 << 12)
#define dynamicsEndFlagExec                  (1 << 13)
#define dynamicsEndFlagCollisionReport       (1 << 14)
#define dynamicsEndFlagNoGravity             (1 << 15)
#define dynamicsEndFlagStop                  (1 << 16)
#define dynamicsEndFlagSlidingGround         (1 << 17)
#define dynamicsEndFlagAlways                (1 << 18)
#define dynamicsEndFlagCrash                 (1 << 19)
#define dynamicsEndFlagSwim                  (1 << 20)
#define dynamicsEndFlagNeverFall             (1 << 21)
#define dynamicsEndFlagHanging               (1 << 22)
#define dynamicsEndFlagWallAdjust            (1 << 23)
#define dynamicsEndFlagActorMove             (1 << 24)
#define dynamicsEndFlagForceSafeWalk         (1 << 25)
#define dynamicsEndFlagDontUseNewMechanic    (1 << 26)

#pragma mark Designer variable types
#define dsgVarTypeBoolean           0
#define dsgVarTypeByte              1
#define dsgVarTypeUByte             2
#define dsgVarTypeShort             3
#define dsgVarTypeUShort            4
#define dsgVarTypeInt               5
#define dsgVarTypeUInt              6
#define dsgVarTypeFloat             7
#define dsgVarTypeVector            8
#define dsgVarTypeList              9
#define dsgVarTypeComport           10
#define dsgVarTypeAction            11
#define dsgVarTypeCapabilities      12
#define dsgVarTypeInput             13
#define dsgVarTypeSoundEvent        14
#define dsgVarTypeLight             15
#define dsgVarTypeGameMaterial      16
#define dsgVarTypeVisualMaterial    17
#define dsgVarTypeActor             18
#define dsgVarTypeWaypoint          19
#define dsgVarTypeGraph             20
#define dsgVarTypeText              21
#define dsgVarTypeSuperObject       22
#define dsgVarTypeSOLinks           23
#define dsgVarTypeActorArray        24
#define dsgVarTypeVectorArray       25
#define dsgVarTypeFloatArray        26
#define dsgVarTypeIntArray          27
#define dsgVarTypeWaypointArray     28
#define dsgVarTypeTextArray         29
#define dsgVarTypeTextRefArray      30
#define dsgVarTypeGraphArray        31
#define dsgVarTypeArray9            32
#define dsgVarTypeSNDEventArray     33
#define dsgVarTypeArray11           34
#define dsgVarTypeWay               35
#define dsgVarTypeActionArray       36
#define dsgVarTypeSuperObjectrArray 37
#define dsgVarTypeObjectList        38
#define dsgVarNumTypes              39

#pragma mark Script node types
#define scriptNodeTypeKeyword           0
#define scriptNodeTypeCondition         1
#define scriptNodeTypeOperator          2
#define scriptNodeTypeFunction          3
#define scriptNodeTypeProcedure         4
#define scriptNodeTypeMetaAction        5
#define scriptNodeTypeBeginMacro        6
#define scriptNodeTypeBeginMacro2       7
#define scriptNodeTypeEndMacro          8
#define scriptNodeTypeField             9
#define scriptNodeTypeDsgVarRef         10
#define scriptNodeTypeDsgVarRef2        11
#define scriptNodeTypeConstant          12
#define scriptNodeTypeReal              13
#define scriptNodeTypeButton            14
#define scriptNodeTypeConstantVector    15
#define scriptNodeTypeVector            16
#define scriptNodeTypeMask              17
#define scriptNodeTypeModuleRef         18
#define scriptNodeTypeDsgVarID          19
#define scriptNodeTypeString            20
#define scriptNodeTypeLipsSynchroRef    21
#define scriptNodeTypeFamilyRef         22
#define scriptNodeTypeActorRef          23
#define scriptNodeTypeActionRef         24
#define scriptNodeTypeSuperObjectRef    25
#define scriptNodeTypeSOLinksRef        26 /* ? */
#define scriptNodeTypeWaypointRef       27
#define scriptNodeTypeTextRef           28
#define scriptNodeTypeBehaviorRef       29
#define scriptNodeTypeModuleRef2        30
#define scriptNodeTypeSoundEventRef     31
#define scriptNodeTypeObjectTableRef    32
#define scriptNodeTypeGameMaterialRef   33
#define scriptNodeTypeVisualMaterial    34
#define scriptNodeTypeParticleGenerator 35
#define scriptNodeTypeModelRef          36
#define scriptNodeTypeModelRef2         37
#define scriptNodeTypeCustomBits        38
#define scriptNodeTypeCaps              39
#define scriptNodeTypeGraph             40 /* ? */
#define scriptNodeTypeSubroutine        41
#define scriptNodeTypeNULL              42
#define scriptNodeTypeCineRef           43 /* ? */
#define scriptNodeTypeGraphRef          44

#pragma mark Collide element types
#define collideElementTypeIndexedTriangles    1
#define collideElementTypeFacemap             2
#define collideElementTypeSprite              3
#define collideElementTypeTMesh               4
#define collideElementTypePoints              5
#define collideElementTypeLines               6
#define collideElementTypeIndexedSpheres      7
#define collideElementTypeAABB                8
#define collideElementTypeCones               9
#define collideElementTypeDeformationSetInfo  13
#define collideElementTypeInvalid             0xFFFF

#pragma mark Dynamics obstacle types
#define dynamicsObstacleTypeNoObstacle     0x00000000
#define dynamicsObstacleTypeGround         0x00000001
#define dynamicsObstacleTypeWall           0x00000004
#define dynamicsObstacleTypeCeiling        0x00000010
#define dynamicsObstacleTypeWater          0x00000040
#define dynamicsObstacleTypeForceMobile    0x00000080
#define dynamicsObstacleTypeMobile         0x00010000
#define dynamicsObstacleTypeError          0x80000000

#pragma mark - Pointers -

#define GCN_POINTER_ENGINE_STRUCTURE        0x803E7C0C
#define GCN_POINTER_INPUT_STRUCTURE         0x8042F5A8
#define GCN_POINTER_FIX                     0x804334CC
#define GCN_POINTER_LVL                     0x804334D0
#define GCN_POINTER_RND                     0x80436924
#define GCN_POINTER_GHOST_MODE              0x805D8580
#define GCN_POINTER_INACTIVE_DYNAMIC_WORLD  0x805D8594
#define GCN_POINTER_FATHER_SECTOR           0x805D8598
#define GCN_POINTER_DYNAMIC_WORLD           0x805D859C
#define GCN_POINTER_ACTUAL_WORLD            0x805D85A0
#define GCN_POINTER_MENU_SELECTION_V        0x805D884C
#define GCN_POINTER_MENU_RUMBLEPAD          0x805D89B0
