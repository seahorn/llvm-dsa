add_llvm_library (AssistDS
  ArgCast.cpp
  Devirt.cpp
  GEPExprArgs.cpp
  LoadArgs.cpp
  SimplifyExtractValue.cpp
  StructReturnToPointer.cpp
  ArgSimplify.cpp
  DynCount.cpp
  IndCloner.cpp
  SimplifyGEP.cpp
  TypeChecks.cpp
  DSNodeEquivs.cpp
  FuncSimplify.cpp
  Int2PtrCmp.cpp
  MergeGEP.cpp
  SimplifyInsertValue.cpp
  TypeChecksOpt.cpp
  DataStructureCallGraph.cpp
  FuncSpec.cpp
  SVADevirt.cpp
  SimplifyLoad.cpp)

add_definitions(-fPIC)