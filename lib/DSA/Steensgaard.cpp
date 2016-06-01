//===- Steensgaard.cpp - Context Insensitive Data Structure Analysis ------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass computes a context-insensitive data analysis graph.  It does this
// by computing the local analysis graphs for all of the functions, then merging
// them together into a single big graph without cloning.
//
//===----------------------------------------------------------------------===//

#include "dsa/DataStructure.h"
#include "dsa/DSGraph.h"

#include "dsa/Steensgaard.hh"

#include "llvm/IR/Module.h"
#include "llvm/Support/Debug.h"

/// added for debugging
#include "llvm/ADT/DenseMap.h"
#include "llvm/Pass.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/Target/TargetLibraryInfo.h"
#include "llvm/Analysis/MemoryBuiltins.h"

using namespace llvm;

#define DSA_STEENS_DEBUG

#ifdef DSA_STEENS_DEBUG
namespace steens_debug {

  std::string type_to_str (DSNode *nn) {

    std::string type_str;
    llvm::raw_string_ostream rso(type_str);

    if (!nn)  {
      // do nothing
    } else if (nn->isNodeCompletelyFolded())  {
      rso << "  Types={collapsed";
    } else if (nn->hasNoType ())  {
      rso << "  Types={void";
    } else {
      rso << "  Types={";
      DSNode::type_iterator ii = nn->type_begin(), ie = nn->type_end();
      DSNode::type_iterator jj = ii;
      if (++jj == ie) {
        auto ty_set_ptr = ii->second;
        if (ty_set_ptr->size () == 1) {
          rso <<  **(ty_set_ptr->begin ());
        } else {
          svset<Type*>::const_iterator ki = (*ty_set_ptr).begin (), ke = (*ty_set_ptr).end ();
          rso << "[";
          for (; ki != ke; ) { 
            rso << **ki;
            ++ki;
            if (ki != ke) rso << " | ";
          }
          rso << "]";
        }
      }
      else {
        rso << "struct {";
        while (ii != ie) {
          rso << ii->first << ":";
          if (!ii->second) { 
            rso << "untyped";
          } else {
            auto ty_set_ptr = ii->second;
            if (ty_set_ptr->size () == 1) {
              rso << **(ty_set_ptr->begin ());
            } else {
              svset<Type*>::const_iterator ki = ty_set_ptr->begin (), ke = ty_set_ptr->end ();
              rso << "[";
              for (; ki != ke; ) { 
                rso << **ki;
                ++ki;
                if (ki != ke) rso << " | ";
              }
              rso << "]";
            }
          }
          ++ii;
          if (ii != ie)
            rso << ",";   
        }
        rso << "}*";
      }
    }
    rso << "}\n";
    return rso.str();
  }

  // if < 0 then the  node has no type
  // if > 0  then return the number of types
  int getNumTypes (DSNode *nn) {
    if (!nn)  {
      return 0;
    } else {
      // FIXME: this is not actually a good measure of the number of
      // types
      return std::distance(nn->type_begin(), nn->type_end());
    }
  }

  bool equalTypes (DSNode *N1 , DSNode *N2) {
    return (type_to_str(N1) == type_to_str(N2));
  }

  bool compareDSGraph(DSGraph* G1, DSGraph* G2, Value* Use) {
    DSScalarMap& SM1 = G1->getScalarMap ();
    DSScalarMap& SM2 = G2->getScalarMap ();
    for (typename DSScalarMap::iterator it = SM1.begin (), et = SM1.end (); it!=et;++it){
      DSNodeHandle DH1 = it->second; 
      DSNodeHandle DH2 = SM2 [it->first];
      if (!DH1.isForwarding ()) {
        DSNode* NN1 = DH1.getNode ();
        DSNode* NN2 = DH2.getNode ();
        if ( NN1 && NN2 && (!NN1->hasNoType ()) && (NN2->hasNoType ())) {
          errs () << "\t--- Node became untyped \n";
          NN1->dump (); 
          NN2->dump (); 
          return false;
        }
        else if (!equalTypes(NN1,NN2)) {
          errs () << "\t--- Node changed its types\n";
          NN1->dump (); 
          NN2->dump (); 
          return false;
        }
      }
    }
    return true;
  }
}

bool IsStaticallyKnown (const Value* V, 
                        const TargetLibraryInfo * tli,
                        const DataLayout* dl) {
  uint64_t Size;
  if (getObjectSize (V, Size, dl, tli, true))
    return (Size > 0);
  
  return false; 
}

struct DSNodeStats {
  unsigned ID;
  unsigned Accesses;
  const DSNode* N;

  //DSNodeStats(): N(nullptr), ID(0), Accesses(0) { }

  DSNodeStats(const DSNode*_N, unsigned _ID): ID(_ID), Accesses(0), N(_N)  { }

  bool operator<(const DSNodeStats& o) const {
    return Accesses < o.Accesses;
  }

  bool operator==(const DSNodeStats& o) const {
    return N == o.N;
  }
};

void printAccesses(DenseMap<const DSNode*, DSNodeStats >& acc_map) {
  std::vector<DSNodeStats> V;
  V.reserve(acc_map.size());
  for (auto p: acc_map) V.push_back(p.second);
    
  std::sort (V.begin (), V.end ());
  std::reverse(V.begin(), V.end());
  const unsigned threshold = 5;
  unsigned c =0;
  //errs () << "Printing the " << threshold << " most accessed DSNodes ... \n";
  for (auto N : V){
    //if (c >= threshold) break;
    if (N.Accesses == 0) continue ;
    errs () << "[Node ID " << N.ID << "] " << "Accesses=" << N.Accesses;
    N.N->dump();
    c++;
  }
}

void countValAcc (Value* V, DSGraph* G, 
                  const TargetLibraryInfo * tli, const DataLayout* dl,
                  DenseMap<const DSNode*, DSNodeStats>& acc_map) {

  const DSNode* n = G->getNodeForValue (V).getNode ();
  if (!n) return;

  if (acc_map.find (n) == acc_map.end ()) {
    DSNodeStats N(n, acc_map.size()+1);
    acc_map.insert(std::make_pair(n, N));
  }

  auto It = acc_map.find (n);
  if (It != acc_map.end () && !IsStaticallyKnown (V, tli, dl))
    It->second.Accesses++;
}

void countFuncAcc (Function &f, DSGraph* G, 
                   const TargetLibraryInfo * tli, const DataLayout* dl,
                   DenseMap<const DSNode*,  DSNodeStats>& acc_map) {
  Module* M = f.getParent();
  for (Function &F: *M) {
    for (inst_iterator i = inst_begin(F), e = inst_end(F); i != e; ++i)  {
      Instruction *I = &*i;
      if (LoadInst *LI = dyn_cast<LoadInst>(I)) {
        countValAcc (LI->getPointerOperand (), G, tli, dl, acc_map);
      } else if (StoreInst *SI = dyn_cast<StoreInst>(I)) {
        countValAcc (SI->getPointerOperand (), G, tli, dl,acc_map);
      } else if (MemTransferInst *MTI = dyn_cast<MemTransferInst>(I)) {
        countValAcc (MTI->getDest (), G, tli, dl, acc_map);
        countValAcc (MTI->getSource (), G, tli, dl, acc_map);
      } else if (MemSetInst *MSI = dyn_cast<MemSetInst>(I)) {
        countValAcc (MSI->getDest (), G, tli, dl, acc_map);
      }   
    }
  }
}

#endif 

void SteensgaardDataStructures::releaseMemory() 
{
  delete ResultGraph; 
  ResultGraph = 0;
  DataStructures::releaseMemory();
}


void SteensgaardDataStructures::print(llvm::raw_ostream &O, const Module *M) const {
  assert(ResultGraph && "Result graph has not yet been computed!");
  ResultGraph->writeGraphToFile(O, "steensgaards");
}

/// run - Build up the result graph, representing the pointer graph for the
/// program.
///
bool SteensgaardDataStructures::runOnModule(Module &M) 
{
  DS = &getAnalysis<StdLibDataStructures>();
  init (&getAnalysis<DataLayoutPass>().getDataLayout ());

#ifdef DSA_STEENS_DEBUG
  errs () << "Before Steensgaard\n";
  errs () << M << "\n";
#endif 
  return runOnModuleInternal(M);
}

bool SteensgaardDataStructures::runOnModuleInternal (Module &M) 
{
  assert(ResultGraph == 0 && "Result graph already allocated!");
  
  // Get a copy for the globals graph.
  DSGraph * GG = DS->getGlobalsGraph();
  GlobalsGraph = new DSGraph(GG, GG->getGlobalECs(), *TypeSS, 0);

  // Create a new, empty, graph...
  ResultGraph = new DSGraph(GG->getGlobalECs(), getDataLayout(), 
                            *TypeSS, GlobalsGraph);
  
  // Loop over the rest of the module, merging graphs for non-external functions
  // into this graph.
  //
  for (const Function &F : M)
    if (!F.isDeclaration())  {
#ifdef DSA_STEENS_DEBUG
      errs () << "Begin Merging graph from " << F.getName () << "\n";
#endif 
      ResultGraph->spliceFrom(DS->getDSGraph(F));
#ifdef DSA_STEENS_DEBUG
      errs () << "End Merging graph from " << F.getName () << "\n";
#endif 
    }

  ResultGraph->removeTriviallyDeadNodes();

  // FIXME: Must recalculate and use the Incomplete markers!!

  // Now that we have all of the graphs inlined, we can go about eliminating
  // call nodes...
  //

  // Start with a copy of the original call sites.
  DSGraph::FunctionListTy &Calls = ResultGraph->getFunctionCalls ();
  std::vector<const Function*> CallTargets;
  for (std::list<DSCallSite>::iterator CI = Calls.begin(), E = Calls.end();
       CI != E;) 
  {
    DSCallSite &CurCall = *CI++;
    // Loop over the called functions, eliminating as many as possible...
    CallTargets.clear ();
    if (CurCall.isDirectCall ())
      CallTargets.push_back (CurCall.getCalleeFunc());
    else
      CurCall.getCalleeNode()->addFullFunctionList(CallTargets);

    for (unsigned c = 0; c != CallTargets.size(); ) 
    {
      // If we can eliminate this function call, do so!
      const Function *F = CallTargets [c];
      if (!F->isDeclaration()) 
      {
        ResolveFunctionCall(F, CurCall, ResultGraph->getReturnNodes()[F]);
        CallTargets[c] = CallTargets.back();
        CallTargets.pop_back();
      } 
      else
        ++c;  // Cannot eliminate this call, skip over it...
    }

    if (CallTargets.empty()) {        // Eliminated all calls?
      std::list<DSCallSite>::iterator I = CI;
      Calls.erase(--I);               // Remove entry
    }
  }

  /// AG: Need to keep the return nodes so that we can map a call-site
  // in a caller to the callee. Not sure what this does to the
  // incompleteness markers.

  // Remove our knowledge of what the return
  // values of the functions are, except for functions that are
  // externally visible from this module (e.g. main).  We keep these
  // functions so that their arguments are marked incomplete.  for
  // (DSGraph::ReturnNodesTy::iterator I =
  // ResultGraph->getReturnNodes().begin(), E =
  // ResultGraph->getReturnNodes().end(); I != E; ) if
  // (I->first->hasInternalLinkage())
  // ResultGraph->getReturnNodes().erase(I++); else ++I;

  // Update the "incomplete" markers on the nodes, ignoring unknownness due to
  // incoming arguments...
  ResultGraph->maskIncompleteMarkers();

  ResultGraph->markIncompleteNodes(DSGraph::MarkFormalArgs | DSGraph::IgnoreGlobals);

  // Remove any nodes that are dead after all of the merging we have done...

  ResultGraph->removeDeadNodes(DSGraph::KeepUnreachableGlobals);

  GlobalsGraph->removeTriviallyDeadNodes();
  GlobalsGraph->maskIncompleteMarkers();

  // Mark external globals incomplete.
  GlobalsGraph->markIncompleteNodes(DSGraph::IgnoreGlobals);

  formGlobalECs();

  // Clone the global nodes into this graph.
  ReachabilityCloner RC(ResultGraph, GlobalsGraph,
                        DSGraph::DontCloneCallNodes |
                        DSGraph::DontCloneAuxCallNodes);
  for (DSScalarMap::global_iterator I = GlobalsGraph->getScalarMap().global_begin(),
         E = GlobalsGraph->getScalarMap().global_end(); I != E; ++I)
    if (isa<GlobalVariable>(*I) || isa<Function> (*I))
      RC.getClonedNH(GlobalsGraph->getNodeForValue(*I));
   
  //ResultGraph->writeGraphToFile (errs (), "Module.st");
  
  return false;
}


/// ResolveFunctionCall - Resolve the actual arguments of a call to function F
/// with the specified call site descriptor.  This function links the arguments
/// and the return value for the call site context-insensitively.
///
void
SteensgaardDataStructures::ResolveFunctionCall(const Function *F, 
                                               const DSCallSite &Call,
                                               DSNodeHandle &RetVal) 
{
  assert(ResultGraph != 0 && "Result graph not allocated!");

#ifdef DSA_STEENS_DEBUG
  errs () << "BEGIN STEENSGAARD \n"
          << "Merging nodes at " << *(Call.getCallSite().getInstruction()) << "\n"
          << "Caller=" << F->getName () << "\n";
#endif 

  DSGraph::ScalarMapTy &ValMap = ResultGraph->getScalarMap();

  // Handle the return value of the function...
  if (Call.getRetVal().getNode() && RetVal.getNode()) {
    #ifdef DSA_STEENS_DEBUG
    errs () << "\t RETURN VALUE\n"
            << "\t Actual " << RetVal.getNode () 
            << " (refs=" << RetVal.getNode ()->getNumReferrers() << ") "
            << steens_debug::type_to_str (RetVal.getNode ())
            << "\t Formal " << Call.getRetVal().getNode() 
            << " (refs=" << Call.getRetVal().getNode()->getNumReferrers() << ") "
            << steens_debug::type_to_str (Call.getRetVal().getNode());
    #endif 
    RetVal.mergeWith (Call.getRetVal());
    #ifdef DSA_STEENS_DEBUG
    errs () << "\t RESULT " << RetVal.getNode ()
            << " (refs=" << RetVal.getNode ()->getNumReferrers() << ") "
            << steens_debug::type_to_str(RetVal.getNode ()); 
    #endif 

  }

  // Loop over all pointer arguments, resolving them to their provided pointers
  unsigned PtrArgIdx = 0;
  #ifdef DSA_STEENS_DEBUG
  unsigned ArgIdx=0;
  #endif 
  for (Function::const_arg_iterator AI = F->arg_begin(), AE = F->arg_end();
       AI != AE && PtrArgIdx < Call.getNumPtrArgs(); ++AI) 
  {
    DSGraph::ScalarMapTy::iterator I = ValMap.find(AI);
    if (I != ValMap.end()) {   // If its a pointer argument...
      // errs () << I->second.getNode () ;

      #ifdef DSA_STEENS_DEBUG
      errs () << "\tPARAM " << ArgIdx+1 << "-th MERGING \n"
              << "\t Actual " << I->second.getNode () 
              << " (refs=" << I->second.getNode ()->getNumReferrers() << ") "
              << steens_debug::type_to_str (I->second.getNode ())
              << "\t Formal " << Call.getPtrArg(PtrArgIdx).getNode() 
              << " (refs=" << Call.getPtrArg(PtrArgIdx).getNode()->getNumReferrers() << ") "
              << steens_debug::type_to_str (Call.getPtrArg(PtrArgIdx).getNode());
      #endif 
      I->second.mergeWith (Call.getPtrArg(PtrArgIdx++));
      #ifdef DSA_STEENS_DEBUG
      errs () << "\t RESULT " << I->second.getNode ()
              << " (refs=" << I->second.getNode ()->getNumReferrers() << ") "
              << steens_debug::type_to_str(I->second.getNode ()); 
      #endif 
    }
    #ifdef DSA_STEENS_DEBUG
    ArgIdx++;
    #endif 
  }

#ifdef DSA_STEENS_DEBUG
  const TargetLibraryInfo * tli = &getAnalysis<TargetLibraryInfo>();
  const DataLayout* dl = &getAnalysis<DataLayoutPass>().getDataLayout ();
  DenseMap<const DSNode*, DSNodeStats> acc_map;
  Function * FF = const_cast<Function*>(F);
  countFuncAcc (*FF, ResultGraph, tli, dl, acc_map);
  printAccesses (acc_map);  
#endif 

#ifdef DSA_STEENS_DEBUG
  errs () << "END ----------- \n";
#endif 
}

void SteensgaardDataStructures::getAnalysisUsage(AnalysisUsage &AU) const
{
  AU.addRequired<DataLayoutPass>();
  AU.addRequired<StdLibDataStructures>();
  AU.addRequired<llvm::DataLayoutPass>();
  AU.addRequired<llvm::TargetLibraryInfo>();
  AU.setPreservesAll();
}

char SteensgaardDataStructures::ID = 0;

// Publicly exposed interface to pass...
char &llvm::SteensgaardDataStructuresID = SteensgaardDataStructures::ID;

// Register the pass...
static RegisterPass<SteensgaardDataStructures> X
("dsa-steens",
 "Context-insensitive Data Structure Analysis");
