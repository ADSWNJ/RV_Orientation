// ==============================================================
//
//	Rendezvous Orientation MFD (Dialog Function Handlers)
//	=====================================================
//
//	Copyright (C) 2013-2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==============================================================

#include "RVO_DialogFunc.hpp"
#include "RVO_Cores.hpp"



// Callback from Target Selection Input Box
bool RVO_DialogFunc::clbkTGT(void *id, char *str, void *usrdata) {

  OBJHANDLE hTgtV;
  VESSEL* tv;
  bool DockFree = false;
  
  RVO_LCore* LC = (RVO_LCore*) usrdata;
  RVO_GCore* GC = LC->GC;
  RVO_VCore* VC = LC->VC;

  if (strlen(str) == 0) return true;    // Empty string - assume canceled dialog

  hTgtV = oapiGetVesselByName(str);
  if (!hTgtV) return true;               // String was not a vessel 

  tv = oapiGetVesselInterface(hTgtV);
  if (!tv) return true;               // Couldn't find the vessel interface
  if (tv == LC->v) return true;       // We can't dock with ourself (that would be rude!)

  // Target accepted 
  strcpy_s(VC->TargetText, 50, tv->GetName());
  VC->hTgtV = hTgtV;
  VC->tv = tv;

  VC->MaxPorts = VC->tv->DockCount();

  for (int i=0; i<VC->MaxPorts; i++) {

    VC->hDock = VC->tv->GetDockHandle(i);
    if (!VC->hDock) continue;                  // Couldn't find the dock??
    if (VC->tv->DockingStatus(i)==0) { // dock free
      DockFree = true;
      VC->tv->GetDockParams(VC->hDock, VC->rtvdPos, VC->rtvdDir, VC->rtvdRot);
      VC->CreatePortRotMatrix(VC->rtvPortOri,VC->rtvdDir,VC->rtvdRot);
      VC->PortNumber = i;
      break;
    }
  }

  if (!DockFree) {
    LC->showMessage = true;
    sprintf_s(LC->Message,"No free ports on %s!", VC->TargetText);
    VC->PortNumber = -1;
    VC->hDock = 0;
  }
  
  return true;

}


// Callback from Approach Waypoint Distance Input Box
bool RVO_DialogFunc::clbkDST(void *id, char *str, void *usrdata) {
  
  int scanRet;
  float ApproachWaypointDistance; 

  if (strlen(str) == 0) return true;      // Empty string - assume canceled dialog
  
  RVO_LCore* LC = (RVO_LCore*) usrdata;
  RVO_GCore* GC = LC->GC;
  RVO_VCore* VC = LC->VC;


  scanRet = sscanf_s(str,"%f",&ApproachWaypointDistance);
  if (scanRet != 1) return true;          // String was not a decimal
   
  if (VC->units) {

    if (ApproachWaypointDistance < 50.0) ApproachWaypointDistance = 50.0;
    if (ApproachWaypointDistance > 100000.0) ApproachWaypointDistance = 100000.0;

    // Approach Waypoint Distance accepted
    VC->ApproachWaypointDistance = ApproachWaypointDistance;
    VC->ApproachWaypointOfsDistance = 250.0;

  } else {

    if (ApproachWaypointDistance < 150.0) ApproachWaypointDistance = 150.0;
    if (ApproachWaypointDistance > 300000.0) ApproachWaypointDistance = 300000.0;

    // Approach Waypoint Distance accepted
    VC->ApproachWaypointDistance = ApproachWaypointDistance/3.2808399;
    VC->ApproachWaypointOfsDistance = 825.0 / 3.2808399;
  }
  VC->apSWPselect = true;

  return true;

}


// Callback from HUD Perspective Parameter
bool RVO_DialogFunc::clbkHPP(void *id, char *str, void *usrdata) {
  
  int scanRet;
  int z_in; 

  if (strlen(str) == 0) return true;      // Empty string - assume canceled dialog
  
  RVO_LCore* LC = (RVO_LCore*) usrdata;
  RVO_GCore* GC = LC->GC;
  RVO_VCore* VC = LC->VC;


  scanRet = sscanf_s(str,"%d",&z_in);
  if (scanRet != 1) return true;          // Bad parse - ignore
   
  VC->hpp_z = 0.001 * z_in; 
  return true;

}
