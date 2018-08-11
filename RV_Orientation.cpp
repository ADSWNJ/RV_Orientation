// ================================================================================================
//
//	Rendezvous Orientation MFD (RV_Orientation)
//	===========================================
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	Description:
//
//	Orbiter Simulator MFD and Heads-Up Display for rendezvous guidance control.
//	This MFD allows the user to efficiently navigate to an alignment point in line
//	with a target vessel and docking port, then traverse to complete the docking.
//	Also provided is a simple PID based autopilot to implement the docking. 
//
//	This MFD exploits Enjo's awesome multi-display buttons page code, and heads up display
//	code. It all runs on the Orbiter Space Flight Simulator provided by Dr Martin Schweiger.  
//
//	Copyright Notice:
//
//	This program is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	For full licencing terms, pleaserefer to the GNU General Public License
//	(gpl-3_0.txt) distributed with this release, or see
//	http://www.gnu.org/licenses/.
//
//
//	Credits:
//
//	Orbiter Simulator	(c) 2003-2013 Martin (Martins) Schweiger
// 	MFDButtonPage		(c) 2012-2013 Szymon (Enjo) Ender
// 	IDrawsHUD		(c) 2013 Szymon (Enjo) Ender
//	
//
//	Release History:
//
//	V1.00	Initial Release
//	V1.01	Typos in the documentation, and fixed button descriptions
//  V2.00 Invariant thrusters (determining own orientation to ship, not to thruster mode)
//  V3.00 Multi-stage docking corridor, with numerous minor mods and enhancements throughout
//  V3.02 Minor feature: import target from LaunchMFD, thanks to my buddy Enjo's ModuleMessaging code!
//  V3.03 Minor feature: support Orion MPCV with interesting thruster configs
//  V3.04 updated for MM Ext
// ================================================================================================

#define STRICT
#define ORBITER_MODULE
#include "windows.h"
#include "orbitersdk.h"
#include "RV_Orientation.hpp"
#include "RVO_Cores.hpp"
#include "RVO_DialogFunc.hpp"
#include "MFDPersist.hpp"

// =======================================================================
// Global variables

RVO_GCore *g_SC;      // points to the static core, root of all persistence
int g_MFDmode;      // holds the mode identifier for our MFD


// =======================================================================
// API interface

DLLCLBK void InitModule (HINSTANCE hDLL)
{
	static char *name = "RV Orientation";   // MFD mode name
	MFDMODESPECEX spec;
	spec.name = name;
	spec.key = OAPI_KEY_T;                // MFD mode selection key
	spec.context = NULL;
	spec.msgproc = RV_Orientation::MsgProc;  // MFD mode callback function

	// Register the new MFD mode with Orbiter
	g_MFDmode = oapiRegisterMFDMode (spec);
}

DLLCLBK void ExitModule (HINSTANCE hDLL)
{
	// Unregister the custom MFD mode when the module is unloaded
	oapiUnregisterMFDMode (g_MFDmode);
}

DLLCLBK void opcPreStep(double SimT,double SimDT,double mjd) {
  double thisSimT = oapiGetSimTime();

  if (!g_SC) return;  // Static core not initialized

  g_SC->VC->UpdateOrientation();  // Update orientation for the focus core
}

// ==============================================================
// MFD class implementation
//
// ... derive also from EnjoLib::IDrawsHUD to hook to the HUD drawer interface


// Constructor
RV_Orientation::RV_Orientation (DWORD w, DWORD h, VESSEL *vessel, UINT mfd) : modMsg("RV_Orientation"), MFD2 (w, h, vessel), EnjoLib::IDrawsHUD()
{
  if (!g_SC) {
    g_SC = new RVO_GCore;                     // First time only for RV Orientation in this Orbiter session. Init the static core.
  }
  GC = g_SC;                                  // Make the RV_Orientation instance Global Core point to the static core. 

  VC = (RVO_VCore*) GC->P.FindVC(vessel);
  if (!VC) {
    VC = new RVO_VCore(vessel,GC);            // Init the vessel core once per focus vessel. 
    GC->P.AddVC(vessel, VC);
  }
  GC->VC = VC;                                // Store the focus vessel core

  MC = (RVO_MCore*) GC->P.FindMC(mfd);
  if (!MC) {
    MC = new RVO_MCore(mfd,GC);               // Init the MFD core once per MFD position (e.g. left, right, extern)
    GC->P.AddMC(mfd, MC);
  }

  LC = (RVO_LCore*) GC->P.FindLC(vessel, mfd);
  if (!LC) {
    LC = new RVO_LCore(vessel,mfd,GC);        // Init the local core once per vessel + mfd position
    GC->P.AddLC(vessel, mfd, LC);
  }

  font = oapiCreateFont (h/25, true, "Fixed", FONT_NORMAL, 0);


  ColorMap[0] = 0x00FF00;                     // Green
  ColorMap[1] = 0x00BFFF;                     // Amber
  ColorMap[2] = 0x0000FF;                     // Red

  GuidePen[0] = oapiCreatePen(1,3,0x00E5EE);
  GuidePen[1] = oapiCreatePen(1,3,0xEEE685);
  GuidePen[2] = oapiCreatePen(1,3,0xEED2EE);
  ArrowPen[0] = oapiCreatePen(1,1,ColorMap[0]);
  ArrowBrush[0] = oapiCreateBrush(ColorMap[0]);
  ArrowPen[1] = oapiCreatePen(1,1,ColorMap[1]);
  ArrowBrush[1] = oapiCreateBrush(ColorMap[1]);
  ArrowPen[2] = oapiCreatePen(1,1,ColorMap[2]);
  ArrowBrush[2] = oapiCreateBrush(ColorMap[2]);

// v3.02 feature .. import target from LaunchMFD if available
	if (VC->hTgtV == nullptr) {


    int toi;
    if (modMsg.Get("LaunchMFD","TargetObjectIndex", &toi)) {
			// Work with objIndex.value
			OBJHANDLE hTgt = oapiGetObjectByIndex(toi);
			if (hTgt && oapiIsVessel(hTgt)) {
				VESSEL * vTgt = oapiGetVesselInterface(hTgt);
				RVO_DialogFunc::clbkTGT(nullptr, vTgt->GetName(), LC);
				if (VC->hTgtV) {
					char tmpMsg[750];
					if (!LC->showMessage) LC->Message[0] = '\0';
					LC->showMessage = true;
					sprintf_s(tmpMsg,"LaunchMFD target %s set!\n\n\n%s", VC->TargetText, LC->Message);	
					sprintf_s(LC->Message,"%s", tmpMsg);
				}
			}
    }


/*  OLD MODULEMESSAGING ORIGINAL CODE
		EnjoLib::Result<int> objIndex = EnjoLib::ModuleMessaging().GetInt("LaunchMFD", "TargetObjectIndex");
		if (objIndex.isSuccess && objIndex.value >= 0) {
			// Work with objIndex.value
			OBJHANDLE hTgt = oapiGetObjectByIndex(objIndex.value);
			if (hTgt && oapiIsVessel(hTgt)) {
				VESSEL * vTgt = oapiGetVesselInterface(hTgt);
				RVO_DialogFunc::clbkTGT(nullptr, vTgt->GetName(), LC);
				if (VC->hTgtV) {
					char tmpMsg[750];
					if (!LC->showMessage) LC->Message[0] = '\0';
					LC->showMessage = true;
					sprintf_s(tmpMsg,"Target %s imported from LaunchMFD!\n\n\n%s", VC->TargetText, LC->Message);	
					sprintf_s(LC->Message,"%s", tmpMsg);
				}
			}
		}
*/


	}
}

// Destructor
RV_Orientation::~RV_Orientation ()
{
  return;
}


// ==============================================================
// MFD message parser
int RV_Orientation::MsgProc (UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam)
{
	switch (msg) {
	case OAPI_MSG_MFD_OPENED:
		// Our new MFD mode has been selected, so we create the MFD and
		// return a pointer to it.
		return (int)(new RV_Orientation (LOWORD(wparam), HIWORD(wparam), (VESSEL*)lparam, mfd));
	}
	return 0;
}



// ==============================================================
// Persistence functions
void RV_Orientation::ReadStatus(FILEHANDLE scn) {
  char *line;
  int param = 0;
  float paramf = 0.0;

  OBJHANDLE hTgtV;
  VESSEL* tv;

  while (oapiReadScenario_nextline(scn, line)) {
    if (!_strnicmp(line,"END_MFD",7)) {
      break;
    } else if (!_strnicmp(line, "RVO_TGT", 7)) {

      hTgtV = oapiGetVesselByName(line+8);
      if (!hTgtV) continue;                // String was not a vessel 

      tv = oapiGetVesselInterface(hTgtV);
      if (!tv) continue;                   // Couldn't find the vessel interface
      if (tv == LC->v) continue;           // We can't dock with ourself (that would be rude!)

      // Target accepted 
      strcpy_s(VC->TargetText, 50, tv->GetName());
      VC->hTgtV = hTgtV;
      VC->tv = tv;
      VC->MaxPorts = VC->tv->DockCount();

    } else if (!_strnicmp(line, "RVO_PRT", 7)) {
      if (sscanf_s(line+8,"%d",&param)) {
        if (param<VC->MaxPorts) {
          VC->PortNumber = param;
          VC->hDock = VC->tv->GetDockHandle(VC->PortNumber);
          VC->tv->GetDockParams(VC->hDock, VC->rtvdPos, VC->rtvdDir, VC->rtvdRot);
          VC->CreatePortRotMatrix(VC->rtvPortOri,VC->rtvdDir,VC->rtvdRot);
        }
      }
    } else if (!_strnicmp(line, "RVO_MOD", 7)) {
      if (sscanf_s(line+8,"%d",&param)) {
        VC->mode = param;
        if (VC->mode == -1) VC->apcMode = 0;
      }
    }  else if (!_strnicmp(line, "RVO_UNITS", 9)) {
      if (!_strnicmp(line+10, "US", 2)) {
        VC->units = false;
      } else if (!_strnicmp(line+10, "METRIC", 6)) {
        VC->units = true;
      }
    }  else if (!_strnicmp(line, "RVO_APA", 7)) {
      if (sscanf_s(line+8,"%d",&param)) {
        VC->apAppArmed = (param==1);
      }
    } else if (!_strnicmp(line, "RVO_DST", 7)) {
      if (sscanf_s(line+8,"%f",&paramf)) {
        VC->ApproachWaypointDistance = id(paramf);
        VC->ApproachWaypointOfsDistance = id(floor((ed(250.0)+25.0)/50.0)*50.0);
      }
   }  else if (!_strnicmp(line, "RVO_APR", 7)) {
      if (sscanf_s(line+8,"%d",&param)) {
        VC->apRotArmed = (param==1);
      }
    }  else if (!_strnicmp(line, "RVO_APT", 7)) {
      if (sscanf_s(line+8,"%d",&param)) {
        VC->apAttArmed = (param==1);
      }
    }



  }
  if (VC->mode == 3) {    // Can't do translation AP or approach on RVEL mode
    VC->apAppArmed = false;
    VC->apAttArmed = false;
  }
  return;
}

void RV_Orientation::WriteStatus(FILEHANDLE scn) const {
  char buf[128];

  oapiWriteScenario_string(scn, "RVO_TGT", VC->TargetText);
  sprintf_s(buf,"%d",VC->PortNumber);
  oapiWriteScenario_string(scn, "RVO_PRT", buf);
  sprintf_s(buf,"%s",(VC->units?"METRIC":"US"));
  oapiWriteScenario_string(scn, "RVO_UNITS", buf);

  sprintf_s(buf,"%.0f",ed(VC->ApproachWaypointDistance));
  oapiWriteScenario_string(scn, "RVO_DST", buf);
  sprintf_s(buf,"%d",VC->mode);
  oapiWriteScenario_string(scn, "RVO_MOD", buf);
  if (VC->apRotArmed) {
    oapiWriteScenario_string(scn, "RVO_APR", "1");
  } else {
    oapiWriteScenario_string(scn, "RVO_APR", "0");
  }
  if (VC->apAttArmed) {
    oapiWriteScenario_string(scn, "RVO_APT", "1");
  } else {
    oapiWriteScenario_string(scn, "RVO_APT", "0");
  }
  if (VC->apAppArmed) {
    oapiWriteScenario_string(scn, "RVO_APA", "1");
  } else {
    oapiWriteScenario_string(scn, "RVO_APA", "0");
  }

  return;
}



