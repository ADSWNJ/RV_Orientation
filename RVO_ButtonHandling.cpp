// ==============================================================
//
//	Rendezvous Orientation MFD (Button Handling Code)
//	=============================================
//
//	Copyright (C) 2013-2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==============================================================

#include "RV_Orientation.hpp"
#include "RVO_DialogFunc.hpp"

// ==============================================================
// MFD button hooks to Button Page library
//
char *RV_Orientation::ButtonLabel (int bt)
{
	return LC->B.ButtonLabel(bt);
}

// Return button menus
int RV_Orientation::ButtonMenu (const MFDBUTTONMENU **menu) const
{
	return LC->B.ButtonMenu(menu);
}

// Return clicked button
bool RV_Orientation::ConsumeButton (int bt, int event) {
  return LC->B.ConsumeButton(this, bt, event);
}

// Return pressed keystroke
bool RV_Orientation::ConsumeKeyBuffered (DWORD key) {
  return LC->B.ConsumeKeyBuffered(this, key);
}



// ==============================================================
// MFD Button Handler Callbacks
//


// TGT = Select Target Vessel for docking
void RV_Orientation::Button_TGT() {
  oapiOpenInputBox( "Enter RV Target Vessel",RVO_DialogFunc::clbkTGT, VC->TargetText, 30, LC);
  return;
}

// DPT = Select next free docking port
void RV_Orientation::Button_PRT() {

  if (!(VC->hTgtV)) {
    LC->showMessage = true;
    sprintf_s(LC->Message,"Error!\n\nPRT inactive until TGT selected.");
    VC->PortNumber = -1;
    VC->hDock = 0;
    return;
  }

  int origPort = VC->PortNumber;
  bool DockFree = false;

  do {
    VC->PortNumber = (VC->PortNumber + 1) % VC->MaxPorts;
    VC->hDock = VC->tv->GetDockHandle(VC->PortNumber);
    if (!VC->hDock) continue;                  // Couldn't find the dock??
    if (VC->tv->DockingStatus(VC->PortNumber)==0) { // dock free
      VC->tv->GetDockParams(VC->hDock, VC->rtvdPos, VC->rtvdDir, VC->rtvdRot);
      VC->CreatePortRotMatrix(VC->rtvPortOri,VC->rtvdDir,VC->rtvdRot);
      DockFree = true;
      if (VC->mode < 2) VC->mode = 2;  // If WP, select SWP
      VC->apSWPselect = true;
      break;
    }
  } while (VC->PortNumber != origPort);

  if (!DockFree) {
    LC->showMessage = true;
    sprintf_s(LC->Message,"Error!\n\nNo free ports on %s!", VC->TargetText);
    VC->PortNumber = -1;
    VC->hDock = 0;
  }

  return;
};

// HUD = Toggle HUD 0, 1, 2 (0 = off, 1 = arrows, 2 = arrows and nav boxes)
void RV_Orientation::Button_HUD() {
  VC->showHUD = (VC->showHUD+1)%3;
  return;
};

// MOD = Select tracking mode (i.e. track to RVel, track to alignment waypoint, or track to dock)
void RV_Orientation::Button_MOD() {
  if (!VC->hTgtV) {
    LC->showMessage = true;
    sprintf_s(LC->Message,"Error!\n\nMOD inactive until TGT selected.");
    return;
  }
  VC->apAppArmed = false;
  VC->apAttArmed = false;
  VC->apRotArmed = false;
  VC->AllStopThrust();
  if (VC->apApcArmed) {
    VC->apApcArmed = false;
    VC->mode = VC->apcMode;
  }
  VC->mode = (VC->mode+1)%4;
  if (VC->mode==1) VC->mode = 2;
  VC->firstCalc = true; // Need 2 iterations again for the rate calcs
  VC->apSWPselect = true;
  return;
};

// DST = Set Waypoint Distance - e.g. 500m out, 2000m out...
void RV_Orientation::Button_DST() {
  char buf[30];
  sprintf_s(buf,30,"%.0f",ed(VC->ApproachWaypointDistance));

  if (VC->units) {
    oapiOpenInputBox( "Enter approach waypoint distance (50 to 100,000 meters)",RVO_DialogFunc::clbkDST, buf, 30, LC);
  } else {
    oapiOpenInputBox( "Enter approach waypoint distance (150 to 300,000 feet)",RVO_DialogFunc::clbkDST, buf, 30, LC);
  }
  return;
};

// HPP = HUD Perspective Parameters Entry ... playing with the HUD FOV scale and Z distance
void RV_Orientation::Button_HPP() {
  char buf[30];
  sprintf_s(buf,30,"%.0f",VC->hpp_z*1000.0);

  oapiOpenInputBox( "Enter HUD Perspective Param Z Dist (e.g. 1000) (int only, in mm)",RVO_DialogFunc::clbkHPP, buf, 30, LC);
  return;
};

// UNT = Set units (decimal feet or decimal meters)
void RV_Orientation::Button_UNT() {
  VC->units = !(VC->units);
  LC->showMessage = true;
  if (VC->units) {
    strcpy_s(LC->Message,128,"Units set to METRIC.\n\nDistances are now in meters, rates in m/s.");
  } else {
    strcpy_s(LC->Message,128,"Units set to US.\n\nDistances are now in feet, rates in ft/s.");
  }
  VC->ApproachWaypointDistance = id(floor((ed(VC->ApproachWaypointDistance)+50.0)/100.0)*100.0);
  VC->ApproachWaypointOfsDistance = id(floor((ed(250.0)+25.0)/50.0)*50.0);
  VC->firstCalc = true; // Need 2 iterations again for the rate calcs
  VC->apSWPselect = true;
  return;
};

// APR = Set autopilot rotation mode (lock rotations on target)
void RV_Orientation::Button_APR() {

  if (!VC->hTgtV) {
    LC->showMessage = true;
    sprintf_s(LC->Message,"Error!\n\nAPR inactive until TGT selected.");
    return;
  }

  if (VC->v->GroundContact()) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nCannot engage APR on the ground!");
    return;
  }

  if (!VC->apRefRatesLoaded) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nNo reference thruster parameters found for this ship class.\n\nPlease hit APC to auto-calibrate.");
    return;
  }

  if (!VC->apRotArmed && (VC->v->GetDockStatus(VC->hMyDock) !=0)) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nCannot engage APR while docked!");
    VC->apAppArmed = false;
    VC->apAttArmed = false;
    VC->apRotArmed = false;
    return;
  };

  VC->apRotArmed = !VC->apRotArmed;

  if (!VC->apRotArmed) {                                          // Clean out any rotations when disarming ROT AP
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0.0);
    if (VC->apAppArmed) {
      VC->v->SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0.0);
      VC->v->SetThrusterGroupLevel(THGROUP_ATT_BACK, 0.0);
      VC->apAppArmed = false;                                    // disarm approach AP if ROT AP is deselected 
    }
  } else {
    VC->apSWPselect = true;
  }

  return;
};

// APT = Set autopilot transation mode (lock L-R, U-D translations on target)
void RV_Orientation::Button_APT() {

  if (!VC->hTgtV) {
    LC->showMessage = true;
    sprintf_s(LC->Message,"Error!\n\nAPT inactive until TGT selected.");
    return;
  }

  if (VC->v->GroundContact()) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nCannot engage APT on the ground!");
    return;
  }

  if (!VC->apRefRatesLoaded) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nNo reference thruster parameters found for this ship class.\n\nPlease hit APC to auto-calibrate.");
    return;
  }

  if ((VC->mode == 3) && (!VC->apAttArmed)) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nAPT not active in RVEL mode!");
    return;
  }

  if (!VC->apAttArmed && (VC->v->GetDockStatus(VC->hMyDock) !=0)) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nCannot engage APT while docked!");
    VC->apAppArmed = false;
    VC->apAttArmed = false;
    VC->apRotArmed = false;
    return;
  };

  VC->apAttArmed = !VC->apAttArmed;
  if (!VC->apAttArmed) {                                          // Clean out any translations when disarming ATT AP
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_UP, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0.0);
    VC->apAppArmed = false;                                     // disarm approach AP if attitude AP is deselected 
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_BACK, 0.0);
  } else {
    VC->apSWPselect = true;
  }
  return;
};

// APP = Set autopilot approach mode (approach target if APR and APT are engaged)
void RV_Orientation::Button_APP() {

  if (VC->v->GroundContact()) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nCannot engage APP on the ground!");
    return;
  }

  if (!VC->hTgtV) {
    LC->showMessage = true;
    sprintf_s(LC->Message,"Error!\n\nAPP inactive until TGT selected.");
    return;
  }

  if (!VC->apRefRatesLoaded) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nNo reference thruster parameters found for this ship class.\n\nPlease hit APC to auto-calibrate.");
    return;
  }

  if (VC->apAppArmed) {
    VC->apAppArmed = false;
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0.0);
    VC->v->SetThrusterGroupLevel(THGROUP_ATT_BACK, 0.0);
  } else {
    if (VC->v->GetDockStatus(VC->hMyDock) !=0) {
      LC->showMessage = true;
      strcpy_s(LC->Message,128,"Error!\n\nCannot engage APP while docked!");
      VC->apAppArmed = false;
      VC->apAttArmed = false;
      VC->apRotArmed = false;
      return;
    };
    VC->apRotArmed = true;
    VC->apAttArmed = true;
    VC->apAppArmed = true;
    VC->apWPreached = false;
    VC->mode=2; // Approach sphere first. 
    VC->apSWPselect = true;

  }
  return;
};

// APC = AutoPilot Calibration
void RV_Orientation::Button_APC() {
  if (!VC->hTgtV) {
    LC->showMessage = true;
    sprintf_s(LC->Message,"Error!\n\nPlease select TGT first.");
    return;
  }

  if (VC->v->GetDockStatus(VC->hMyDock) !=0) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nCannot engage APC while docked!");
    return;
  }

  if (VC->v->GroundContact()) {
    LC->showMessage = true;
    strcpy_s(LC->Message,128,"Error!\n\nCannot engage APC on the ground!");
    return;
  }
  if (!VC->apRefRatesLoaded) {
    Button_OVR();
  } else {
    LC->showMessage = true;
    LC->okMessagePage = false;
    strcpy_s(LC->Message,512,"CALIBRATION WARNING!\n\nThis ship class is already fully calibrated. Press CAN to cancel without changing the calibration, or OVR to override this warning and force a recalibration.");
    LC->B.SwitchPage(this,3);
  }
  return;
}
// APC = AutoPilot Calibration Cancel after Override confirmation
void RV_Orientation::Button_CAN() {
  LC->B.SwitchPage(this,0);
  LC->showMessage = false;
  LC->okMessagePage = true;
  return;
}
// APC = AutoPilot Calibration Proceed after Override confirmation
void RV_Orientation::Button_OVR() {
  LC->B.SwitchPage(this,4);
  LC->showMessage = true;
  LC->okMessagePage = false;
  strcpy_s(LC->Message,512,"CALIBRATION INSTRUCTIONS!\n\nPlease set your RCS thrusters in docking config if necessary (e.g. XR-5 style). The ship will purge and reset fuel loads several times in calibration - please ignore any fuel warnings (or turn down your speakers!).All calibration is in METRIC units.\n\nPres GO to start calibration run.");
  return;
}
// APC = AutoPilot Calibration GO ... after pre-calibration warnings
void RV_Orientation::Button_GO() {
  LC->B.SwitchPage(this,2);
  LC->showMessage = false;
  LC->okMessagePage = true;
  VC->apcMode = VC->mode;
  VC->mode = -1;
  VC->apApcArmed = true;
  VC->apAppArmed = false;
  VC->apAttArmed = false;
  VC->apRotArmed = false;
  VC->apCalPhase = 0;
  VC->apCalStep  = 0;
  VC->apCalPass = 0;
  VC->apCalInit = false;
  VC->apCalComplete = false;
  return;
};

// APD = Toggle dump of autopilot parameters on / off
void RV_Orientation::Button_APD() {
  if (VC->dumpAP) {
    fclose(VC->apD);
  } else {
    VC->apD_err = fopen_s(&(VC->apD), ".\\Config\\MFD\\RVO\\RVO_AP_Dump.csv","w");
    if (VC->apD_err) {
      LC->showMessage = true;
      sprintf_s(LC->Message,"Error!\n\nError %d opening AP dump file!", VC->apD_err);
      return; // Couldn't open dump file
    }

    fprintf_s(VC->apD, "SIMT,STEPAVG,Dx,Dx.V,Dx.L,Dx.Th,Dx.C,Dx.A,Dx.B,Dy,Dy.V,Dy.L,Dy.Th,Dy.C,Dy.A,Dy.B,Dz,Dz.V,Dz.L,Dz.Th,Dz.C,Dz.A,DZ.B,Px,Px.V,Px.L,Px.Th,Px.C,Px.A,Px.B,Py,Py.V,Py.L,Py.Th,Py.C,Py.A,Py.B,Pz,Pz.V,Pz.L,Pz.Th,Pz.C,Pz.A,Pz.B,Pz.Tgt,AppInCone,MaxDirErr,MaxPosErr,RotActive,AttActive,AppActive,AppAttAllow,ApToggle,  Range,OurLat,WPLat,OurLong,WPLong,Mode\n");
    VC->dumpSimT = VC->simT;
  }
  VC->dumpAP = !VC->dumpAP;

  // Temp dump of guidance
  if (VC->guideRectDump) {
    fclose(VC->gD);
  } else {
    VC->gD_err = fopen_s(&(VC->gD), ".\\Config\\MFD\\RVO\\RVO_Guidance_Dump.csv","w");
    if (VC->gD_err) {
      LC->showMessage = true;
      sprintf_s(LC->Message,"Error!\n\nError %d opening Guidance dump file!", VC->gD_err);
      return; // Couldn't open dump file
    }

  }
  VC->guideRectDump = !VC->guideRectDump;
  return;
};



// OK - acknowledge message on MFD to flip back
void RV_Orientation::Button_OK() {
  LC->B.SwitchPage(this,0);
  LC->showMessage = false;
  VC->apCalComplete = false;
  return;
};
