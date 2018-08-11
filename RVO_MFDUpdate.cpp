// ==============================================================
//
//	Rendezvous Orientation MFD (MFD Update)
//	=============================================
//
//	Copyright (C) 2013-2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==============================================================

#include "RV_Orientation.hpp"

bool RV_Orientation::Update (oapi::Sketchpad *skp)
{
  if (VC->apApcArmed) {
    if (VC->apCalInit) {
      return CalibrationUpdate( skp );
    } else {
      return true;
    }
  } else if (VC->apCalComplete) {
    LC->B.SwitchPage(this,1);
    return CalibrationUpdate( skp );
  }

  Title (skp, "RV Orientation 3.05");
	// Draws the MFD title

  int l = 3;
  char buf[128];

	skp->SetTextAlign (oapi::Sketchpad::LEFT, oapi::Sketchpad::BOTTOM);
	skp->SetTextColor (0x00FFFF);



  if (LC->showMessage) {
    ShowMessage(skp);
    if (LC->okMessagePage) LC->B.SwitchPage(this,1);
    return true;
  }

  if (VC->apAppArmed) {
    strcpy_s(buf, 128, ">> APP active <<"); 
  	skp->Text (Col(3), Line(l), buf, strlen(buf));
  }
  if (VC->hTgtV != 0) {
    sprintf_s(buf, 128, "Tgt:  %s", VC->TargetText );
  } else {
    sprintf_s(buf, 128, "Tgt:  %s", "<not set>");
  }
	skp->Text (Col(0), Line(l++), buf, strlen(buf));

  if (VC->apRotArmed) {
    strcpy_s(buf, 128, ">> APR active <<");
  	skp->Text (Col(3), Line(l), buf, strlen(buf));
  }
  if (VC->PortNumber != -1) {
    sprintf_s(buf, 128, "Port: %i", VC->PortNumber+1 );
  } else {
    sprintf_s(buf, 128, "Port: %s", "<not set>");
  }
	skp->Text (Col(0), Line(l++), buf, strlen(buf));

  if (VC->apAttArmed) {
    strcpy_s(buf, 128, ">> APT active <<");
  	skp->Text (Col(3), Line(l), buf, strlen(buf));
  }
  switch (VC->mode) {
    case -1:
      sprintf_s(buf, 128, "Mode: %s", "APC");
      break;
    case 0:
      sprintf_s(buf, 128, "Mode: %s", "DOCK");
      break;
    case 1:
      sprintf_s(buf, 128, "Mode: %s", "WP");
      break;
    case 2:
      sprintf_s(buf, 128, "Mode: %s.%.0f.%.0f", "SWP", VC->rtvWPlat[VC->curWP], VC->rtvWPlong[VC->curWP]);
      break;
    case 3:
      sprintf_s(buf, 128, "Mode: %s", "RVEL");
      break;
  }
	skp->Text (Col(0), Line(l++), buf, strlen(buf));
  
  if (VC->dumpAP) {
    strcpy_s(buf, 128, ">> AP dump <<");
  	skp->Text (Col(3), Line(l), buf, strlen(buf));
  }

  sprintf_s(buf, 128, "Dist: %.0f %s", ed(VC->ApproachWaypointDistance), (VC->units? "m" : "ft" ));
	skp->Text (Col(0), Line(l++), buf, strlen(buf));

  if (VC->firstCalc) return true;   // Relative data needs 2 calcs to initialize
	
  l++;
  sprintf_s(buf, 128, "      Att     Rate    Action");
  skp->Text (Col(0), Line(l++), buf, strlen(buf));
  if (abs(VC->dispOriDir.y)<0.5) {
      sprintf_s(buf, 128, "Y%9.2f%9.3f    ", VC->dispOriDir.y, VC->dispOriDirRate.y);
  } else if (VC->dispOriDir.y>0) {
    sprintf_s(buf, 128, "Y%9.2f%9.3f   Yaw Right", VC->dispOriDir.y, VC->dispOriDirRate.y);
  } else {
    sprintf_s(buf, 128, "Y%9.2f%9.3f   Yaw Left", VC->dispOriDir.y, VC->dispOriDirRate.y);
  }
  skp->Text (Col(0), Line(l++), buf, strlen(buf));

  if (abs(VC->dispOriDir.x)<0.5) {
    sprintf_s(buf, 128, "P%9.2f%9.3f    ", VC->dispOriDir.x, VC->dispOriDirRate.x);
  } else if (VC->dispOriDir.x>0) {
    sprintf_s(buf, 128, "P%9.2f%9.3f   Pitch Up", VC->dispOriDir.x, VC->dispOriDirRate.x);
  } else {
    sprintf_s(buf, 128, "P%9.2f%9.3f   Pitch Down", VC->dispOriDir.x, VC->dispOriDirRate.x);
  }
  skp->Text (Col(0), Line(l++), buf, strlen(buf));

  if (VC->mode < 3) {  // Roll guidance not avtive for RVEL
    if (abs(VC->dispOriDir.z)<0.5) {
      sprintf_s(buf, 128, "R%9.2f%9.3f    ", VC->dispOriDir.z, VC->dispOriDirRate.z);
    } else if (VC->dispOriDir.z>0) {
      sprintf_s(buf, 128, "R%9.2f%9.3f   Roll Right", VC->dispOriDir.z, VC->dispOriDirRate.z);
    } else {
      sprintf_s(buf, 128, "R%9.2f%9.3f   Roll Left", VC->dispOriDir.z, VC->dispOriDirRate.z);
    }
    skp->Text (Col(0), Line(l++), buf, strlen(buf));
  }

  if (VC->mode < 3) { // Pos guidance not active for RVEL mode
    l++;
    sprintf_s(buf, 128, "      Pos     Vel     Action");
    skp->Text (Col(0), Line(l++), buf, strlen(buf));

    if ((abs(VC->dispOriPos.x)<1000000.0)&&(abs(VC->dispOriPosRate.x)<1000000.0)) {
      if (abs(VC->dispOriPos.x)<0.04) {
        sprintf_s(buf, 128, "X%9.2f%9.3f    ", ed(VC->dispOriPos.x), ed(VC->dispOriPosRate.x));
      } else if (VC->dispOriPos.x<0) {
        sprintf_s(buf, 128, "X%9.2f%9.3f   Go Left", ed(VC->dispOriPos.x), ed(VC->dispOriPosRate.x));
      } else {
        sprintf_s(buf, 128, "X%9.2f%9.3f   Go Right", ed(VC->dispOriPos.x), ed(VC->dispOriPosRate.x));
      }
      skp->Text (Col(0), Line(l++), buf, strlen(buf));
    }

    if ((abs(VC->dispOriPos.y)<1000000.0)&&(abs(VC->dispOriPosRate.y)<1000000.0)) {
      if (abs(VC->dispOriPos.y)<0.04) {
        sprintf_s(buf, 128, "Y%9.2f%9.3f    ", ed(VC->dispOriPos.y), ed(VC->dispOriPosRate.y));
      } else if (VC->dispOriPos.y<0) {
        sprintf_s(buf, 128, "Y%9.2f%9.3f   Go Down", ed(VC->dispOriPos.y), ed(VC->dispOriPosRate.y));
      } else {
        sprintf_s(buf, 128, "Y%9.2f%9.3f   Go Up", ed(VC->dispOriPos.y), ed(VC->dispOriPosRate.y));
      }
      skp->Text (Col(0), Line(l++), buf, strlen(buf));
    }

    if ((abs(VC->dispOriPos.z)<1000000.0)&&(abs(VC->dispOriPosRate.z)<1000000.0)) {
      if (abs(VC->dispOriPos.z)<0.04) {
        sprintf_s(buf, 128, "Z%9.2f%9.3f    ", ed(VC->dispOriPos.z), ed(VC->dispOriPosRate.z));
      } else if (VC->dispOriPos.z<0) {
        sprintf_s(buf, 128, "Z%9.2f%9.3f   Go Backwards", ed(VC->dispOriPos.z), ed(VC->dispOriPosRate.z));
      } else {
        sprintf_s(buf, 128, "Z%9.2f%9.3f   Go Forwards", ed(VC->dispOriPos.z), ed(VC->dispOriPosRate.z));
      }
      skp->Text (Col(0), Line(l++), buf, strlen(buf));
    }
  }

  if ((VC->mode == 1)||(VC->mode == 2)) {
    // SWP mode ... add in port orientation data
    l++;
    sprintf_s(buf, 128, "Port Ori");
    skp->Text (Col(0), Line(l++), buf, strlen(buf));
    sprintf_s(buf, 128, "          Act         Tgt");
    skp->Text (Col(0), Line(l++), buf, strlen(buf));
    sprintf_s(buf, 128, "Lat  %9.2f   %9.2f", VC->rtvOurLat, VC->rtvWPlat[VC->curWP]); 
    skp->Text (Col(0), Line(l++), buf, strlen(buf));
    sprintf_s(buf, 128, "Long %9.2f   %9.2f", VC->rtvOurLong, VC->rtvWPlong[VC->curWP]);
    skp->Text (Col(0), Line(l++), buf, strlen(buf));
    sprintf_s(buf, 128, "Rng  %9.2f   %9.2f", ed(VC->rtvOurRange), ed(VC->rtvWPrng[VC->curWP]));
    skp->Text (Col(0), Line(l++), buf, strlen(buf));
  }

//	l++;l++;
//	sprintf_s(buf, 128, "Mass %9.2f", VC->v->GetMass());
//  skp->Text (Col(0), Line(l++), buf, strlen(buf));


	return true;
}


bool RV_Orientation::CalibrationUpdate (oapi::Sketchpad *skp) {
  Title (skp, "RV Orientation");

  int l = 3;
  char buf[128]; 
//  char buf2[128];

	skp->SetTextAlign (oapi::Sketchpad::LEFT, oapi::Sketchpad::BOTTOM);
	skp->SetTextColor (0x00FFFF);

  sprintf_s(buf, 128, "  C A L I B R A T I O N   M O D E");
	skp->Text (Col(0), Line(l++), buf, strlen(buf));
  sprintf_s(buf, 128, "  +++++++++++++++++++++++++++++++");
	skp->Text (Col(0), Line(l++), buf, strlen(buf));
  l++;
  if (!VC->apCalComplete) {
    sprintf_s(buf, 128, "    Please don't use thrusters!");
	  skp->Text (Col(0), Line(l++), buf, strlen(buf));
    l++; l++;
    sprintf_s(buf, 128, "Pass %d, %s Test", VC->apCalPass+1, VC->apCalThGName[VC->apCalCtrl[VC->apCalPhase]]);
    skp->Text (Col(0), Line(l++), buf, strlen(buf));
    if (VC->apCalTTG - VC->simT > 0.0) {
      sprintf_s(buf, 128, "Time Left: %.0f sec", VC->apCalTTG - VC->simT);
      skp->Text (Col(0), Line(l), buf, strlen(buf));
    }
    l++;l++;
  }

  sprintf_s(buf, 128,  "+ve");
	skp->Text (Col2(1), Line(l), buf, strlen(buf));
  sprintf_s(buf, 128, " -ve");
	skp->Text (Col2(4), Line(l), buf, strlen(buf));
  sprintf_s(buf, 128, " LoW");
	skp->Text (Col2(7), Line(l), buf, strlen(buf));
  sprintf_s(buf, 128, " HiW");
	skp->Text (Col2(10), Line(l++), buf, strlen(buf));


  sprintf_s(buf, 128, "Y");
	skp->Text (Col2(0), Line(l), buf, strlen(buf));
  if (VC->apCalRateSet[1]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[1]);
	  skp->Text (Col2(1), Line(l), buf, strlen(buf));
  }
  if (VC->apCalRateSet[7]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[7]);
	  skp->Text (Col2(4), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[1][0]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[1][0]);
	  skp->Text (Col2(7), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[1][1]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[1][1]);
	  skp->Text (Col2(10), Line(l), buf, strlen(buf));
  }
  l++;

  sprintf_s(buf, 128, "P");
	skp->Text (Col2(0), Line(l), buf, strlen(buf));
  if (VC->apCalRateSet[0]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[0]);
	  skp->Text (Col2(1), Line(l), buf, strlen(buf));
  }
  if (VC->apCalRateSet[6]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[6]);
	  skp->Text (Col2(4), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[0][0]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[0][0]);
	  skp->Text (Col2(7), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[0][1]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[0][1]);
	  skp->Text (Col2(10), Line(l), buf, strlen(buf));
  }
  l++;

  sprintf_s(buf, 128, "R");
	skp->Text (Col2(0), Line(l), buf, strlen(buf));
  if (VC->apCalRateSet[2]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[2]);
	  skp->Text (Col2(1), Line(l), buf, strlen(buf));
  }
  if (VC->apCalRateSet[8]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[8]);
	  skp->Text (Col2(4), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[2][0]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[2][0]);
	  skp->Text (Col2(7), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[2][1]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[2][1]);
	  skp->Text (Col2(10), Line(l), buf, strlen(buf));
  }
  l++; l++;

  sprintf_s(buf, 128, " +ve");
	skp->Text (Col2(1), Line(l), buf, strlen(buf));
  sprintf_s(buf, 128, " -ve");
	skp->Text (Col2(4), Line(l), buf, strlen(buf));
  sprintf_s(buf, 128, " LoW");
	skp->Text (Col2(7), Line(l), buf, strlen(buf));
  sprintf_s(buf, 128, " HiW");
	skp->Text (Col2(10), Line(l++), buf, strlen(buf));

  sprintf_s(buf, 128, "X");
	skp->Text (Col2(0), Line(l), buf, strlen(buf));
  if (VC->apCalRateSet[3]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[3]);
	  skp->Text (Col2(1), Line(l), buf, strlen(buf));
  }
  if (VC->apCalRateSet[9]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[9]);
	  skp->Text (Col2(4), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[3][0]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[3][0]);
	  skp->Text (Col2(7), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[3][1]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[3][1]);
	  skp->Text (Col2(10), Line(l), buf, strlen(buf));
  }
  l++;

  sprintf_s(buf, 128, "Y");
	skp->Text (Col2(0), Line(l), buf, strlen(buf));
  if (VC->apCalRateSet[4]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[4]);
	  skp->Text (Col2(1), Line(l), buf, strlen(buf));
  }
  if (VC->apCalRateSet[10]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[10]);
	  skp->Text (Col2(4), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[4][0]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[4][0]);
	  skp->Text (Col2(7), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[4][1]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[4][1]);
	  skp->Text (Col2(10), Line(l), buf, strlen(buf));
  }
  l++;

  sprintf_s(buf, 128, "Z");
	skp->Text (Col2(0), Line(l), buf, strlen(buf));
  if (VC->apCalRateSet[5]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[5]);
	  skp->Text (Col2(1), Line(l), buf, strlen(buf));
  }
  if (VC->apCalRateSet[11]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalRate[11]);
	  skp->Text (Col2(4), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[5][0]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[5][0]);
	  skp->Text (Col2(7), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterRateSet[5][1]) {
    sprintf_s(buf, 128, "%6.3f", VC->apCalMasterRate[5][1]);
	  skp->Text (Col2(10), Line(l), buf, strlen(buf));
  }
  l++;l++;
  if (VC->apCalMasterMassSet[0]) {
    sprintf_s(buf, 128, "Mass:");
	  skp->Text (Col2(5), Line(l), buf, strlen(buf));
    sprintf_s(buf, 128, "%6.0f", VC->apCalMasterMass[0]);
	  skp->Text (Col2(7), Line(l), buf, strlen(buf));
  }
  if (VC->apCalMasterMassSet[1]) {
    sprintf_s(buf, 128, "%6.0f", VC->apCalMasterMass[1]);
	  skp->Text (Col2(10), Line(l), buf, strlen(buf));
  }
  l++;

  if (VC->apCalComplete) {
    l++; 
    sprintf_s(buf, 128, "Press OK to continue...");
	  skp->Text (Col(1), Line(l), buf, strlen(buf));
  }

  return true;
}

// MFD Line formatting helper
void RV_Orientation::ShowMessage(oapi::Sketchpad *skp) {

  char localMsg[750];
  strcpy_s(localMsg,750, LC->Message);
  char *bp = localMsg;
  char *bp2 = localMsg;
  char *bp3;
  char c1, c2;
  int i = 0;
  int j;
  int l = 4;
  bool eol = false;

  do {
    if ((*bp2 == '\n') || (*bp2 == '\0')) {     // Look for user newline or end of buffer
      eol = true;
      c1 = *bp2;
      *bp2 = '\0';
    } else {
      if (i==34) {                              // 34 chars no newline ... need to break the line
        eol=true;
        bp3 = bp2;
        for (j=34; j>20; j--) {                 // look for a space from 21 to 34
          if (*bp3==' ') break;
          bp3--;
        }
        if (j>20) {                             // space found
          bp2 = bp3;
          c1 = *bp2;
          *bp2 = '\0';
        } else {                                // no space ... insert hyphen
          bp3 = bp2 + 1;
          c1 = *bp2;
          c2 = *bp3;
          *bp2 = '-';
          *bp3 = '\0';
        }
      } else {                                  // Scan forward      
        i++;
        bp2++;
      }
    }

    if (eol) {                                  // EOL flag ... write out buffer from bp to bp2.
  	  skp->Text (Col(0), Line(l++), bp, strlen(bp));
      eol = false;
      if (c1 == '\0') {
        bp = bp2;     // End of buffer
      } else if ((c1 == '\n') || (c1 == ' ')) {
        bp = bp2+1;   // Reset for next line of the buffer
        bp2++;
        i=0;
      } else {
        bp = bp2;     // Put back the chars we stomped
        *bp2 = c1;
        *bp3 = c2;
        i=0;
      }
    }
  } while (*bp);

  return;
}


// MFD Positioning Helper Functions
int RV_Orientation::Line( int row ) {  // row is 0-24, for 24 rows. e.g. Line(12)
  int ret;
  ret = (int) ((H-(int)(ch/4)) * row / 25) + (int) (ch/4);
  return ret;
};

int RV_Orientation::Col( int pos ) {  // pos is 0-5, for 6 columns. Eg Col(3) for middle
  int ret = (int) ((W-(int)(cw/2)) * pos / 6) + int (cw/2);
  return ret;
};

int RV_Orientation::Col2( int pos ) {  // pos is 0-11, for 12 columns. Eg Col(6) for middle
  int ret = (int) ((W-(int)(cw/2)) * pos / 12) + int (cw/2);
  return ret;
};

// MFD + HUD Conversion Routines

double RV_Orientation::id( double d ) const {
  // Internalizes distances (if in US, converts to meters)
  return (VC->units? d : d / 3.2808399);
}

double RV_Orientation::ed( double d ) const {
  // Externalizes distances (if in US, converts to feet)
  return (VC->units? d : d * 3.2808399);
}