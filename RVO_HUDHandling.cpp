// ==============================================================
//
//	Rendezvous Orientation MFD (HUD Handling)
//	=============================================
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==============================================================

#include "RV_Orientation.hpp"

void RV_Orientation::DrawHUD (int mode, const HUDPAINTSPEC *hps, oapi::Sketchpad * skp)
{
  if (VC->showHUD==0) return;

  char buf[256];
  int xPix, yPix;
  int xOfs, yOfs;
  int ix; // Pen + Brush Index
  oapi::Font *oldFont;
  oapi::Pen *oldPen;
  oapi::Brush *oldBrush;
  DWORD oldTextColor;
  bool aOn, bOn; // used for coloring in the a arrow or b arrow 
  bool normalScreen = (hps->W > 500);   // ... else TinyScreen for 3D cockpits, etc, where we need to move things around

  DWORD skpcsHW;
  DWORD skpcsCH;
  DWORD skpcsCW;

  int CX = (normalScreen ? hps->W / 2 : (int) (hps->W * 0.50));    // For some reason, offset camera views set hps->CX and hps->CY to -10000 ? 
  int CY = (normalScreen ? hps->H / 2 : (int) (hps->H * 0.60));
  int yPixInc;

  char valFmt[10], rateFmt[10];

  if (normalScreen) {
    strcpy_s(valFmt,"%9.2f");
    strcpy_s(rateFmt,"%+5.2f");
  } else {
    strcpy_s(valFmt,"%9.1f");
    strcpy_s(rateFmt,"%+5.1f");
  }


  if (!VC->firstCalc) {

    if (normalScreen) {
      HUDfont = oapiCreateFont (20, true, "Fixed", FONT_BOLD, 0);
    } else {
      HUDfont = oapiCreateFont (8, true, "Fixed", FONT_NORMAL, 0);
    }
    oldFont = skp->SetFont (HUDfont);

    skpcsHW = skp->GetCharSize();           // GetCharSize delivers to numbers bit-shifted in the same DWORD.
    skpcsCH = skpcsHW & 0xFFFF;             // Char height is in the lower 16 
    skpcsCW = skpcsHW >> 16;	              // Char width in the upper 16
    yPixInc = (normalScreen ? skpcsCH+2 : skpcsCH);
    xOfs = (normalScreen ? (int) (hps->Markersize*5.5) : (int) (hps->Markersize*4.0));     // x offset used for arrow left right positioning
    yOfs = (int) (hps->Markersize*4.25);    // y offset used for arrow up down positioning

// Draw target, port, mode
    xPix = (normalScreen? (int) (CX-xOfs*1.68) : (int) (CX+xOfs*1.05));
    yPix = (normalScreen? (int) (CY-yOfs*1.75) : (int) (CY+yOfs*1.15));
    oldPen = skp->SetPen(ArrowPen[0]);
    oldBrush = skp->SetBrush(ArrowBrush[0]);
    oldTextColor = skp->SetTextColor(ColorMap[0]);
    sprintf_s(buf, "%s %02d", VC->TargetText, VC->PortNumber+1);
    skp->Text (xPix, yPix, buf, strlen(buf));
    switch (VC->mode) {
      case -1:
        sprintf_s(buf, "%s", "APC");
        break;
      case 0:
        sprintf_s(buf, "%s", "DOCK");
        break;
      case 1:
        sprintf_s(buf, "%s", "WP");
        break;
      case 2:
        sprintf_s(buf, "%s.%.0f.%.0f", "SWP", VC->rtvWPlat[VC->curWP], VC->rtvWPlong[VC->curWP]);
        break;
      case 3:
        sprintf_s(buf, "%s", "RVEL");
        break;
    }
    yPix += yPixInc;
    skp->Text (xPix, yPix, buf, strlen(buf));
    sprintf_s(buf,"%s%s%s", (VC->apAppArmed? "APP " : ""), (VC->apRotArmed? "APR " : ""), (VC->apAttArmed? "APT " : ""));
    yPix += yPixInc;
    skp->Text (xPix, yPix, buf, strlen(buf));

// YAW RIGHT LEFT
    if (abs(VC->dispOriDir.y)<0.5) {
      aOn = false;
      bOn = false;
    } else if (VC->dispOriDir.y>0) {
      aOn = true;
      bOn = false;
    } else {
      aOn = false;
      bOn = true;
    }
    if (normalScreen) {
      xPix = (VC->dispOriDir.y<0 ? (int) (CX-xOfs*1.75) : (int) (CX-xOfs*0.73)); 
      yPix = (int) (CY-yOfs*1.22);
    } else {
      xPix = (VC->dispOriDir.y<0 ? (int) (CX-xOfs*2.25) : (int) (CX-xOfs*0.80)); 
      yPix = (int) (CY-yOfs*1.20);
    }
    if (VC->mode) {
      ix = (abs(VC->dispOriDir.y)<10.0 ? 0 : abs(VC->dispOriDir.y)<20.0 ? 1 : 2);
    } else {
      ix = (abs(VC->dispOriDir.y)<(VC->maxDirErr/2.0) ? 0 : abs(VC->dispOriDir.y)<(VC->maxDirErr) ? 1 : 2);
    }
    skp->SetPen(ArrowPen[ix]);
    skp->SetBrush(ArrowBrush[ix]);
    skp->SetTextColor(ColorMap[ix]);
    sprintf_s(buf, valFmt, VC->dispOriDir.y);
    skp->Text (xPix, yPix, buf, strlen(buf));
    xPix += skpcsCW*4;
    yPix += yPixInc;
    sprintf_s(buf, rateFmt, VC->dispOriDirRate.y);
    skp->Text (xPix, yPix, buf, strlen(buf));
    DrawArrow(skp,aOn, false, CX-xOfs,CY-yOfs,hps->Markersize,_V(80, 180,0)); 
    DrawArrow(skp,bOn, false, (int) (CX-xOfs*0.98),CY-yOfs,hps->Markersize,_V(80, 0, 0)); 


// PITCH UP DOWN
    if (abs(VC->dispOriDir.x)<0.2) {
      aOn = false;
      bOn = false;
    } else if (VC->dispOriDir.x>0) {
      aOn = true;
      bOn = false;
    } else {
      aOn = false;
      bOn = true;
    }
    if (normalScreen) {
      xPix = (int) (CX-xOfs*1.20); 
      yPix = (VC->dispOriDir.x<0 ? (int) (CY-yOfs*0.63) : (int) (CY-yOfs*1.61));
    } else {
      xPix = (int) (CX-xOfs*1.50); 
      yPix = (VC->dispOriDir.x<0 ? (int) (CY-yOfs*0.63) : (int) (CY-yOfs*1.75));
    }
    if (VC->mode) {
      ix = (abs(VC->dispOriDir.x)<10.0 ? 0 : abs(VC->dispOriDir.x)<20.0 ? 1 : 2);
    } else {
      ix = (abs(VC->dispOriDir.x)<(VC->maxDirErr/2.0) ? 0 : abs(VC->dispOriDir.x)<(VC->maxDirErr) ? 1 : 2);
    }
    skp->SetPen(ArrowPen[ix]);
    skp->SetBrush(ArrowBrush[ix]);
    skp->SetTextColor(ColorMap[ix]);
    sprintf_s(buf, valFmt, VC->dispOriDir.x);
    skp->Text (xPix, yPix, buf, strlen(buf));
    xPix += skpcsCW*4;
    yPix += yPixInc;
    sprintf_s(buf, rateFmt, VC->dispOriDirRate.x);
    skp->Text (xPix, yPix, buf, strlen(buf));
    DrawArrow(skp,aOn, false, CX-xOfs,(int) (CY-yOfs*1.00),hps->Markersize,_V(0,-75,90)); 
    DrawArrow(skp,bOn, false, CX-xOfs,(int) (CY-yOfs*1.05),hps->Markersize,_V(0, 105,-90)); 


    if (VC->mode < 3) {
// ROLL RIGHT LEFT - not active for RVEL     xPix = (int) (CX-xOfs*0.93);
      if (abs(VC->dispOriDir.z)<0.2) {
        aOn = false;
        bOn = false;
      } else if (VC->dispOriDir.z>0) {
        aOn = true;
        bOn = false;
      } else {
        aOn = false;
        bOn = true;
      }
      if (normalScreen) {
        xPix = (VC->dispOriDir.z<0 ? (int) (CX-xOfs*1.53) : (int) (CX-xOfs*0.92));
        yPix = (int) (CY-yOfs*1.43);
      } else {
        xPix = (VC->dispOriDir.z<0 ? (int) (CX-xOfs*1.95) : (int) (CX-xOfs*1.06));
        yPix = (int) (CY-yOfs*1.48);
      }
      if (VC->mode) {
        ix = (abs(VC->dispOriDir.z)<10.0 ? 0 : abs(VC->dispOriDir.z)<20.0 ? 1 : 2);
      } else {
        ix = (abs(VC->dispOriDir.z)<(VC->maxDirErr/2.0) ? 0 : abs(VC->dispOriDir.z)<(VC->maxDirErr) ? 1 : 2);
      }
      skp->SetPen(ArrowPen[ix]);
      skp->SetBrush(ArrowBrush[ix]);
      skp->SetTextColor(ColorMap[ix]);
      sprintf_s(buf, valFmt, VC->dispOriDir.z);
      skp->Text (xPix, yPix, buf, strlen(buf));
      xPix += skpcsCW*4;
      yPix += yPixInc;
      sprintf_s(buf, rateFmt, VC->dispOriDirRate.z);
      skp->Text (xPix, yPix, buf, strlen(buf));
      DrawArrow(skp,aOn, false, (int) (CX-xOfs*0.93),(int) (CY-yOfs*1.35),(int) (hps->Markersize/2),_V(0, 180,-90)); 
      DrawArrow(skp,bOn, false, (int) (CX-xOfs*1.01),(int) (CY-yOfs*1.35),(int)(hps->Markersize/2),_V(0, 0,-90)); 
    }


    if (VC->mode < 3) { // no positioning for RVEL

// TRANSLATE RIGHT LEFT

      if (abs(VC->dispOriPos.x)<0.05) {
        aOn = false;
        bOn = false;
      } else if (VC->dispOriPos.x<0) {
        aOn = true;
        bOn = false;
      } else {
        aOn = false;
        bOn = true;
      }
      if (normalScreen) {
        xPix = (VC->dispOriPos.x<0 ? (int) (CX+xOfs*0.29) : (int) (CX+xOfs*1.25));
        yPix = (int) (CY-yOfs*1.22);
      } else {
        xPix = (VC->dispOriPos.x<0 ? (int) (CX+xOfs*0.05) : (int) (CX+xOfs*1.05));
        yPix = (VC->dispOriPos.x<0 ? (int) (CY-yOfs*0.92) : (int) (CY-yOfs*1.26));
      }
      if (VC->mode) {
        ix = (abs(VC->dispOriPos.x)<100 ? 0 : abs(VC->dispOriPos.x)<200 ? 1 : 2);
      } else {
        ix = (abs(VC->dispOriPos.x)<(VC->maxPosErr/2.0) ? 0 : abs(VC->dispOriPos.x)<(VC->maxPosErr) ? 1 : 2);
      }
      skp->SetPen(ArrowPen[ix]);
      skp->SetBrush(ArrowBrush[ix]);
      skp->SetTextColor(ColorMap[ix]);
      if (abs(VC->dispOriPos.x) < 1000000.0) {
        sprintf_s(buf, valFmt, ed(VC->dispOriPos.x));
        skp->Text (xPix, yPix, buf, strlen(buf));
        xPix += skpcsCW*4;
        yPix += yPixInc;
      }
      if (abs(VC->dispOriPosRate.x) < 1000000.0) {
        sprintf_s(buf, rateFmt, ed(VC->dispOriPosRate.x));
        skp->Text (xPix, yPix, buf, strlen(buf));
      }
      DrawArrow(skp,aOn, true, CX+xOfs,CY-yOfs,hps->Markersize,_V(-80,0,-90)); 
      DrawArrow(skp,bOn, true, CX+xOfs,CY-yOfs,hps->Markersize,_V(-80,0,90)); 


  // TRANSLATE DOWN UP
      if (abs(VC->dispOriPos.y)<0.05) {
        aOn = false;
        bOn = false;
      } else if (VC->dispOriPos.y<0) {
        aOn = true;
        bOn = false;
      } else {
        aOn = false;
        bOn = true;
      }
      if (normalScreen) {
        xPix = (int) (CX+xOfs*0.75); //825
        yPix = (VC->dispOriPos.y<0 ? (int) (CY-yOfs*0.43) : (int) (CY-yOfs*1.75));
      } else {
        xPix = (int) (CX+xOfs*0.44); //825
        yPix = (VC->dispOriPos.y<0 ? (int) (CY-yOfs*0.38) : (int) (CY-yOfs*1.85));
      }
      if (VC->mode) {
        ix = (abs(VC->dispOriPos.y)<100.0 ? 0 : abs(VC->dispOriPos.y)<200.0 ? 1 : 2);
      } else {
        ix = (abs(VC->dispOriPos.y)<(VC->maxPosErr/2.0) ? 0 : abs(VC->dispOriPos.y)<(VC->maxPosErr) ? 1 : 2);
      }
      skp->SetPen(ArrowPen[ix]);
      skp->SetBrush(ArrowBrush[ix]);
      skp->SetTextColor(ColorMap[ix]);
      if (abs(VC->dispOriPos.y) < 1000000.0) {
        sprintf_s(buf, valFmt, ed(VC->dispOriPos.y));
        skp->Text (xPix, yPix, buf, strlen(buf));
        xPix += skpcsCW*4;
        yPix += yPixInc;
      }
      if (abs(VC->dispOriPosRate.y) < 1000000.0) {
        sprintf_s(buf, rateFmt, ed(VC->dispOriPosRate.y));
        skp->Text (xPix, yPix, buf, strlen(buf));
      }
      DrawArrow(skp,aOn, true, CX+xOfs,CY-yOfs,hps->Markersize,_V(0,82,180)); 
      DrawArrow(skp,bOn, true, CX+xOfs,CY-yOfs,hps->Markersize,_V(0,82,0)); 


  // TRANSLATE BACK FORWARD
      if (abs(VC->dispOriPos.z)<0.05) {
        aOn = false;
        bOn = false;
      } else if (VC->dispOriPos.z<0) {
        aOn = true;
        bOn = false;
      } else {
        aOn = false;
        bOn = true;
      }
      if (VC->mode==0) {
        if (abs(VC->dispOriPos.z<0.05)) {
          ix = (abs(VC->dispOriPosRate.z)<0.10 ? 0 : abs(VC->dispOriPosRate.z)<0.20 ? 1 : 2);    
        } else if (abs(VC->dispOriPos.z<2)) {
          ix = (-VC->dispOriPosRate.z<0.25? 0 :-VC->dispOriPosRate.z/VC->dispOriPos.z<0.50 ? 0 : -VC->dispOriPosRate.z/VC->dispOriPos.z<1.00 ? 1 : 2);
        } else if (abs(VC->dispOriPos.z<25)) {
          ix = (-VC->dispOriPosRate.z<0.50 ? 0 : -VC->dispOriPosRate.z/VC->dispOriPos.z<1.00 ? 1 : 2);
        } else {
          ix = (-VC->dispOriPosRate.z/VC->dispOriPos.z<0.022 ? 0 : -VC->dispOriPosRate.z/VC->dispOriPos.z<0.030 ? 1 : 2);
        }
      } else {
        ix = (abs(VC->dispOriPosRate.z)<10.0 ? 0 : abs(VC->dispOriPosRate.z)<20.0 ? 1 : 2); 
      }
      if (normalScreen) {
        xPix = (VC->dispOriPos.z<0 ? (int) (CX+xOfs*0.99) : (int) (CX+xOfs*0.60)); // 98 70
        yPix = (VC->dispOriPos.z<0 ? (int) (CY-yOfs*0.86) : (int) (CY-yOfs*1.36)); // 80 143
      } else {
        xPix = (VC->dispOriPos.z<0 ? (int) (CX+xOfs*0.88) : (int) (CX+xOfs*0.20)); // 98 70
        yPix = (VC->dispOriPos.z<0 ? (int) (CY-yOfs*0.92) : (int) (CY-yOfs*1.30)); // 80 143
      }
      skp->SetPen(ArrowPen[ix]);
      skp->SetBrush(ArrowBrush[ix]);
      skp->SetTextColor(ColorMap[ix]);
      if (abs(VC->dispOriPos.z) < 1000000.0) {
        sprintf_s(buf, valFmt, ed(VC->dispOriPos.z));
        skp->Text (xPix, yPix, buf, strlen(buf));
        xPix += skpcsCW*4;
        yPix += yPixInc;
      }
      if (abs(VC->dispOriPosRate.z) < 1000000.0) {
        sprintf_s(buf, rateFmt, ed(VC->dispOriPosRate.z));
        skp->Text (xPix, yPix, buf, strlen(buf));
      }
      DrawArrow(skp,aOn, true, CX+xOfs,CY-yOfs,hps->Markersize,_V(0,82,90)); 
      DrawArrow(skp,bOn,  true, CX+xOfs,CY-yOfs,hps->Markersize,_V(0,82,-90)); 

      if (VC->showHUD==2) {

        //
        // Draw HUD Tracking corridor
        //
        VECTOR3 cgd, cgp,cld, clp;
        oapiCameraGlobalDir(&cgd);              // Get COP direction ... (WHY NO oapiCameraGlobalRot function?)
        oapiCameraGlobalPos(&cgp);              // Get COP position
        cgd = cgd + cgp;                        // Flip direction into a point offset from COP
        VC->v->Global2Local(cgd, cld);          // Switch to ship coords
        VC->v->Global2Local(cgp, clp);          // .. for direction and position
        cld = cld - clp;                        // Pull the position out of the direction point offset to make it a direction again

  //      sprintf_s(oapiDebugString(),256,"HUD Data: W %i H %i CX %i CY %i FOV %.2f CMode %i Camera Pos {%.2f, %.2f, %.2f} Camera Dir {%.2f, %.2f, %.2f}  HUD PARAMS {Z %0.2f}", hps->W, hps->H, CX, CY, oapiCameraAperture()*DEG*2, oapiCameraMode(), clp.x,clp.y,clp.z, cld.x,cld.y,cld.z, VC->hpp_z);

        VECTOR3 HUD_dim;                        // HUD dimensions (x wide, y high, at z distance from our center of projection)
        oapi::IVECTOR2 HUD_vertex[20];          // HUD 2D shape vertices for HUD projection
        double halffov = oapiCameraAperture();  // oapiCamerAperture gives the field of view angle from the center of the HUD to the top of the HUD - or half fov really
        double xprime, yprime;                  // Projections of the x,y coordinates w.r.t z coordinate

        HUD_dim.z = VC->hpp_z;                  // The z-dimension is assumed 1.0m, but can be adjusted by HPP
        HUD_dim.y = HUD_dim.z * tan(halffov);   // Height of display port is the tan of the angle times the z-distance.  
        HUD_dim.x =  HUD_dim.y;                 // We want the geometry to be symmetric ... i.e. 1m up is 1m left

        VC->GetFirstGuidanceRect();             // Initialize the guidance box track and get first box
        while (VC->guideRectMore) {
          int j = 0;
          for (int i=0; i<VC->guideRectCount; i++) {
            xprime = (VC->lGuideRectVertex[i].x / VC->lGuideRectVertex[i].z) * HUD_dim.z;    // Projection calc: x' = x/z * HUD_z
            yprime = (VC->lGuideRectVertex[i].y / VC->lGuideRectVertex[i].z) * HUD_dim.z;    // Projection calc: y' = y/z * HUD_z
            HUD_vertex[i].x = int((float) hps->H * (xprime / HUD_dim.x) / 2.0)+CX;           // HUD vertex calc: xp = H/2 * (x'/HUD_x) + center_X
            HUD_vertex[i].y = int((float) hps->H * (-yprime / HUD_dim.y) / 2.0)+CY;          // HUD vertex calc: yp = H/2 * (-y'/HUD_y) + center_Y   (negative because yp-origin is top left,and positive yp is down!)
          }
          skp->SetPen(GuidePen[VC->guideRectColor]);                                         // Drawing guidance boxes now
          skp->Polyline(HUD_vertex,VC->guideRectCount);                                      // Draw my box
          VC->GetNextGuidanceRect();                                                         // Go back to the vessel core for the next box
        }
      }
    }; // if VC->mode<3

    skp->SetPen(oldPen);                       // Tidy up the pens and brushes and finish
    skp->SetBrush(oldBrush);
    skp->SetTextColor(oldTextColor);
    skp->SetFont (oldFont);
    oapiReleaseFont(HUDfont);

  }; // if !FirstCalc
	return ;
}




bool RV_Orientation::ShouldDrawHUD () const
{
  return (VC->showHUD>0);
}

const char* RV_Orientation::GetModuleName() const {
  return "RV Orientation";
}

void RV_Orientation::DrawArrow(oapi::Sketchpad *skp, bool fill, bool straight, int x, int y, int scale, VECTOR3 drot) {
  const int si = scale / 4;
  const double s = (double) si;
  int ptn = 12;
  VECTOR3 rot = {drot.x*PI/180,drot.y*PI/180,drot.z*PI/180};
  const VECTOR3 e = {0,0,500};
  VECTOR3 pt[24];
  const VECTOR3 straightArrow[8] = {
      {   s, -2*s, 0},
      {   s, -6*s, 0},
      { 4*s, -6*s, 0},
      {   0,-10*s, 0},
      {-4*s, -6*s, 0},
      {-1*s, -6*s, 0},
      {  -s, -2*s, 0},
      {   s, -2*s, 0}
  };
  const VECTOR3 curveArrow[24] = {
      {-1.000*s, -2.000*s, 0},
      {-1.076*s, -2.868*s, 0},
      {-1.302*s, -3.710*s, 0},
      {-1.670*s, -4.500*s, 0},
      {-2.170*s, -5.214*s, 0},
      {-2.786*s, -5.830*s, 0},
      {-3.500*s, -6.330*s, 0},
      {-4.290*s, -6.698*s, 0},
      {-5.132*s, -6.924*s, 0},
      {-6.000*s, -7.000*s, 0},
      {-6.000*s, -10.000*s, 0},
      {-8.000*s, -6.000*s, 0},
      {-6.000*s, -2.000*s, 0},
      {-6.000*s, -5.000*s, 0},
      {-5.479*s, -4.954*s, 0},
      {-4.974*s, -4.819*s, 0},
      {-4.500*s, -4.598*s, 0},
      {-4.072*s, -4.298*s, 0},
      {-3.702*s, -3.928*s, 0},
      {-3.402*s, -3.500*s, 0},
      {-3.181*s, -3.026*s, 0},
      {-3.046*s, -2.521*s, 0},
      {-3.000*s, -2.000*s, 0},
      {-1.000*s, -2.000*s, 0}
  };

  if (straight) {
    ptn=8;
    for (int i=0;i<ptn;i++) pt[i] = straightArrow[i];
  } else {
    ptn=24;
    for (int i=0;i<ptn;i++) pt[i] = curveArrow[i];
  }

  oapi::IVECTOR2 rpt[24];

  const MATRIX3 Xrot = {1,0,0,0,cos(rot.x),-sin(rot.x),0,sin(rot.x),cos(rot.x)};
  const MATRIX3 Yrot = {cos(rot.y),0,sin(rot.y),0,1,0,-sin(rot.y),0,cos(rot.y)};
  const MATRIX3 Zrot = {cos(rot.z),-sin(rot.z),0,sin(rot.z),cos(rot.z),0,0,0,1};
  const MATRIX3 Trot = mul(Xrot,mul(Yrot,Zrot));

  for (int i=0; i<ptn; i++) {
//    pt[i] = mul(Zrot,pt[i]);
//    pt[i] = mul(Yrot,pt[i]);
//    pt[i] = mul(Xrot,pt[i]);
    pt[i] = mul(Trot,pt[i]);
    rpt[i].x = (long) ((pt[i].x - e.x) * ((e.z - pt[i].z)/e.z) + x);
    rpt[i].y = (long) ((pt[i].y - e.y) * ((e.z - pt[i].z)/e.z) + y);
  }
 
  if (fill) {
    skp->Polygon(rpt,ptn); 
  } else {
    skp->Polyline(rpt,ptn); 
  }
  return;
}