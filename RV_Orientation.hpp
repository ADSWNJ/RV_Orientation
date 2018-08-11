// ==============================================================
//
//	Rendezvous Orientation MFD (RV_Orientation)
//	===========================================
//
//	Copyright (C) 2013-2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==============================================================


#ifndef __RVO_H
#define __RVO_H


#include "RVO_Cores.hpp" 
#include "MMExt2_Basic.hpp"
#include "EnjoLib/IDrawsHUD.hpp"   

class RV_Orientation: public MFD2, public EnjoLib::IDrawsHUD
{
public:
	RV_Orientation (DWORD w, DWORD h, VESSEL *vessel, UINT mfd);
	~RV_Orientation ();
	
  char *ButtonLabel (int bt);
	int ButtonMenu (const MFDBUTTONMENU **menu) const;
  bool ConsumeKeyBuffered (DWORD key);
  bool ConsumeButton (int bt, int event);
  
  bool Update (oapi::Sketchpad *skp);
  bool CalibrationUpdate (oapi::Sketchpad *skp);
  static int MsgProc (UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);

  // Button Press Handlers
  void Button_TGT();
  void Button_PRT();
  void Button_HUD();
  void Button_MOD();
  void Button_DST();
  void Button_UNT();
  void Button_APR();
  void Button_APT();
  void Button_APP();
  void Button_APD();
  void Button_APC();
  void Button_APE();
  void Button_OK();
  void Button_GO();
  void Button_CAN();
  void Button_OVR();
  void Button_HPP();

  // HUD Hookers
  void DrawHUD(int mode, const HUDPAINTSPEC *hps, oapi::Sketchpad * skp);
  bool ShouldDrawHUD() const;
  const char* GetModuleName() const;

  // Persistence functions
  void ReadStatus(FILEHANDLE scn);
  void WriteStatus(FILEHANDLE scn) const;

  // Unit conversions
  double id( double d ) const;
  double ed( double d ) const;

protected:
  RVO_GCore* GC;
  RVO_LCore* LC;
  RVO_MCore* MC;
  RVO_VCore* VC;

  int Line( int row );
  int Col( int pos );
  int Col2( int pos );
  void ShowMessage(oapi::Sketchpad *skp);

  oapi::Font *font, *HUDfont;
  DWORD ColorMap[3];
  oapi::Pen *ArrowPen[3];
  oapi::Brush *ArrowBrush[3];
  oapi::Pen *GuidePen[3];

  //Module Messaging interface
  MMExt2::Basic modMsg;

  // Internal functions
  void DrawArrow(oapi::Sketchpad *skp, bool fill, bool straight, int x, int y, int scale, VECTOR3 drot);


};

#endif // !__RVO_H