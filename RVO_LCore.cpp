// ==========================================================================
//
//	Rendezvous Orientation MFD (Local (Vessel+MFD Panel) Core Persistence)
//	======================================================================
//
//	Copyright (C) 2013-2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==========================================================================

#include "RVO_Cores.hpp"

RVO_LCore::RVO_LCore(VESSEL *vin, UINT mfdin, RVO_GCore* gcin) {
  GC = gcin;
  v = vin;
  m = mfdin;

  VC = (RVO_VCore*) GC->P.FindVC(v);
  MC = (RVO_MCore*) GC->P.FindMC(m);

  showMessage = false;
  okMessagePage = true;
  return;
}
