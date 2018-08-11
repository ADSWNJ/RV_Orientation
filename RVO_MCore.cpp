// ==============================================================
//
//	Rendezvous Orientation MFD (MFD Panel Core Persistence)
//	=======================================================
//
//	Copyright (C) 2013-2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==============================================================


#include "RVO_Cores.hpp"

RVO_MCore::RVO_MCore(UINT mfdin, RVO_GCore* gcin) {
  GC = gcin;
  m = mfdin;

  return;
}


