// ==============================================================
//
//	Rendezvous Orientation MFD (RV_Orientation)
//	===========================================
//
//	Copyright (C) 2013	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==============================================================


#ifndef _RVO_BUTTON_CLASS
#define _RVO_BUTTON_CLASS
#include "MFDButtonPage.hpp"

class RV_Orientation;

class RVO_Buttons : public MFDButtonPage<RV_Orientation>
{
  public:
    RVO_Buttons();
  protected:
    bool SearchForKeysInOtherPages() const;
  private:
};
#endif // _RVO_BUTTON_CLASS

