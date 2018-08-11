// ==============================================================
//
//	Rendezvous Orientation MFD (MFD Button Management)
//	=============================================
//
//	Copyright (C) 2013-2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==============================================================
#include "MFDButtonPage.hpp"
#include "RVO_Buttons.hpp"
#include "RV_Orientation.hpp"


RVO_Buttons::RVO_Buttons() 
{
    // The menu descriptions of all buttons
    static const MFDBUTTONMENU mnu0[] =
    {
      {"Select Mode", 0, 'M'},
      {"Select Target", 0, 'T'},
      {"Next Dock Port", 0, 'P'},
      {"Select WP Dist", 0, 'D'},
      {"Select Units", 0, 'U'},
      {"HUD On/Off", 0, 'H'},
      {"AP Approach", 0, '1'},
      {"AP Rotate", 0, '2'},
      {"AP Translate", 0, '3'},
      {"AP Dump", 0, '4'},
      {"AP Calibrate", 0, '5'}
//      ,{"HUD Pers Params", 0, '6'}
    };

    RegisterPage(mnu0, sizeof(mnu0) / sizeof(MFDBUTTONMENU));

    RegisterFunction("MOD", OAPI_KEY_M, &RV_Orientation::Button_MOD);
    RegisterFunction("TGT", OAPI_KEY_T, &RV_Orientation::Button_TGT);
    RegisterFunction("PRT", OAPI_KEY_P, &RV_Orientation::Button_PRT);
    RegisterFunction("DST", OAPI_KEY_D, &RV_Orientation::Button_DST);
    RegisterFunction("UNT", OAPI_KEY_U, &RV_Orientation::Button_UNT);
    RegisterFunction("HUD", OAPI_KEY_H, &RV_Orientation::Button_HUD);
    RegisterFunction("APP", OAPI_KEY_1, &RV_Orientation::Button_APP);
    RegisterFunction("APR", OAPI_KEY_2, &RV_Orientation::Button_APR);
    RegisterFunction("APT", OAPI_KEY_3, &RV_Orientation::Button_APT);
    RegisterFunction("APD", OAPI_KEY_4, &RV_Orientation::Button_APD);
    RegisterFunction("APC", OAPI_KEY_5, &RV_Orientation::Button_APC);
//    RegisterFunction("HPP", OAPI_KEY_6, &RV_Orientation::Button_HPP);


    static const MFDBUTTONMENU mnu1[] =
    {
      {"Return", 0, 'K'}
    };

    RegisterPage(mnu1, sizeof(mnu1) / sizeof(MFDBUTTONMENU));

    RegisterFunction("OK", OAPI_KEY_K, &RV_Orientation::Button_OK);

    static const MFDBUTTONMENU mnu2[] = {{}};   // Used in Calibration Mode

    RegisterPage(mnu2, sizeof(mnu2) / sizeof(MFDBUTTONMENU));

    static const MFDBUTTONMENU mnu3[] =
    {
      {"Cancel", 0, 'C'},
      {"Override", 0, 'O'},
    };

    RegisterPage(mnu3, sizeof(mnu3) / sizeof(MFDBUTTONMENU));

    RegisterFunction("CAN", OAPI_KEY_C, &RV_Orientation::Button_CAN);
    RegisterFunction("OVR", OAPI_KEY_O, &RV_Orientation::Button_OVR);

    static const MFDBUTTONMENU mnu4[] =
    {
      {"Go", 0, 'G'},
    };

    RegisterPage(mnu4, sizeof(mnu4) / sizeof(MFDBUTTONMENU));

    RegisterFunction("GO", OAPI_KEY_G, &RV_Orientation::Button_GO);

    return;
}

bool RVO_Buttons::SearchForKeysInOtherPages() const
{
    return false;
}



