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

#ifndef __RVO_DIALOGFunc
#define __RVO_DIALOGFunc


class RVO_DialogFunc
{
    public:
        static bool clbkTGT(void *id, char *str, void *usrdata);
        static bool clbkDST(void *id, char *str, void *usrdata);
        static bool clbkHPP(void *id, char *str, void *usrdata);
        static bool clbkAPE(void *id, char *str, void *usrdata);
    protected:
    private:
};

#endif // RVO_DIALOGTGT