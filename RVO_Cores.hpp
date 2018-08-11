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

#include "windows.h"
#include "orbitersdk.h"
#include "RVO_Buttons.hpp"
#include "MFDPersist.hpp"
#include <list>
//#include <EnjoLib\ModuleMessaging.hpp>
using namespace std;

#ifndef _RVO_CORE_CLASSES
#define _RVO_CORE_CLASSES

// Forward reference needed, as the cores all talk to each other. (This is also why the cores are kept together in this single hpp file)
class RVO_GCore;

//+++++
// Vessel Persistence core. One of these is instantiated per Vessel flown with RV Orientation up.
//+++++

class RVO_VCore {
  public:
    // Core references ... instantiation, vessel reference and GC.
    RVO_VCore(VESSEL *vin, RVO_GCore* gcin);
    RVO_GCore* GC;

    // Add Vessel data here

    // Mode control ... it's vessel rather than MFD local as it controls the vessel calcs
    int mode;                                                       // -1 = CALI, 0 = DOCK, 1 = WP, 2 = SWP, 3 = RVEL
    int apcMode;                                                    // Holds previous mode for APC tuning
    bool units;                                                     // true for METRIC, false for US

    // AP control
    int apToggle;                                                   // Toggles control to rot every 10 goes

    bool apRotArmed;                                                // Arms the rotation control autopilot
    bool apAttArmed;                                                // Arms the attitude control autopilot
    bool apAppArmed;                                                // Arms the approach control autopilot
    bool apAppConeOK;                                               // Suppresses approach if outside cone
    bool apAttAllow;                                                // Interlocks out the attitude if the Rot AP is on and > 5 deg or 0.5 rate deg
    double maxDirErr, maxPosErr;                                    // Cone control - max error allowed in Dir or Pos
    double apAppRMax;                                               // Max allowable rate on approach (keeps it tiny on final)
    double apAppPAtten;                                             // Attenuates the approach rate
    double apAppRVel;                                               // Final approach relative velocity
    

    // Autopilot reference thrust calculations
    VECTOR3 apRotThrust, apAttThrust;                               // Autopilot rotation and attitude thrust settings
    VECTOR3 apRotThrustE, apAttThrustE;                             // Desired thrust, estimate, rate
    VECTOR3 apRotRateLim, apAttRateLim;                             // Resulting thrust, estimate, rate

    // Autopilot Reference Thrust Data - loaded by AP Calibration or from Scenario, or from Master Lookup
    bool apRefRatesLoaded;                                          // Do we have a good set of claibrations onboard?
    double apRefMass;                                               // Reference mass (recalculated when 0.1% away from current mass)
    VECTOR3 apRefRot;                                               // Reference rotation rates (deg/s) for apRefMass
    VECTOR3 apRefAtt;                                               // Reference attitude rates (m/s) for apRefMass
    double apRefMaster[7][2];                                       // Reference Master Calibration Power Coefficients
                                                                    // Y = AX^B ... so LN(Y) = LN(A) + B.LN(X) ... straight line!
    bool apWPreached;                                               // Flags when on station

    // Autopilot Calibration Execute Vars
    int apCalPass;                                                  // Calinration Pass (two passes - lo weight and hi)
    int apCalPhase;                                                 // Calibration Phases
    int apCalStep;                                                  // Calibration Steps
    int apCalCtrl[12];                                              // Calibration control
    double apCalMasterRate[6][2];                                   // Master rates, 1 per direction, low and high weights
    bool apCalMasterRateSet[6][2];                                  // Can I display the master rate?
    double apCalMasterMass[2];                                      // Master mass - average of mass through each burn
    bool apCalMasterMassSet[2];                                     // Can I display master mass?
    int apCalMassFlag;                                              // 0 for low, 1 for high
    bool apCalInit;                                                 // Flags the end of Calibration
    bool apCalComplete;                                             // Flags the end of Calibration
    ELEMENTS apCalEl;                                               // Stores elements for restore after we complete calibration
    double apCalMjdRef;                                             // MJD_REF for element save
    char apCalThGName[13][12];                                      // Names of the thruster directions
    double apCalTTG;                                                // APCal Time To Go
    double apCalTime;                                               // Holds start time for each cal
    double apCalStepTime[4];                                        // Holds the cal step time for steps 1-4
    VECTOR3 apCalStepAVel[4];                                       // Holds the cal angular velocities for steps 1-4
    VECTOR3 apCalStepLVel[4];                                       // Holds the cal linear velocities for steps 1-4

    double apCalStepTimePhase[4][12][4];                            // Stores each phase calibration, for debug
    VECTOR3 apCalStepAVelPhase[4][12][4];                           // Stores each phase cal angular velocities for steps 1-4
    VECTOR3 apCalStepLVelPhase[4][12][4];                           // Stores each phase cal linear velocities for steps 1-4
    double apCalRatePhase[4][12][12];                               // Holds the rate per second post cal run 

    double apCalMass;                                               // Holds ship mass for AP calibration
    double apCalMass2;                                              // Holds ship mass for AP calibration
    double apCalRate[12];                                           // Holds the rate per second post cal run
    bool apCalRateSet[12];                                          // Shows when the calibration rate is set

    int apCalPropCount;                                             // Count of Propellant Sources
    PROPELLANT_HANDLE apCalPH[10];                                  // Handles to up to 10 prop sources
    PROPELLANT_HANDLE apCalThPH;                                    // Handles to up to 10 prop sources
    double apCalPropMass[10];                                       // Holds the original propellant masses pre-cali



    // Autopilot calibration variables
    bool apApcArmed;                                                // Arms the autopilot calibration mode
    VECTOR3 apApcPOfs;                                              // Holds a target relative pos offset to the port for calibration tests
    VECTOR3 apApcDOfs;                                              // Holds a target relative dir offset to the port for calibration tests
    VECTOR3 apApcROfs;                                              // Holds a target relative rot offset to the port for calibration tests
    VECTOR3 apApcRtvrdPos;                                          // Holds remote target vehicle remote dock (i.e. ours) pos in target coords
    VECTOR3 apApcGtvPos;                                            // Holds remote target vehicle pos in global coords
    VECTOR3 apApcLtvPos;                                            // Holds remote target vehicle pos in our ship coords
    MATRIX3 apApcrPitch, apApcrYaw, apApcrRoll;                     // Pitch, yaw, roll rotation matrices for reference rot tests

    // Autopilot file dump debug
    bool dumpAP;                                                    // dump autopilot trigger - opens file, dumps the autopilot responses until toggled off
    bool apRotActive;                                               // Was AP Rot active this turn?
    bool apAttActive;                                               // Was AP Att active this turn?
    bool apAppActive;                                               // Was AP App active this turn?
    FILE* apD;                                                      // Autopilot dump file
    errno_t apD_err;                                                // file errors on autopilot dump
    double dumpSimT;                                                // Start of dump

    // Autopilot rate controls
    double apRotXCtrl[12][2];                                       // Pitch rates
    double apRotYCtrl[12][2];                                       // Yaw rates
    double apRotZCtrl[12][2];                                       // Roll rates
    double apAttXCtrl[12][2];                                       // Right rates
    double apAttYCtrl[12][2];                                       // Up rates
    double apAttZCtrl[12][2];                                       // Fwd rates
                                                                    // Mode 2 rates are more forgiving, to keep the thrust down
    double apRotXCtrlMode2[12][2];                                  //   Pitch rates
    double apRotYCtrlMode2[12][2];                                  //   Yaw rates
    double apRotZCtrlMode2[12][2];                                  //   Roll rates
    double apAttXCtrlMode2[12][2];                                  //   Right rates
    double apAttYCtrlMode2[12][2];                                  //   Up rates
    double apAttZCtrlMode2[12][2];                                  //   Fwd rates

    // Autopilot Error Aggregators
    VECTOR3 apRotErr, apAttErr;                                     // Aggregates errors where actual rate < 75% demanded rate (boots thrust)

    // Core target vessel data
    char TargetText[50];                                            // e.g. ISS
    int  MaxPorts;                                                  // Port count on the target
    int  PortNumber;                                                // Selected port number on target. Note internally and in the API, ports start at 0, and we add 1 so the display ports start at 1. 
    double ApproachWaypointDistance;                                // Distance to the dock - e.g. 500
    double ApproachWaypointOfsDistance;                             // Offset to approach waypoint distance (allowing for turn in on final)

    OBJHANDLE hTgtV;                                                // Handle to target vehicle
    VESSEL *tv;                                                     // Target VESSEL structure
    DOCKHANDLE hDock;                                               // Handle to target dock
    VECTOR3 gtvVel, gtvRVel, ltvRVel;                               // Target vehichle global velocity, global relative to us, and local in our coordinate frame relative to us
    VECTOR3 rtvdPos, rtvdDir, rtvdRot;                              // Target vehicle dock port position, direction & rotation in remote ship coords
    VECTOR3 rTgtPos, rTgtDir, rTgtRot;                              // RV Target in remote ship coords (either the dock or the WP)
    VECTOR3 gTgtPos, gTgtDir, gTgtRot;                              // RV Target in global coords
    VECTOR3 lTgtPos, lTgtDir, lTgtRot;                              // RV Target in our local ship or dock coords
    VECTOR3 gtvdPos, ltvdPos;                                       // Global and local target vehicle doc position


    // Our vessel data
    VESSEL *v;                                                      // Our vessel stucture
    VECTOR3 dPos, dDir, dRot;                                       // Our dock position, alignment direction and alignment roatation in local coords
    VECTOR3 gPos;                                                   // Our dock in global coords
    VECTOR3 gVel;                                                   // Global velocity
    DOCKHANDLE hMyDock;                                             // handle to our own dock

    // Calculation control
    bool firstCalc;                                                 // On the first calc, we don't have the rates
    double simT, oldSimT, simStep;                                  // Simulation time, last time and the step, for rate calcs
    double simStepTimes[10];                                        // Holds the last 10 simstep times
    double lastSimStep;                                             // Holds the last simstep time delta
    double simStepAvg;                                              // Holds the average simstep time
    int simStepTimesPtr;                                            // Points to the next simstep

    VECTOR3 oriPos, oriPosRate;                                     // Orientation position and rate (X Y Z)
    VECTOR3 oriDir, oriDirRate;                                     // Orientation direction and rate (P Y R)
    double oriRVel;                                                 // Our RVel to target port
    VECTOR3 dispOriPos, dispOriPosRate;                             // Display versions of position & rate (with a deadband around zero to stop flickering)
    VECTOR3 dispOriDir, dispOriDirRate;                             // Display versions of orientation & rate (with deadband)
    VECTOR3 oldDir, oldPos;                                         // Previous orientations and positions from last sim step

    // Docking port orientation rotations
    double aPitch, aYaw, aRoll;                                     // Pitch, yaw, roll angles to rotate our coordinate system to align with our dock
    MATRIX3 rPitch, rYaw, rRoll;                                    // Pitch, yaw, roll rotation matrices to translate our coordintes into port coordinates
    MATRIX3 rPortOri;                                               // Resultant port orientation matrix. Multiply this time a local ship coordinate to get a local dock coordinate. 

    // Remote Target orientation rotations for spherical WP calcs
    MATRIX3 rtvPortOri;                                             // Rotation matrix for remote ship's target port
    VECTOR3 rtvOurPos;                                              // Our position (our port's actually) in remote target ship port coords
    double rtvOurLong, rtvOurLat;                                   // Our virtual lat and long in remote target port coords
    double rtvOurRange;                                             // Our range to target port
    double rtvOurWPrange;                                           // Our range to next WP
    MATRIX3 rtvDirRotLong, rtvDirRotLat;                            // Rotation matrices for long lat

    // Set up course to dock
    bool apSWPselect;                                               // Triggers WP selection
    int curWP, prWP;                                                // Current WP (0 = tgt port, 1 = primary WP, rest are the SWP course), and previous one
    double rtvWPlatStart, rtvWPlongStart;                           // Defines start of SWP track
    double rtvWPlong[20], rtvWPlat[20];                             // SWP lat/long WP track
    double rtvWPdist[20];                                           // Dist to waypoint
    double rtvWPrng[20];                                            // Range to port (either ApproachWaypointDist or ApproachWaypointDist + the offset)
    MATRIX3 rtvWProtLong, rtvWProtLat;                              // Rotation matrices for long lat for this WP

    MATRIX3 rtvWPoriL[20];                                          // Coord transform from port coord to WP coord
    VECTOR3 rtvWPpos[20];                                           // Waypoint pos in rtv ship coords
    VECTOR3 rtvWPdir[20];                                           // Direction into the WP in rtv ship vector
    VECTOR3 rtvWProt[20];                                           // Rotation direction up from the WP in rtv ship vector



    // Core HUD data
    int showHUD;                                                     // HUD display on/off ... 0 = off, 1 = arrows only, 2 = arrows and nav boxes

    VECTOR3 rGuideRectVertex[20];                                     // Holds the guidance rectangle vertex tracepath (remote target coords)
    VECTOR3 gGuideRectVertex[20];                                     // Holds the guidance rectangle vertex tracepath (global coords)
    VECTOR3 lGuideRectVertex[20];                                     // Holds the guidance rectangle vertex tracepath (local ship coords)
    VECTOR3 lGuideRectWP[10];                                         // Holds the WP coordinate in target port-local frame for each WP
    double  lGuideRectWPlat[10];                                      // Holds the SWP lat for each WP
    double  lGuideRectWPlong[10];                                     // Holds the SWP long for each WP
    int guideRectColor;                                               // Color for the box
    int guideRectCount;                                               // Holds the count of the rectangle vertices
    int guideWPdist;                                                  // Holds the horizontal distance between two WP's
    int guideRectWP;                                                  // Tracks which WP we are drawing away from
    int guideRectLastWP;                                              // Tracks the last WP in the sequence
    int guideRectWPbox;                                               // Tracks the last box drawn on a path
    bool guideRectMore;                                               // Look for the next one, elase set false at end of path
    bool guideRectDump;                                               // Dumps guidance rectangles
    FILE* gD;                                                         // Guidance dump file
    errno_t gD_err;                                                   // file errors on guidance dump

    // Camera positions
    VECTOR3 cgd,cld,cgp,clp,cgr,clr;                                  // Camera (COP, or Center of Perspective) global and location direction, global and local position, global and local rot
    MATRIX3 cRot;                                                     // Camera roation matrix (ship->COP)
    double hpp_z;                                                     // HUD Perspective Parameter, setting HUD z distance

    // Functions
    void UpdateOrientation();                                         // Called on the Opc Pre-Step to update positions and orientations
    void CreatePortRotMatrix(MATRIX3 &OriMx, VECTOR3 &OriDir, VECTOR3 &OriRot); // Calculates OriMx to transform ship coord system to port coord system
    bool GetFirstGuidanceRect();                                      // Called in the HUD drawer to find the first guidance rectangle (false if none)
    bool GetNextGuidanceRect();                                       // Called in the HUD drawer for the next rectangle until false (end of guidance) 
    void AllStopThrust();                                             // Shut down all thrusters

    double id( double d ) const;                                      // Internalize distance (unit conversion)
    double ed( double d ) const;                                      // Externalize distance (unit conversion)

  private:
    void AutoPilotExecute();                                          // Run all Auto Pilot functions
    void AutoPilotCalibrationExecute();                               // Run AutoPilot Calibration
    void AutoPilotCalibrationCalcRates(int apCalDir, int apCalPass,
      VECTOR3 av1, VECTOR3 av2, VECTOR3 av3, 
      VECTOR3 lv1, VECTOR3 lv2, VECTOR3 lv3,
      double t1, double t2, double t3);                               // Control Rate Calculation (Master switch)
    void AutoPilotCalibrationCalcOneRate(int apCalDir, 
      double v1, double v2, double v3,
      double t1, double t2, double t3);                               // Control Rate Calculation (Individual calc)
    void AutoPilotThrust(                                             // Calculates thrusts for each of the 6 degrees of freedom
      THGROUP_TYPE thrustPositive,THGROUP_TYPE thrustNegative,        // Thruster groups for this move
      double ofs, double ofsRate,                                     // Offset and offset rate for this move
      double &dVs, double &apErr,                                     // Expected full thrust dV per second, and error booster,
      double rateCtrl[12][2],                                         // Rate Control Array
      double &thrust, double &thrustEst, double &rateLim);            // Return thrust, estimate, rate
    bool AutoPilotRateFetch();                                        // Find data for this ship class (returns true if found)
    void AutoPilotRateStore();                                        // Store data for this ship class
    void AutoPilotCtrlLoad();                                         // Initialize Control Rates
    void CalculateApRefMaster(double apRateSet[6][2], double apMassSet[2] ); // Loads apRefMaster coefficients
    void CalculateApRef();                                            // Loads apRefMass, apRefRot and apRefAtt via the apRefMaster
};

//+++++
// MFD Panel Persistence core. One of these is instantiated per MFD panel position used by RV Orientation (e.g. usually left, right, extmfd)
//+++++

class RVO_MCore {
  public:
    // MFD Panel references ... instantiation, mfd position reference and GC
    RVO_MCore(UINT mfdin, RVO_GCore* gcin);
    UINT m;
    RVO_GCore* GC;
 
    // Add MFD panel data here

};

//+++++
// Local Persistence core. One of these is instantiated per Vessel AND MFD panel location. Local defaults for that combination.
//+++++

class RVO_LCore {
  public:
    // Local references ... instantiation, references for vesseland mfd position, and links to the appropriate VC, MC and GC
    RVO_LCore(VESSEL *vin, UINT mfdin, RVO_GCore* gcin);
    VESSEL *v;
    UINT m;
    RVO_GCore* GC;
    RVO_VCore* VC;
    RVO_MCore* MC;

    // Add local vessel+panel data here


    RVO_Buttons B;
    bool showMessage;
    bool okMessagePage;
    char Message[750];

};

//+++++
// Global Persistence core. One of these is instantiated for the whole orbiter session, on the first launch of RV Orientation
//+++++

class RVO_GCore {
  public:
    // Global references ... instantiation and a link to the persistence library (running the linked lists)
    RVO_GCore();
    MFDPersist P;

    // Add global RV Orientation core data here


    RVO_VCore* VC;    // Focus Vessel core

};


/**
 * \ingroup vec
 * \brief Multiplication of vector with vector
 * \param a vector operand
 * \param b vector operand
 * \return Result of element-wise a*b.
 */
inline double operator* (const VECTOR3 &a, const VECTOR3 &b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

#endif // _RVO_CORE_CLASSES