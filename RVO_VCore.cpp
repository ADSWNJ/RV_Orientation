// ==============================================================
//
//	Rendezvous Orientation MFD (Vessel Core Persistence)
//	====================================================
//
//	Copyright (C) 2013-2018	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See RV_Orientation.cpp
//
// ==============================================================

#include "RVO_Cores.hpp"
#include "ParseFunctions.h"
#include <stdio.h>

RVO_VCore::RVO_VCore(VESSEL *vin, RVO_GCore* gcin) {
  GC = gcin;
  v = vin;

  showHUD = 2;
  firstCalc = true;
  mode = 0; 
  units = true;

  oldSimT = 0.0;
  simStepAvg = 1.0;                               // Until we set a real average, assume it's 1.0 secs
  simStepTimesPtr = -1;                           // Init mode
  AutoPilotCtrlLoad();                            // Initialize the autopilot controls

  // Default - no target or port set
  strcpy_s(TargetText,50,"");
  PortNumber=-1;
  ApproachWaypointDistance = 500;
  ApproachWaypointOfsDistance = 250;
  tv = 0;
  hDock = 0;
  apRefRatesLoaded = AutoPilotRateFetch();        // Attempt to load reference rates for our ship 
  if (apRefRatesLoaded) {
    // Calculate the Master Reference set from our rates and weights
    CalculateApRefMaster(apCalMasterRate,apCalMasterMass);

    // Calculate our current reference rates for this weight
    CalculateApRef();
  }
  apApcArmed = false;
  apRotArmed = false;
  apAttArmed = false;
  apAppArmed = false;
  apCalComplete = false;
  apCalInit = false;
  apToggle = 0;
  apAppConeOK = false;
  apAppRMax = 1.0;
  apAppPAtten = 1.0;
  dumpAP = false;
  maxDirErr = 1.0;
  maxPosErr = 1.0;
  apRotErr = _V(1.0,1.0,1.0);
  apAttErr = _V(1.0,1.0,1.0);
  curWP = 0;                                              // WP 0 is the target port
  rtvWPlat[curWP] = 0.0;
  rtvWPlong[curWP] = 0.0;
  rtvWPpos[curWP] = _V(0.0,0.0,0.0);
  rtvWPdir[curWP] = _V(0.0,0.0,1.0);
  rtvWProt[curWP] = _V(0.0,1.0,0.0);
  oriRVel = 0.0;
  guideRectDump = false;

  // HUD Perspective Param
  hpp_z = 1.0;

  // Get our port (assuming always port 0 for our docking)
  hMyDock = v->GetDockHandle(0);
  v->GetDockParams(hMyDock, dPos, dDir, dRot);

  // Check to see if we are docked
  hTgtV = v->GetDockStatus(hMyDock);
  if (hTgtV) {
    // We are docked. Get the vessel's data
    tv = oapiGetVesselInterface(hTgtV);
    strcpy_s(TargetText, 50, tv->GetName());
    MaxPorts = tv->DockCount();

    // Iterate through the target's docks to find ourselves
    OBJHANDLE hTgtDockV;
    for (int i=0; i<MaxPorts; i++) {
      hDock = tv->GetDockHandle(i);
      if (!hDock) continue;                  // Couldn't find the dock??
      hTgtDockV = tv->GetDockStatus(hDock);
      if (hTgtDockV) {
        // Found a ship attached
        if (oapiGetVesselInterface(hTgtDockV) == v) {
          // It's us!
          PortNumber = i;
          tv->GetDockParams(hDock, rtvdPos, rtvdDir, rtvdRot);
          CreatePortRotMatrix(rtvPortOri,rtvdDir,rtvdRot);
          break;
        }
      }
    }
  }

  CreatePortRotMatrix(rPortOri, dDir, dRot);

  return;
};



void RVO_VCore::UpdateOrientation() {

  //
  // Heart and soul of RV Orientation. This is where we compute the orientation offsets
  //

  double thisSimT;

  thisSimT = oapiGetSimTime();  

  if (thisSimT < (simT + 0.10)) return; // we have updated orientation already from another active RV Orientation MFD or less than 0.1 sec ago

  simT = thisSimT;
  if ((!tv)||(!hDock)) return; // No target vehicle or dock selected so far

  // Find orientation of us w.r.t. the target's port coord system
  v->Local2Global(dPos,gPos);                               // Get our port in global coords
  tv->Global2Local(gPos,rtvOurPos);                         // ... and flip into target ship coords 
  rtvOurPos = rtvOurPos - rtvdPos;                          // Translate over to target ship dock origin
  rtvOurPos = mul(rtvPortOri,rtvOurPos);                    // ... and rotate into target port coord system
  if ((abs(rtvOurPos.x) < 0.01) &&                          // Are we right on the dock?
      (abs(rtvOurPos.y) < 0.01) && 
      (abs(rtvOurPos.z) < 0.01)) {
    mode = 0;
    rTgtPos = rtvdPos;                                      // DOCK mode - the dock is the target
    rTgtDir = rTgtPos - rtvdDir;                            // Make direction vector into a unit distance towards (or into) target port
    rTgtRot = rTgtPos + rtvdRot;                            // Turn the rotation vector into an alignment point on the dock ring
  }
  rtvOurRange = sqrt(rtvOurPos.x*rtvOurPos.x+rtvOurPos.y*rtvOurPos.y+rtvOurPos.z*rtvOurPos.z); // Find our range
  rtvOurLong = atan2(rtvOurPos.x,rtvOurPos.z)*DEG;          // 'Longitude' is our x offset w.r.t. our front on position (i.e. primary WP approach is {0,0,1})
  rtvOurLat = asin(rtvOurPos.y/rtvOurRange)*DEG;            // 'Latitude' is our y offset w.r.t. our front on position (i.e. primary WP approach is {0,0,1})
//sprintf_s(oapiDebugString(),128,"Range %.2f, Long %.2f, Lat %.2f, X %.2f, Y %.2f, Z %.2f", rtvOurRange, rtvOurLong, rtvOurLat, rtvOurPos.x, rtvOurPos.y, rtvOurPos.z);

  if (mode < 3) {

    // Determine relative target in PYRXYZ and rates of these. 
    // Modes -1, 0, 1, 2 all reference the target ship for position
   
    switch (mode) {
    case 2:
      // Spherical WP mode ... by far the most complex. We want to select one of 61 waypoints around the target, those being
      // 11 WP on the target 'equatorial' plane (30,60,90,120,150,180,-150,-120,-90,-60,-30), 12 on 'lat' -60, 12 on lat -30, 12 on lat 30, 12 on lat 60,
      // plus the 'north' and 'south' poles on the port's sphere. The special case is 0 by 0 ... which is our primary WP and where we fall through to WP mode (mode 1)

      if (rtvOurLat > 90.0) {
        rtvOurLat = 180.0-rtvOurLat;
      } else if (rtvOurLat < -90.0) {
        rtvOurLat = -180.0 - rtvOurLat;
      }
      if (apSWPselect) {                                        // Selects optimal WP
        apSWPselect = false;

        curWP = 0;                                              // WP 0 is the target port
        rtvWPlat[curWP] = 0.0;
        rtvWPlong[curWP] = 0.0;
        rtvWPpos[curWP] = _V(0.0,0.0,0.0);
        rtvWPrng[curWP] = 0;
        rtvWPdir[curWP] = _V(0.0,0.0,1.0);
        rtvWProt[curWP] = _V(0.0,1.0,0.0);
        rtvWPdist[curWP] = ApproachWaypointDistance;

        rtvWPoriL[curWP] = _M(1,0,0,0,1,0,0,0,1);

        curWP++;                                                // WP 1 is the extended port alignment for final approach
        rtvWPlat[curWP] = 0.0;
        rtvWPlong[curWP] = 0.0;
        rtvWPpos[curWP] = _V(0.0,0.0,ApproachWaypointDistance);
        rtvWPrng[curWP] = ApproachWaypointDistance;


        int optimSel = 0;                                       // Selects direction
        double optimWP = 0.0;                                   // Hunting for optimal WP - starting at 0 longitude (if in front of the z-line, go direct to primary WP
        bool optimFound = false;                                // Assume not found
        while ((rtvOurRange > 1.1 * (ApproachWaypointDistance + ApproachWaypointOfsDistance)) && (!optimFound) && (optimSel<7)) {
          MATRIX3 optimRot = _M(cos(-optimWP*RAD),0,sin(-optimWP*RAD),  0,1,0,  -sin(-optimWP*RAD),0,cos(-optimWP*RAD)); // Set up the temp rot matrix (-ve angle to bring to front)
          VECTOR3 optimPos = mul(optimRot,rtvOurPos);           // Apply to our waypoint
          if (optimPos.z >= 1.0 * (ApproachWaypointDistance + ApproachWaypointOfsDistance)) {   // We are in front of this waypoint, so this is optimal
            optimFound = true;  
            rtvWPlongStart = optimWP;                                // Select it
            rtvWPlatStart = 0.0;                                     // And we can track to zero latitude, straight in
          } else {
            optimSel++;
            switch (optimSel) {                                 // We are checking each of these node points to see if we are beyond a tangent line from it. If so, no need to go in
            case 1:                                             // tight to the sphere and run all the way round ... we should go direct to this node point
              optimWP = -30.0;                                  // Are we in front of 30 left
              break;
            case 2:
              optimWP = 30.0;                                   // Are we in front of 30 right
              break;
            case 3:
              optimWP = -60.0;                                  // Are we in front of 60 left
              break;
            case 4:
              optimWP = 60.0;                                   // Are we in front of 60 right
              break;
            case 5:
              optimWP = -90.0;                                  // Are we in front of 90 left
              break;
            case 6:
              optimWP = 90.0;                                   // Are we in front of 90 right
              break;
            case 7:
              break;                                            // Default to pick a node on the WP sphere
            }
          }
        }

        if (!optimFound) {
          rtvWPlongStart = floor((rtvOurLong/30.0)+0.5)*30.0;   // Target WP long
          if (rtvWPlongStart<-170.0) rtvWPlongStart = 180.0;    // Correct -180.0 degrees into 180.0
          rtvWPlatStart = floor((rtvOurLat/15.0)+0.50)*15.0;    // Target WP lat
          if (rtvWPlatStart>75.0) rtvWPlatStart = 75.0;         // Don't want to go to 90 up
          if (rtvWPlatStart<-75.0) rtvWPlatStart = -75.0;       // ... or 90 down because we want the longitude to mean something 
        }

        // Set up parameters for the path around the sphere 
        double rtvWPlongNext = 0.0, rtvWPlatNext = 0.0;         // Starting point is WP1 (0,0)
        double rtvWPlongSign = (rtvWPlongStart < 0.0 ? -1.0 : 1.0); // Direction of longitude traversal
        double rtvWPlatSign = (rtvWPlatStart < 0.0 ? -1.0 : 1.0); // Ditto latitude
        double rtvWPlongInc = 30.0 * rtvWPlongSign;             // WP2 and beyond = 30 degrees longitude
        double rtvWPlatInc = 15.0 * rtvWPlatSign;               //  ... and 15 degreed latitude
        double rtvWPlongTwo = rtvWPlongInc;                     // WP2 longitude 
        double rtvWPlatTwo = rtvWPlatInc;                       // WP2 latitude
        double modAppWPdist = ApproachWaypointDistance;         // Modified app WP distance, adding 100 on for WP2 and beyond
        bool intraWP1WP2 = true;                                // Between WP2 and WP1 ... special rules apply
        bool intraWP1WP2LongFlat = (abs(rtvWPlongNext - rtvWPlongStart)< 0.1);  // Check to see if we are ramping long
        bool intraWP1WP2LatFlat = (abs(rtvWPlatNext - rtvWPlatStart)< 0.1);     // Check to see if we are ramping lat
        int intraWP1WP2IncCounter = 0;                          // Increments from 0 to 3. 0 = +0 lat,+5 long, a+60 range. 1 = +5,+5,a+60. 2 = +5,+10,a+80. 3 = +5,+10,a+100.  

        while ((abs(rtvWPlongNext - rtvWPlongStart)> 0.1) || (abs(rtvWPlatNext - rtvWPlatStart)> 0.1)) {  // Track out along the SWP WP's from {0,0} to the start point
          if (intraWP1WP2) {
            switch (intraWP1WP2IncCounter) {
            case 0:
              rtvWPlongNext = (intraWP1WP2LongFlat ? 0.0 : 15.0 * rtvWPlongSign);
              rtvWPlatNext = (intraWP1WP2LatFlat ? 0.0 : 15.0 * rtvWPlatSign);
              modAppWPdist = ApproachWaypointDistance + ApproachWaypointOfsDistance;
              break;
            default:
              rtvWPlongNext = (intraWP1WP2LongFlat ? 0.0 : 30.0 * rtvWPlongSign);
              rtvWPlatNext = (intraWP1WP2LatFlat ? 0.0 : 15.0 * rtvWPlatSign);
              intraWP1WP2 = false;
              break;
            }
            intraWP1WP2IncCounter++;
          } else {
            if (abs(rtvWPlongNext - rtvWPlongStart)> 0.1) {
              rtvWPlongNext += rtvWPlongInc;
            }
            if (abs(rtvWPlatNext - rtvWPlatStart)> 0.1) {
              rtvWPlatNext += rtvWPlatInc;
            }
          }
          prWP = curWP++;                                          // Move to next SWP
          rtvWPlat[curWP] = rtvWPlatNext;                          // Store new SWP lat
          rtvWPlong[curWP] = rtvWPlongNext;                        //  ... and long

          rtvWProtLat = _M(1,0,0,  0,cos(-rtvWPlatNext*RAD),-sin(-rtvWPlatNext*RAD),  0,sin(-rtvWPlatNext*RAD),cos(-rtvWPlatNext*RAD));   // Rotation of latitude
          rtvWProtLong = _M(cos(rtvWPlongNext*RAD),0,sin(rtvWPlongNext*RAD),  0,1,0,  -sin(rtvWPlongNext*RAD),0,cos(rtvWPlongNext*RAD));  // and longitude

          rtvWPpos[curWP] = _V(0.0,0.0,modAppWPdist*1.01733);      // Initial pos target is the primary WP (extended out to the modified range)
                                                                   // The 1.01733 story is interesting! You fly straight line sectors between WP's, so it looks like the course is always just
                                                                   // inside the target range, as the stright line range mid-sector is cos(30/2) times radius, versus the desired 1.00 * radius.
                                                                   // So we are somewhere between 1.00000 and 0.96593 of radius, averaging  0.982965 of radius. So multiply range by the reciprocal
                                                                   // 1 / 0.982965 = 1.01733 to get an average flight track on our desired range. 
          rtvWPrng[curWP] = modAppWPdist;
          rtvWPpos[curWP] = mul(rtvWProtLat,rtvWPpos[curWP]);      // Elevate to latitude
          rtvWPpos[curWP] = mul(rtvWProtLong,rtvWPpos[curWP]);     // ... and rotate to longitude

          // Now we know the next WP, we can calculate the previous WP's direction, rotation, and range, and then create the orientation matrix
          VECTOR3 ofsDir = rtvWPpos[curWP] - rtvWPpos[prWP];                                                              // Find the direction between cur and prev WP
          normalise(ofsDir);                                                                                              // We now want the offset to be a direction
          double ofsLat = asin(ofsDir.y)*DEG;                                                                             // Lat is arcsine opp / hypotenuse but we have a unit length hypotenuse
          double ofsLong = atan2(ofsDir.x, ofsDir.z)*DEG;                                                                 // Long ... use atan2 to avoid problems on cardinal points
          MATRIX3 ofsRotLat = _M(1,0,0,  0,cos(-ofsLat*RAD),-sin(-ofsLat*RAD),  0,sin(-ofsLat*RAD),cos(-ofsLat*RAD));     // Make into rotation lat matrix
          MATRIX3 ofsRotLong = _M(cos(ofsLong*RAD),0,sin(ofsLong*RAD),  0,1,0,  -sin(ofsLong*RAD),0,cos(ofsLong*RAD));    // ... and long matrix
          VECTOR3 WPdir = mul(ofsRotLong,mul(ofsRotLat,_V(0,0,1)));                                                       // Check ... does WPdir equal ofsDir? Should do
          VECTOR3 WProt = mul(ofsRotLong,mul(ofsRotLat,_V(0,1,0)));                                                       // Rotation from default up, according to pitch
          rtvWPdir[prWP] = ofsDir;                                            // Load previous dir
          rtvWProt[prWP] = WProt;                                             // Load previous rot
          rtvWPdist[prWP] = length(rtvWPpos[curWP] - rtvWPpos[prWP]);         // Load previous range
          CreatePortRotMatrix(rtvWPoriL[prWP],rtvWPdir[prWP],rtvWProt[prWP]); // Orientation matrix from port coords to WP coords

        }

        // Find last WP, then set the dir/rot on prev.
        prWP = curWP++;                                                       // Last point
        rtvWPlatNext = rtvOurLat;
        rtvWPlongNext = rtvOurLong; 
        rtvWPlat[curWP] = rtvWPlatNext;                          // Store new SWP lat
        rtvWPlong[curWP] = rtvWPlongNext;                        //  ... and long
        rtvWProtLat = _M(1,0,0,  0,cos(-rtvWPlatNext*RAD),-sin(-rtvWPlatNext*RAD),  0,sin(-rtvWPlatNext*RAD),cos(-rtvWPlatNext*RAD));   // Rotation of latitude
        rtvWProtLong = _M(cos(rtvWPlongNext*RAD),0,sin(rtvWPlongNext*RAD),  0,1,0,  -sin(rtvWPlongNext*RAD),0,cos(rtvWPlongNext*RAD));  // and longitude

        rtvWPpos[curWP] = _V(0.0,0.0,rtvOurRange);
        rtvWPpos[curWP] = mul(rtvWProtLat,rtvWPpos[curWP]);         // Elevate to latitude
        rtvWPpos[curWP] = mul(rtvWProtLong,rtvWPpos[curWP]);        // ... and rotate to longitude

        VECTOR3 ofsDir = rtvWPpos[curWP] - rtvWPpos[prWP];                                                              // Find the direction between cur and prev WP
        normalise(ofsDir);                                                                                              // We now want the offset to be a direction
        double ofsLat = asin(ofsDir.y)*DEG;                                                                             // Lat is arcsine opp / hypotenuse but we have a unit length hypotenuse
        double ofsLong = atan2(ofsDir.x, ofsDir.z)*DEG;                                                                 // Long ... use atan2 to avoid problems on cardinal points
        MATRIX3 ofsRotLat = _M(1,0,0,  0,cos(-ofsLat*RAD),-sin(-ofsLat*RAD),  0,sin(-ofsLat*RAD),cos(-ofsLat*RAD));     // Make into rotation lat matrix
        MATRIX3 ofsRotLong = _M(cos(ofsLong*RAD),0,sin(ofsLong*RAD),  0,1,0,  -sin(ofsLong*RAD),0,cos(ofsLong*RAD));    // ... and long matrix
        VECTOR3 WPdir = mul(ofsRotLong,mul(ofsRotLat,_V(0,0,1)));                                                       // Check ... does WPdir equal ofsDir? Should do
        VECTOR3 WProt = mul(ofsRotLong,mul(ofsRotLat,_V(0,1,0)));                                                       // Rotation from default up, according to pitch
        rtvWPdir[prWP] = ofsDir;                                            // Load previous dir
        rtvWProt[prWP] = WProt;                                             // Load previous rot
        rtvWPdist[prWP] = 0.75* length(rtvWPpos[curWP] - rtvWPpos[prWP]);   // Load previous range
        CreatePortRotMatrix(rtvWPoriL[prWP],rtvWPdir[prWP],rtvWProt[prWP]); // Orientation matrix from port coords to WP coords

        curWP--;          // Throw away last point

      }

      // Waypoint reached check...
      rtvOurWPrange = dist(rtvOurPos, rtvWPpos[curWP]);
      if (rtvOurWPrange < 22.0) {
        if (curWP==1) {                                         // Flip to DOCK mode (skip WP)
          mode = 0;
          break;
        }
        curWP--;                                                // Reduce curWP towards 0
        rtvOurWPrange = dist(rtvOurPos, rtvWPpos[curWP]);
      }

      // remote target position is the waypoint calc somewhere on the sphere 
      rTgtPos = rtvWPpos[curWP];                                // Target pos is cur WP in port coords
      rTgtPos = tmul(rtvPortOri,rTgtPos);                       // Convert back into ship coords
      rTgtPos += rtvdPos;                                       // And add on the port offset   

      // remote direction is towards SWP
      rTgtDir = -rtvWPdir[curWP];                               // Point into our port, in target port coords
      rTgtRot = rtvWProt[curWP];                                // Rotation is in target port coords

      // Dir/Rot blend as we get closer to the target. Start at 50m to go, full mix by 22m to go
      if ((rtvOurWPrange < 50.0)&&(curWP>0)) {
        int nxWP = curWP - 1;                                   // Look at the next direction
        double mix = (rtvOurWPrange > 10.0? (50.0 - rtvOurWPrange)/(50.0 - 22.0) : 1.0); // Ratio of old WP : new WP ... blending in change of direction as we approach the WP
        VECTOR3 rNxTgtDir, rNxTgtRot;                           // Next target direction and rotation
        
        rNxTgtDir = -rtvWPdir[nxWP];                            // Point into our port, in port coords
        rTgtDir = (rTgtDir * (1.0-mix)) + (rNxTgtDir * mix);    // Blend in the delta

        rNxTgtRot = rtvWProt[nxWP];                             // Point into our port, in port coords
        rTgtRot = (rTgtRot * (1.0-mix)) + (rNxTgtRot * mix);    // Blend in the delta
      }

      // remote direction is towards SWP
      rTgtDir = tmul(rtvPortOri,rTgtDir);                       // Convert back into ship coords
      rTgtDir += rtvdPos;                                       // Add back in the port coords ... rTgtDir now is a point inside the port in ship coords

      rTgtRot = tmul(rtvPortOri,rTgtRot);                       // Convert back into ship coords
      rTgtRot += rtvdPos;                                       // Add back in the port offset ... rTgtRot now is a direction relative to ship coords
 

      if ((abs(rtvWPlong[curWP]) > 0.001) || (abs(rtvWPlat[curWP]) > 0.001) || (rtvOurWPrange > 10.0) ) {   // Is this NOT the primary WP, or if it is ... track to within 10m before drop through


/*sprintf_s(oapiDebugString(),256,"X: %.0f (Long %.2f, 100X %.02f) Y: %.0f (Lat %.2f, 100Y %.02f) Z: %.0f (100Z %.02f)   WPX: %0.f (Long %.0f) WPY: %0.f (Lat %.0f) WPZ: %0.f", 
  rtvOurPos.x,  rtvOurLong, (100*rtvOurPos.x/sqrt(rtvOurPos.x*rtvOurPos.x+rtvOurPos.y*rtvOurPos.y+rtvOurPos.z*rtvOurPos.z)), rtvOurPos.y,  rtvOurLat,
  (100*rtvOurPos.y/sqrt(rtvOurPos.x*rtvOurPos.x+rtvOurPos.y*rtvOurPos.y+rtvOurPos.z*rtvOurPos.z)), rtvOurPos.z,
  (100*rtvOurPos.z/sqrt(rtvOurPos.x*rtvOurPos.x+rtvOurPos.y*rtvOurPos.y+rtvOurPos.z*rtvOurPos.z)), rTgtPosPortCoordSaved.x, rtvWPLong, rTgtPosPortCoordSaved.y, rtvWPLat, rTgtPosPortCoordSaved.z);
*/

        break;
      } else {                                                    // We found the primary WP.
        mode = 1;                                                 // Switch to WP mode and fall through
      }
    case 1:
      // MODE 1 = WP mode. Straight on to port, some distance out. 
      curWP = 0;                                              // WP 0 is the target port
      rtvWPlat[curWP] = 0.0;
      rtvWPlong[curWP] = 0.0;
      rtvWPpos[curWP] = _V(0.0,0.0,0.0);
      rtvWPdir[curWP] = _V(0.0,0.0,1.0);
      rtvWProt[curWP] = _V(0.0,1.0,0.0);
      rtvWPoriL[curWP] = _M(1,0,0,0,1,0,0,0,1);
      rtvWPdist[curWP] = ApproachWaypointDistance;

      curWP++;                                                // WP 1 is the primary target
      rtvWPlat[curWP] = 0.0;
      rtvWPlong[curWP] = 0.0;
      rtvWPpos[curWP] = _V(0.0,0.0,ApproachWaypointDistance);
      rtvWPdir[curWP] = _V(0.0,0.0,1.0);
      rtvWProt[curWP] = _V(0.0,1.0,0.0);
      rtvWPoriL[curWP] = _M(1,0,0,0,1,0,0,0,1);
      rtvWPdist[curWP] = dist(rtvOurPos,rtvWPdir[curWP]);        // Store the WP distance

      rTgtPos = rtvdPos + (rtvdDir * ApproachWaypointDistance);   // WP mode - add approach dist on to the dock
      rTgtDir = rtvdPos - rtvdDir;                                // Make direction vector into a unit distance towards (or into) target port
      rTgtRot = rtvdPos + rtvdRot;                                // Turn the rotation vector into an alignment point on the dock ring

      if ((abs(rtvOurLong) < 0.1) && (abs(rtvOurLat) < 0.1) &&
          (abs(rtvOurRange) < (1.03 * ApproachWaypointDistance))) {

        mode = 0;                                                 // We are at the primary WP ... switch to dock mode and fall through

      } else if ((abs(rtvOurLong) < 5.0) && (abs(rtvOurLat) < 5.0) &&
          (abs(rtvOurRange)< (0.75 * ApproachWaypointDistance))) {

        mode = 0;                                                 // We are on approach ... switch to dock mode and fall through

      } else {

        break;
      }

    case 0:
      // MODE 0 = DOCK mode. Target is the dock port on the target vehicle.
      curWP = 0;                                              // WP 0 is the target port
      rtvWPlat[curWP] = 0.0;
      rtvWPlong[curWP] = 0.0;
      rtvWPpos[curWP] = _V(0.0,0.0,0.0);
      rtvWPdir[curWP] = _V(0.0,0.0,1.0);
      rtvWProt[curWP] = _V(0.0,1.0,0.0);
      rtvWPoriL[curWP] = _M(1,0,0,0,1,0,0,0,1);
      rtvWPdist[curWP] = ApproachWaypointDistance;

      rTgtPos = rtvdPos;                                        // DOCK mode - the dock is the target
      rTgtDir = rtvdPos - rtvdDir;                              // Make direction vector into a unit distance towards (or into) target port
      rTgtRot = rtvdPos + rtvdRot;                              // Turn the rotation vector into an alignment point on the dock ring
      break;

    case -1:
      // MODE -1 = APC mode. Target is the origin of the target vehicle to minimize wobble affecting our calibrations
      curWP = 0;                                              // WP 0 is the target port
      rtvWPlat[curWP] = 0.0;
      rtvWPlong[curWP] = 0.0;
      rtvWPpos[curWP] = _V(0.0,0.0,0.0);
      rtvWPdir[curWP] = _V(0.0,0.0,-1.0);
      rtvWProt[curWP] = _V(0.0,1.0,0.0);
      rtvWPoriL[curWP] = _M(1,0,0,0,1,0,0,0,1);
      rtvWPdist[curWP] = ApproachWaypointDistance;
      rTgtPos = _V(0.0,0.0,0.0);                                // APC mode - just the origin
      rTgtDir = - rtvdDir;                                      // Make direction vector into a unit distance beyond the vessel core
      rTgtRot = rTgtPos + rtvdRot;                              // Turn the rotation vector into an alignment point referenced to the vessel core
      break;

    }

    // Find our target vehicle port XYZ offset in local port coords
    tv->Local2Global(rtvdPos,gtvdPos);      // Make target port into a global coord
    v->Global2Local(gtvdPos,ltvdPos);       // And flip it back into a target in OUR local coordinate system
    ltvdPos = ltvdPos - dPos;               // Translate it onto our local dock origin
    ltvdPos = mul(rPortOri, ltvdPos);       // Rotate around our port orientation to get into our local port coordinate system

    // Find our orientation target XYZ offset in local port coords
    tv->Local2Global(rTgtPos,gTgtPos);      // Make target into a global coord
    v->Global2Local(gTgtPos,lTgtPos);       // And flip it back into a target in OUR local coordinate system
    oriPos = lTgtPos - dPos;                // Translate it onto our local dock origin
    oriPos = mul(rPortOri, oriPos);         // Rotate around our port orientation to get into our local port coordinate system
  
    // Orientation is a direction relative to us
    // Find orientation to target port alignment and rotation
    tv->Local2Global(rTgtDir,gTgtDir);      // Flip target dir to global
    v->Global2Local(gTgtDir,lTgtDir);       // ... and back to local in our frame
    lTgtDir = lTgtDir - dPos;               // Translate onto our dock origin
    lTgtDir = mul(rPortOri, lTgtDir);       // ... and rotate on our dock coordinate system
    lTgtDir = lTgtDir - ltvdPos;            // Subtract the target dock coords, to make the direction point back into a unit vector (on our dock coord system)
  
    // For roll offset, use the Rot vector
    tv->Local2Global(rTgtRot,gTgtRot);      // Flip target rot to global
    v->Global2Local(gTgtRot,lTgtRot);       // ... and back to local in our frame
    lTgtRot = lTgtRot - dPos;               // Translate onto our dock origin
    lTgtRot = mul(rPortOri, lTgtRot);       // ... and rotate on our dock coordinate system 
    lTgtRot = lTgtRot - ltvdPos;            // Subtract the target dock coords, to make the direction point back into a unit vector (on our dock coord system)



  } else {

    // Mode 3 ... align to the negative RVEL

    curWP = 0;                                              // WP 0 is the target port
    rtvWPlat[curWP] = 0.0;
    rtvWPlong[curWP] = 0.0;
    rtvWPpos[curWP] = _V(0.0,0.0,0.0);
    rtvWPdir[curWP] = _V(0.0,0.0,1.0);
    rtvWProt[curWP] = _V(0.0,1.0,0.0);
    rtvWPoriL[curWP] = _M(1,0,0,0,1,0,0,0,1);

    tv->GetGlobalVel(gtvVel);               // Find target's absolute velocity
    v->GetGlobalVel(gVel);
    gtvRVel = gtvVel - gVel;                // Convert to relative absolute velocity
//    normalise(gtvRVel);                     // Normalize it (not strictly necessary, but feels right as we are dealing with direction vectors)
    MATRIX3 rm;                             // We need out orientation matrix
    v->GetRotationMatrix(rm);               // ... to align the RVel to the ship
    ltvRVel = tmul(rm, gtvRVel);            // Transpose multiply, as we are going global to local. Note not useing Global2Local as we do not want the
                                            // global position to be added .. i.e. just a rotation please.  
    lTgtDir = mul(rPortOri, ltvRVel);       // And then rotate relative to our port.
    lTgtPos = lTgtDir;                      // Pos doesn't make sense for RVEL orientation, but give the calc something anyway to work on below
    v->Global2Local(_V(0.0,1.0,0.0),lTgtRot);     // Use a global "up" (unit up in +y) to orient the roll for the AP's benefit
    lTgtRot = mul(rPortOri, lTgtRot);       // And Drop it into our port coordinates

//sprintf_s(oapiDebugString(),128,"GTVRVel.V %.3f TgtDir.x %.3f TgtDir.y %.3f TgtDir.z %.3f",
//sqrt(gtvRVel.x*gtvRVel.x + gtvRVel.y*gtvRVel.y + gtvRVel.z*gtvRVel.z), lTgtDir.x, lTgtDir.y, lTgtDir.z);

  }


  // Common code ... oriDir and oriPos now loaded

  // Pull out the differences in directions
  oriDir.x = DEG* (atan2(lTgtDir.y, lTgtDir.z));    // OriDir.x is PITCH
  oriDir.y = DEG* (atan2(lTgtDir.x, lTgtDir.z));    // OriDir.y is YAW
  oriDir.z = DEG* (atan2(lTgtRot.x, lTgtRot.y));    // OriDir.z is ROLL on the rotation alignment vector
  if (oriDir.x < -180.0) oriDir.x += 360.0;
  if (oriDir.y < -180.0) oriDir.y += 360.0;
  if (oriDir.z < -180.0) oriDir.z += 360.0;

  if (!firstCalc) {

    // Iterate the sim step, saving the previous positions
    lastSimStep = simT-oldSimT;
    if (simStepTimesPtr > -1) {
      // update the simstep times
      simStepTimes[simStepTimesPtr] = lastSimStep;
      simStepTimesPtr = (simStepTimesPtr + 1) % 10;
      if (simStepTimesPtr == 0) {
        simStepAvg=0.0;
        for (int i=0; i<10; i++) {
          simStepAvg += simStepTimes[i];
        }
        simStepAvg /= 10.0;
      }
    } else {
      // Init the sim step times
      simStepTimesPtr = 0;
      for (int i=0; i<10; i++) {
        simStepTimes[i] = lastSimStep;
      }
      simStepAvg = lastSimStep;
    }

    simStep = simT - oldSimT;
    oriPosRate = (oriPos - oldPos) / simStep;       // oriPos is relative position, oriPosRate is relative position rate 
    oriDirRate = (oriDir - oldDir) / simStep;       // diffdir is relative direction, oriDirRate is relative direction rate
    oriRVel = sqrt(oriPosRate.x*oriPosRate.x + oriPosRate.y*oriPosRate.y + oriPosRate.z*oriPosRate.z); // Find our RVel
//sprintf_s(oapiDebugString(),128,"OriRVel %.2f, PosRate.x %.2f, PosRate.y %.2f, PosRate.z %.2f", oriRVel, oriPosRate.x, oriPosRate.y, oriPosRate.z);

    if (abs(oriPos.z) < 10.0) {                                   // Within 10m of target ... 0.30 deg and 10cm control tolerances
      maxDirErr = 0.30;
      maxPosErr = 0.10;
    } else {                                                      // Else cone out to 10 deg and 5 m error at 1km
      maxDirErr = (30.0 / 1000.0) * abs(oriPos.z);
      maxPosErr = (10.0 / 1000.0) * abs(oriPos.z);
    }
    if (mode > 0) {                                               // Much less need to be precise for WP, SWP and RVEL modes
      maxDirErr *= 10.0;   
      maxPosErr *= 10.0;
    }

    if (v->GetDockStatus(hMyDock) == 0) {           // If we are not docked, check autopilots
      if (apApcArmed) {                             // AP Calibration active
        AutoPilotCalibrationExecute();
      } else {
        if (apRotArmed || apAttArmed) {               // Don't need to check apAppArmed as it can only be active with both ROT and ATT armed
          AutoPilotExecute();                         // Run the autopilot
        }
      }
    } else {                                        // ... else docked, so disarm autopilots
      apRotArmed = false;
      apAttArmed = false;
      apAppArmed = false;
      AllStopThrust();
    }
  } else {
    oriPosRate = _V(0.0,0.0,0.0);                         // First time only ... pos rate and dir rate initialized to zeros
    oriDirRate = _V(0.0,0.0,0.0); 
    firstCalc = false;
  };


  // Make a dead-band on the display params (stops the annoying flickering 0.00 to -0.00 on the displays)
  dispOriDir.x = (abs(oriDir.x) < 0.01? 0 : oriDir.x);
  dispOriDir.y = (abs(oriDir.y) < 0.01? 0 : oriDir.y);
  dispOriDir.z = (abs(oriDir.z) < 0.01? 0 : oriDir.z);
  dispOriPos.x = (abs(oriPos.x) < 0.01? 0 : oriPos.x);
  dispOriPos.y = (abs(oriPos.y) < 0.01? 0 : oriPos.y);
  dispOriPos.z = (abs(oriPos.z) < 0.01? 0 : oriPos.z);

  // Make a dead-band on the display RATE params as well
  dispOriDirRate.x = (abs(oriDirRate.x) < 0.0005? 0 : oriDirRate.x);
  dispOriDirRate.y = (abs(oriDirRate.y) < 0.0005? 0 : oriDirRate.y);
  dispOriDirRate.z = (abs(oriDirRate.z) < 0.0005? 0 : oriDirRate.z);
  dispOriPosRate.x = (abs(oriPosRate.x) < 0.0005? 0 : oriPosRate.x);
  dispOriPosRate.y = (abs(oriPosRate.y) < 0.0005? 0 : oriPosRate.y);
  dispOriPosRate.z = (abs(oriPosRate.z) < 0.0005? 0 : oriPosRate.z);



  oldSimT = simT;
  oldPos = oriPos;
  oldDir = oriDir;

//  VECTOR3 av;
//  v->GetAngularVel(av);
//  av *= DEG;
//  sprintf_s(oapiDebugString(),128,"ANGULAR VELS:  X. %20f Y. %20f Z. %20f", av.x, av.y, av.z);


  return;
}

void RVO_VCore::AutoPilotCtrlLoad() {
  // Initialize Control Rates

  double RotXCtrl[12][2] = {
    {40.00, 1.000}, {20.00, 0.750}, {10.00, 0.500},
    { 4.00, 0.300}, { 2.00, 0.150}, { 1.00, 0.060},
    { 0.40, 0.020}, { 0.30, 0.015}, { 0.20, 0.010},
    { 0.12, 0.006}, { 0.08, 0.004}, { 0.04, 0.002} };
  double RotYCtrl[12][2] = {
    {40.00, 1.000}, {20.00, 0.750}, {10.00, 0.500},
    { 4.00, 0.300}, { 2.00, 0.150}, { 1.00, 0.060},
    { 0.40, 0.020}, { 0.30, 0.015}, { 0.20, 0.010},
    { 0.12, 0.006}, { 0.08, 0.004}, { 0.04, 0.002} };
  double RotZCtrl[12][2] = {
    {40.00, 1.000}, {20.00, 0.750}, {10.00, 0.500},
    { 4.00, 0.300}, { 2.00, 0.150}, { 1.00, 0.060},
    { 0.40, 0.020}, { 0.30, 0.015}, { 0.20, 0.010},
    { 0.12, 0.006}, { 0.08, 0.004}, { 0.04, 0.002} };
  double AttXCtrl[12][2] = {
    {1000.00, 5.000}, {400.00, 3.000}, {100.00, 1.000},
    { 40.00, 1.000}, { 30.00, 0.800}, { 20.00, 0.600},
    { 10.00, 0.400}, {  3.00, 0.150}, {  1.00, 0.080},
    {  0.60, 0.040}, {  0.30, 0.020}, {  0.10, 0.005} };
  double AttYCtrl[12][2] = {
    {1000.00, 5.000}, {400.00, 3.000}, {100.00, 1.000},
    { 40.00, 1.000}, { 30.00, 0.800}, { 20.00, 0.600},
    { 10.00, 0.400}, {  3.00, 0.150}, {  1.00, 0.150},
    {  0.60, 0.040}, {  0.30, 0.020}, {  0.10, 0.005} };
  double AttZCtrl[12][2] = {
    {1000.00,5.000}, {500.00, 2.000}, {100.00, 1.000},
    { 40.00, 0.800}, { 30.00, 0.600}, { 20.00, 0.300},
    { 10.00, 0.200}, {  3.00, 0.100}, {  1.00, 0.080},
    {  0.60, 0.040}, {  0.30, 0.015}, {  0.10, 0.010} };
  double RotXCtrlMode2[12][2] = {
    {40.00, 1.000}, {20.00, 0.750}, {10.00, 0.500},
    { 4.00, 0.300}, { 3.00, 0.200}, { 2.50, 0.150},
    { 2.00, 0.100}, { 1.50, 0.080}, { 1.00, 0.050},
    { 0.80, 0.040}, { 0.60, 0.030}, { 0.50, 0.020} };
  double RotYCtrlMode2[12][2] = {
    {40.00, 1.000}, {20.00, 0.750}, {10.00, 0.500},
    { 4.00, 0.300}, { 3.00, 0.200}, { 2.50, 0.150},
    { 2.00, 0.100}, { 1.50, 0.080}, { 1.00, 0.050},
    { 0.80, 0.040}, { 0.60, 0.030}, { 0.50, 0.020} };
  double RotZCtrlMode2[12][2] = {
    {40.00, 1.000}, {20.00, 0.750}, {10.00, 0.500},
    { 4.00, 0.300}, { 3.00, 0.200}, { 2.50, 0.150},
    { 2.00, 0.100}, { 1.50, 0.080}, { 1.00, 0.050},
    { 0.80, 0.040}, { 0.60, 0.030}, { 0.50, 0.020} };
  double AttXCtrlMode2[12][2] = {
    {1000.00, 5.000}, {400.00, 2.000}, {100.00, 1.000},
    { 40.00, 0.400}, { 30.00, 0.300}, { 20.00, 0.200},
    { 10.00, 0.200}, {  8.00, 0.200}, {  5.00, 0.150},
    {  3.00, 0.080}, {  2.00, 0.050}, {  1.00, 0.020} };
  double AttYCtrlMode2[12][2] = {
    {1000.00, 5.000}, {400.00, 2.000}, {100.00, 1.000},
    { 40.00, 1.000}, { 30.00, 0.800}, { 20.00, 0.600},
    { 10.00, 0.200}, {  8.00, 0.200}, {  5.00, 0.150},
    {  3.00, 0.080}, {  2.00, 0.050}, {  1.00, 0.020} };
  double AttZCtrlMode2[12][2] = {
    {1000.00,5.000}, {400.00, 2.000}, {100.00, 1.000},
    { 65.00, 0.650}, { 55.00, 0.550}, { 45.00, 0.500},
    { 35.00, 0.500}, { 25.00, 0.500}, { 15.00, 0.500},
    { 10.00, 0.500}, {  5.00, 0.500}, {  1.00, 0.500} };

  for (int i=0; i<12; i++) {
    for (int j=0; j<2; j++) {
      apRotXCtrl[i][j] = RotXCtrl[i][j];
      apRotYCtrl[i][j] = RotYCtrl[i][j];
      apRotZCtrl[i][j] = RotZCtrl[i][j];
      apAttXCtrl[i][j] = AttXCtrl[i][j];
      apAttYCtrl[i][j] = AttYCtrl[i][j];
      apAttZCtrl[i][j] = AttZCtrl[i][j];
      apRotXCtrlMode2[i][j] = RotXCtrlMode2[i][j];
      apRotYCtrlMode2[i][j] = RotYCtrlMode2[i][j];
      apRotZCtrlMode2[i][j] = RotZCtrlMode2[i][j];
      apAttXCtrlMode2[i][j] = AttXCtrlMode2[i][j];
      apAttYCtrlMode2[i][j] = AttYCtrlMode2[i][j];
      apAttZCtrlMode2[i][j] = AttZCtrlMode2[i][j];
    }
  }
  return;
}



void RVO_VCore::AutoPilotExecute() {

  //
  // Master Auto Pilot Execution Sequence
  //  
  //

  double zTarget = 0.0;

  apRotActive = false;                                          // For dump purposes
  apAttActive = false;                                          // For dump purposes      
  apAppActive = false;                                          // For dump purposes


  // 0. Clear out any rates we are looking after
  v->SetThrusterGroupLevel(THGROUP_MAIN, 0.0);
  v->SetThrusterGroupLevel(THGROUP_RETRO, 0.0);
  v->SetThrusterGroupLevel(THGROUP_HOVER, 0.0);
  if (apRotArmed) {
    v->SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0.0);
    v->SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0.0);
    v->SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0.0);
    v->SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0.0);
    v->SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0.0);
    v->SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0.0);
  }
  if (apAttArmed) {
    v->SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0.0);
    v->SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0.0);
    v->SetThrusterGroupLevel(THGROUP_ATT_UP, 0.0);
    v->SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0.0);
  }
  if (apAppArmed) {
    v->SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0.0);
    v->SetThrusterGroupLevel(THGROUP_ATT_BACK, 0.0);
  }

  // 1. Reset mass reference calce if mass outside of 0.1% of reference
  if ((abs(v->GetMass() - apRefMass) > 0.001 * apRefMass)) CalculateApRef();


  // 2. Cross-Interlock code
  if (apRotArmed && ((abs(oriDir.x) > 20.0) || (abs(oriDirRate.x > 1.00)))) {             // All Rot until Pitch settles down step 1
    apAttAllow = false;
  } else if (apRotArmed && ((abs(oriDir.y) > 20.0) || (abs(oriDirRate.y > 1.00)))) {      // All Rot until Yaw settles down step 1
    apAttAllow = false;
  } else if (apRotArmed && ((abs(oriDir.z) > 20.0) || (abs(oriDirRate.z > 1.00)))) {      // All Rot unitl Roll settles down step 1
    apAttAllow = false;

  } else if (apAttArmed && ((abs(oriPos.x)>200.0)||(abs(oriPosRate.x)>10.0))) {           // Once Rot settled, all Att until L-R settles down
    apAttAllow = true;
    apToggle = 5;
  } else if (apAttArmed && ((abs(oriPos.y)>200.0)||(abs(oriPosRate.y)>10.0))) {           // Once Rot settled, all Att until U-D settles down
    apAttAllow = true;
    apToggle = 5;
/*  } else if (apAttArmed && ((abs(oriPos.z)>200.0)||(abs(oriPosRate.z)>10.0))) {         // Once Rot settled, all Att until F-B settles down
    apAttAllow = true;
    apToggle = 5;
*/
  } else if (apRotArmed && ((abs(oriDir.x) > 2.0) || (abs(oriDirRate.x > 0.10)))) {       // All Rot until Pitch settles down step 2
    apAttAllow = false;
  } else if (apRotArmed && ((abs(oriDir.y) > 2.0) || (abs(oriDirRate.y > 0.10)))) {       // All Rot until Yaw settles down step 2
    apAttAllow = false;
  } else if (apRotArmed && ((abs(oriDir.z) > 2.0) || (abs(oriDirRate.z > 0.10)))) {       // All Rot unitl Roll settles down step 2
    apAttAllow = false;

  } else if (apRotArmed && ((abs(oriDir.x) > 0.2) || (abs(oriDirRate.x > 0.05)))) {       // 1 in 3 rot until Pitch accurate step 3
    apAttAllow = apAttArmed;
    if (apToggle>2) apToggle = 2;
  } else if (apRotArmed && ((abs(oriDir.y) > 0.2) || (abs(oriDirRate.y > 0.05)))) {       // 1 in 3 rot until Yaw accurate step 3
    apAttAllow = apAttArmed;
    if (apToggle>2) apToggle = 2;
  } else if (apRotArmed && ((abs(oriDir.z) > 0.2) || (abs(oriDirRate.z > 0.05)))) {       // 1 in 3 rot until Roll accurate step 3
    apAttAllow = apAttArmed;
    if (apToggle>2) apToggle = 2;

  } else {
    apAttAllow = apAttArmed;                                                              // Else ... 1 in 5 Rot (keeps Att accurate).
    if (!apRotArmed) apToggle = 5;
  }

  // Interlocks in place now

  if (apRotArmed && (!apAttAllow || !apToggle)) {

    // 3. Run the ROT AP solid until it's close to target, then once every 2 or 5 times

    apRotActive = true;

    // Pitch Up/Down Control
    if ((mode!=2) ||                                                              // Always do it in non SWP mode
        ((mode==2) && (oriRVel > 30.0) && (abs(oriDir.x) > 0.15)) ||               // Do it to 0.50 when RVel high
        ((mode==2) && (oriRVel <= 30.0)) ) {                                      // Always do it when RVel low
      AutoPilotThrust(THGROUP_ATT_PITCHUP,THGROUP_ATT_PITCHDOWN,oriDir.x,oriDirRate.x,
        apRefRot.x, apRotErr.x, (mode == 2? apRotXCtrlMode2 : apRotXCtrl),
        apRotThrust.x, apRotThrustE.x, apRotRateLim.x);
    }

    // Yaw Right/Left Control
    if ((mode!=2) ||
        ((mode==2) && (oriRVel > 30.0) && (abs(oriDir.y) > 0.15)) ||
        ((mode==2) && (oriRVel <= 30.0)) ) {                                           
      AutoPilotThrust(THGROUP_ATT_YAWRIGHT,THGROUP_ATT_YAWLEFT,oriDir.y,oriDirRate.y,
        apRefRot.y, apRotErr.y, (mode == 2? apRotYCtrlMode2 : apRotYCtrl),
        apRotThrust.y, apRotThrustE.y, apRotRateLim.y);
    }

    // Roll Right/Left Control
    if ((mode!=2) ||
        ((mode==2) && (oriRVel > 30.0) && (abs(oriDir.z) > 0.15)) ||
        ((mode==2) && (oriRVel <= 30.0)) ) {                                      
      AutoPilotThrust(THGROUP_ATT_BANKRIGHT,THGROUP_ATT_BANKLEFT,oriDir.z,oriDirRate.z,
        apRefRot.z, apRotErr.z, (mode == 2? apRotZCtrlMode2 : apRotZCtrl),
        apRotThrust.z, apRotThrustE.z, apRotRateLim.z);
    }
    apToggle = 5;


  } else if (apAttAllow && apToggle) {

    // 4. Run the ATT AP more frequently, once ROT is settled. 

    if (apRotArmed && (apToggle > 0)) apToggle--;
    apAttActive = true;

    // Right-Left Control
    AutoPilotThrust(THGROUP_ATT_RIGHT,THGROUP_ATT_LEFT,oriPos.x,oriPosRate.x,
      apRefAtt.x, apAttErr.x, (mode == 2? apAttXCtrlMode2 : apAttXCtrl),
      apAttThrust.x, apAttThrustE.x, apAttRateLim.x);

    // Up-Down Control
    AutoPilotThrust(THGROUP_ATT_UP,THGROUP_ATT_DOWN,oriPos.y,oriPosRate.y,
      apRefAtt.y, apAttErr.y, (mode == 2? apAttYCtrlMode2 : apAttYCtrl),
      apAttThrust.y, apAttThrustE.y, apAttRateLim.y);

    if (apAppArmed) {

      // 5. Run the APP AP alongside at ATT AP, if requested

      // For approach ... overshoot is not an option!
      // So figure out if we are inside a safe approach cone

      apAppActive = true;

      if ((mode == 0) && ((abs(oriDir.x) > maxDirErr) ||            // Are we outside the alignment cone in docking mode
          (abs(oriDir.y) > maxDirErr) ||
          (abs(oriDir.z) > maxDirErr) ||
          (abs(oriPos.x) > maxPosErr) ||
          (abs(oriPos.y) > maxPosErr))) {
        apAppConeOK = false;

        if (oriPos.z < 10.0) {
                                                                  // DANGER! Inside 10m, outside of approach cone. ==> Backout to 10m
          zTarget = 10.0;                                         
        } else {
                                                                  // NOTICE! Outside approach cone, 10+m away. ==> Just hold station.
          zTarget = 0.0;                                          // Pretend we are on station (bring rate to zero)
        }

      } else {
        apAppConeOK = true;
        zTarget = oriPos.z;                                         // On track ==> Proceed to dock.
      }

      if ((mode == 1) && (abs(oriPos.z)<8.0)) {                     // WP mode and we are there
        zTarget = 0.0;
        apWPreached = true;
      }

      AutoPilotThrust(THGROUP_ATT_FORWARD,THGROUP_ATT_BACK,zTarget,oriPosRate.z,
        apRefAtt.z, apAttErr.z, (mode == 2? apAttZCtrlMode2 : apAttZCtrl),
        apAttThrust.z, apAttThrustE.z, apAttRateLim.z);

    }

  }

  // Dump debug data on AP response if asked...
  if (dumpAP) {

    apD_err = fprintf_s(apD,"%.2f,%.4f,     %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,      %.3f,%i,%.3f,%.3f,%d,%d,%d,%d,%d,   %.3f,%.2f,%.2f,%.2f,%.2f,%d\n",
                      simT - dumpSimT, simStepAvg,
                      oriDir.x, oriDirRate.x, apRotRateLim.x, apRotThrust.x, apRotThrustE.x, apRefRot.x, apRotErr.x,
                      oriDir.y, oriDirRate.y, apRotRateLim.y, apRotThrust.y, apRotThrustE.y, apRefRot.y, apRotErr.y,
                      oriDir.z, oriDirRate.z, apRotRateLim.z, apRotThrust.z, apRotThrustE.z, apRefRot.z, apRotErr.z,
                      oriPos.x, oriPosRate.x, apAttRateLim.x, apAttThrust.x, apAttThrustE.x, apRefAtt.x, apAttErr.x,
                      oriPos.y, oriPosRate.y, apAttRateLim.y, apAttThrust.y, apAttThrustE.y, apRefAtt.y, apAttErr.y,
                      oriPos.z, oriPosRate.z, apAttRateLim.z, apAttThrust.z, apAttThrustE.z, apRefAtt.z, apAttErr.z,

                      zTarget, (apAppConeOK? 1 : 0), maxDirErr, maxPosErr,
                      apRotActive,apAttActive,apAppActive,apAttAllow,apToggle,
                      rtvOurRange, rtvOurLat, rtvWPlat[curWP], rtvOurLong, rtvWPlong[curWP], mode
                      
                      );

  }

  return;
}



void RVO_VCore::AutoPilotThrust(THGROUP_TYPE thrustPositive,THGROUP_TYPE thrustNegative, // Specific thrust calcs
      double ofs, double ofsRate,
      double &dVs, double &apErr,
      double rateCtrl[12][2],                                         // Rate Control Array
      double &thrust, double &thrustEst, double &rateLim) {

  // AutoPilotThrust ... implements control algorithm for rot, att and app thrusts 
  // General idea ... looking to trend the offset rate to a value controlled by the offset
  // We ramp the Hi and Lo checks down ofsIter times, dividing by 10 each time, to achieve fine control


  int    sign;

  sign = (ofs < 0.0? 1 : -1);
  rateLim = rateCtrl[11][1] * sign;   // Default to lowest rate

  // Find target and rate
  for (int i = 0; i<11; i++) {
    if (abs(ofs) > rateCtrl[i][0]) {
      rateLim = rateCtrl[i][1] * sign;
      if (abs(ofs)>100.0) {
        if (rateLim > (150.0 * dVs)) {
          rateLim = 150.0 * dVs;                   // Rate limits for realistic thrust ships - e.g. CEV Orion
        }
      } else if (abs(ofs)<10.0) {
        if (rateLim > (10.0 * dVs)) {
          rateLim = 10.0 * dVs;                    // Min maneuveing rate at low offsets
        }
      } else if (rateLim > ((150.0 * dVs) * (abs(ofs)/100.0))) {
        rateLim = (150.0 * dVs) * (abs(ofs)/100.0); // Taper from 150x dVs to 15x dVs as ofs comes from 100 down to 10
      }
      break;
    }
  }

  double noiseOfs = rateCtrl[11][0] * 0.50;       // Half the lowest offset limit
  double noiseRate = rateCtrl[11][1] * 0.80;   

  if (((rateLim < rateCtrl[11][1] * 1.01) && (abs(ofs) < noiseOfs) && (abs(ofsRate) < noiseRate))) { 
    thrust = 0.0;
    thrustEst = 0.0;
    return;                                     // Low enough that we don't need to do anything
  }

  if (ofs >= 0.0) {
    sign = (ofsRate > rateLim ? -1 : 1);
  } else {
    sign = (ofsRate < rateLim ? 1 : -1);
  }


  thrustEst = abs((rateLim - ofsRate) / dVs) / simStepAvg;   // Estimated cycles to achieve target

  double thrustMod = 30.0;
  if (dVs < 2.0) thrustMod = 30.0;

  if (thrustEst > thrustMod) {                         // More than thrustMod cycles away ...  go to full thrust
    thrust = 1.0;
  } else {                                       
    thrust = thrustEst / thrustMod;                    // Binary chop to the end
  }

  double quietFac = 1.00;
  if ((abs(oriPos.z)>80.0) || (mode==2)) quietFac = 2.50;
 
  if (abs(rateLim)>0.005) {
    double rateErr = ofsRate/rateLim;                 // Bias the thrusters to overcome repeated non-capture of the desired rate
    if (rateErr < -0.25) {
      apErr += 0.05;                                  // Aggregate bias relative to percentage velocity error
    } else if (rateErr < 0.0) {
      apErr += 0.04;
    } else if (rateErr < 0.10) {
      apErr += 0.03;
    } else if (rateErr < 0.50) {
      apErr += 0.02;
    } else if (rateErr < 0.90) {
      apErr += 0.01;
    } else {
      apErr = (0.95 * apErr) - 0.02;   // Cool off the bias
      if (apErr < 1.0) apErr = 1.0; 
    }
    if (apErr>4.00) apErr = 4.0;
    thrust *= apErr;
  }
  if ((abs(rateLim-ofsRate) < abs((mode==2?0.45:0.25)*rateLim))&&((rateLim*ofsRate)>0.0)) thrust = 0.0; // Within 25% of target rate and on same sign ... go quiet
  if ((abs(ofs) < 0.02*quietFac) && (abs(ofsRate) < 0.01*quietFac)) thrust = 0.0; // Close on target, rate low ... go quiet

  if (thrust>1.0) thrust=1.0;

  if (sign == -1) {               
    v->IncThrusterGroupLevel(thrustPositive, thrust);
  } else {                        
    v->IncThrusterGroupLevel(thrustNegative, thrust);
  }

  thrust *= sign;
}




void RVO_VCore::AutoPilotCalibrationExecute() {

  //
  // Auto Pilot Calibration Execution Sequence
  //  
  //
  static int apCalSteps = 12;
  static THGROUP_TYPE apCalThG[12] = {
      THGROUP_ATT_PITCHUP,
      THGROUP_ATT_YAWRIGHT,
      THGROUP_ATT_BANKRIGHT, 
      THGROUP_ATT_RIGHT,
      THGROUP_ATT_UP, 
      THGROUP_ATT_FORWARD,
      THGROUP_ATT_PITCHDOWN,
      THGROUP_ATT_YAWLEFT,
      THGROUP_ATT_BANKLEFT,
      THGROUP_ATT_LEFT,
      THGROUP_ATT_DOWN,
      THGROUP_ATT_BACK 
  };

  //
  // Three control loops here ... apCalPass controlling 4 passes (2 low weight, 2 high), apCalPhase controlling the 12 thruster directions
  // and apCalStep controlling 5 steps per phase (0 init, 1 wait, 2 thrust, 3 wait, 4 collect data)
  //

  if (apCalStep + apCalPhase == 0) {
    // set up for a new phase run


    if (apCalPass == 0) {
      // Init the thruster group descriptors once per calibration
      strcpy_s(apCalThGName[0],"Pitch Up");
      strcpy_s(apCalThGName[1],"Yaw Right");
      strcpy_s(apCalThGName[2],"Bank Right");
      strcpy_s(apCalThGName[3],"Right");
      strcpy_s(apCalThGName[4],"Up");
      strcpy_s(apCalThGName[5],"Forward");
      strcpy_s(apCalThGName[6],"Pitch Down");
      strcpy_s(apCalThGName[7],"Yaw Left");
      strcpy_s(apCalThGName[8],"Bank Left");
      strcpy_s(apCalThGName[9],"Left");
      strcpy_s(apCalThGName[10],"Down");
      strcpy_s(apCalThGName[11],"Back");
      strcpy_s(apCalThGName[12],"");

      // Set up the thruster firing sequence ... must always be in pairs, low first (e.g. 1-7, 0-6, 2-8, 3-9, 4-10, 5-11) 
      apCalCtrl[0] = 1;
      apCalCtrl[1] = 7;
      apCalCtrl[2] = 0;
      apCalCtrl[3] = 6;
      apCalCtrl[4] = 2;
      apCalCtrl[5] = 8; 
      apCalCtrl[6] = 3;
      apCalCtrl[7] = 9;
      apCalCtrl[8] = 4;
      apCalCtrl[9] = 10;
      apCalCtrl[10] = 5;
      apCalCtrl[11] = 11;

      
      apCalTTG = simT + 590.0;                            // Estimate Time To Go

      v->GetElements(apCalEl,apCalMjdRef);                // Save our original elements (as we will warp them in the calibration)

      // Capture starting propellant situation
      THRUSTER_HANDLE th = v->GetGroupThruster(THGROUP_ATT_FORWARD,0);  // Find the first thruster in group FORWARD
      apCalThPH = v->GetThrusterResource(th);                           // ASSUMPTION: all thrusters use same propellant source

      apCalPropCount = v->GetPropellantCount();                         // Number of propellant sources on the ship (e.g. Main, RCS, SCRAM)
      if (apCalPropCount > 10) apCalPropCount = 10;                     // Bound limits on array (who needs more than 10 prop sources anyway?)
      for (int i=0; i<apCalPropCount; i++) {
        PROPELLANT_HANDLE ph = v->GetPropellantHandleByIndex(i);        // Hook onto a tank
        apCalPropMass[i] = v->GetPropellantMass(ph);                    // Store original mass
      }

      for (int i=0;i<6;i++) {   // Empty out the master rates
        for (int j=0;j<2;j++) { 
          apCalMasterRate[i][j] = 0.0;
          apCalMasterRateSet[i][j] = false;
        }
      }
      apCalMasterMassSet[0] = false;
      apCalMasterMassSet[1] = false;
    }

    // Empty out the rates for this pass
    for (int i=0;i<12;i++) { 
      apCalRate[i] = 0.0;
      apCalRateSet[i] = false;
    }
    apCalMassFlag = (apCalPass %2 == 0 ? 0 : 1);

  }

  // Goal ... we will run 12 phases (0-11), running one thruster group at a time to determine response

  if (apCalStep==0) {
    // Step 0 ... initialize position and rates
    for (int i=0; i<12; i++) {
      v->SetThrusterGroupLevel(apCalThG[i], 0.0);       // Zero out all thrusters
    };
    v->SetThrusterGroupLevel(THGROUP_MAIN, 0.0);        // Clear out the main engines! Not useful when calibrating our thrusters!
    v->SetThrusterGroupLevel(THGROUP_RETRO, 0.0);
    v->SetThrusterGroupLevel(THGROUP_HOVER, 0.0);

    v->SetAngularVel(_V(0.0,0.0,0.0));                  // Stop all rotations
    
    ELEMENTS el;
    double mjd_ref;
    tv->GetElements(el,mjd_ref);                        // Pick up our target's elements
    el.L -= RAD * 0.0008347;                            // Go approx 100m behind target, same orbit
    v->SetElements(NULL, el, NULL, mjd_ref);            // Jump to tracking position

    for (int i=0; i<apCalPropCount; i++) {
      PROPELLANT_HANDLE ph = v->GetPropellantHandleByIndex(i);        // Hook onto a tank
      if (ph != apCalThPH) {
        v->SetPropellantMass(ph, v->GetPropellantMaxMass(ph) * (apCalPass %2 == 0? 0.0 : 1.0)); // Empty for evens 2, full for odds
      } else {
        v->SetPropellantMass(ph, v->GetPropellantMaxMass(ph) * (apCalPass %2 == 0? 0.1 : 1.0)); // 10% on the thruster tank evens, full odds
      }
    }

    apCalInit = true;
    apCalTime = simT;
    apCalStep++;
    sprintf_s(oapiDebugString(),128,"CALIBRATING - DO NOT TOUCH THRUSTERS. Phase %d, Step %d, Time Left: %.1f. Initializing", 
      apCalPhase, apCalStep, apCalTTG - simT); 

  } else {

    // Step 1 ... wait 2 sec
    // Step 2 ... thrust 8 sec
    // Step 3 ... wait 2 sec
    // Step 4 ... take results

    if (simT >= (apCalTime + (apCalStep == 2? 8.0 : 2.0))) { // Time passes for 2.0 secs for steps 1 and 3, or 8.0 secs for step 2

      int d = apCalCtrl[apCalPhase];

      apCalStepTime[apCalStep] = simT;                  // Time mark
      v->GetAngularVel(apCalStepAVel[apCalStep]);       // Angular Vel at mark
      apCalStepAVel[apCalStep] *= DEG;
      apCalStepLVel[apCalStep] = oriPosRate;            // RVel at mark
      apCalStepLVel[apCalStep] -= apCalStepLVel[1];     // RVel at mark relative to step 1
      if (apCalStep==1) {
        apCalMass = v->GetMass();                       // Pre-burn mass
      }

      if ((apCalStep==1)||(apCalStep==2)) {
        double th = (apCalStep == 1? 1.0 : 0.0);        // Thrust on for step 1, off for step 2
        v->SetThrusterGroupLevel(apCalThG[d], th);      // Toggle thruster group
      }

      apCalTime = simT;
      apCalStep++;

      if (apCalStep == 4) {
        // Completed phase ... pick up the calibration results
        apCalMass2 = v->GetMass();                      // Post-burn Mass

        // Update our master mass average: M = (M' * P + X) / (P+1), where M' is previous mass, P = phase (0-11), X = This Mass
        // And X = X' + (X' - X'')/2, where X' = starting mass, X'' ending mass 
        apCalMasterMass[apCalMassFlag] = (apCalMasterMass[apCalMassFlag] * apCalPhase + apCalMass2 + 0.5 * (apCalMass - apCalMass2))/(apCalPhase+1);
        apCalMasterMassSet[apCalMassFlag] = true;

        AutoPilotCalibrationCalcRates(d,apCalPass,
          apCalStepAVel[1],apCalStepAVel[2],apCalStepAVel[3],
          apCalStepLVel[1],apCalStepLVel[2],apCalStepLVel[3],
          apCalStepTime[1],apCalStepTime[2],apCalStepTime[3]);
        // Dump the calibrations for debug
        for (int i=1; i<4; i++) {
          apCalStepTimePhase[apCalPass][apCalPhase][i] = apCalStepTime[i];
          apCalStepAVelPhase[apCalPass][apCalPhase][i] = apCalStepAVel[i];
          apCalStepLVelPhase[apCalPass][apCalPhase][i] = apCalStepLVel[i];
        }
        for (int i=0; i<12; i++) {
          apCalRatePhase[apCalPass][apCalPhase][i] = apCalRate[i];
        }


        apCalStep = 0;
        apCalPhase++;
        if (apCalPhase == apCalSteps) {
          //
          // Completed pass
          //
          if (apCalPass < 3) {
            apCalPhase = 0;                                  // Finished pass ... reset for the next (4 passes total)
            apCalStep = 0;
            apCalPass++;
            return;
          }

          //
          // Completed all calibration phases ... do tidy-up
          //
          apRefRatesLoaded = true;
          apApcArmed = 0;
          mode = apcMode;

          sprintf_s(oapiDebugString(),128,"");
          apCalComplete = true;
          for (int i=0; i<12; i++) {
           v->SetThrusterGroupLevel(apCalThG[i], 0.0);       // Zero out all thrusters
          };
          v->SetAngularVel(_V(0.0,0.0,0.0));                  // Stop all rotations
          v->SetElements(NULL, apCalEl, NULL, apCalMjdRef);   // Restore old position

          for (int i=0; i<apCalPropCount; i++) {
            PROPELLANT_HANDLE ph = v->GetPropellantHandleByIndex(i);   // Hook onto a tank
            v->SetPropellantMass(ph, apCalPropMass[i]);        // Reset pre-test fuel weights
          }

          // Harmonize linear rates if within 3%
          for (int i=0; i<2; i++) {
           double av = (apCalMasterRate[3][i]+apCalMasterRate[4][i]+apCalMasterRate[5][i])/3.0;
           double av2pc = 0.03*av;
           if ( (abs(apCalMasterRate[3][i]-av)<av2pc) && (abs(apCalMasterRate[3][i]-av)<av2pc) && (abs(apCalMasterRate[3][i]-av)<av2pc) ) {
             apCalMasterRate[3][i] = av;
             apCalMasterRate[4][i] = av;
             apCalMasterRate[5][i] = av;
           }
          }

          // Append our new data line to the RV_Orientation_APRates
          AutoPilotRateStore();

          // Calculate the Master Reference set from our rates and weights
          CalculateApRefMaster(apCalMasterRate,apCalMasterMass);

          // Calculate our current reference rates for this weight
          CalculateApRef();

          FILE* of;
          fopen_s(&of, ".\\Config\\MFD\\RVO\\RVO_Calibration_Dump.csv","w");
          for (int p=0; p<4; p++) {
            fprintf(of,"PASS %d\n", p);
            for (int i=0; i<12;i++) {
              fprintf(of,"%s\n", apCalThGName[apCalCtrl[i]]);
              fprintf(of,"T, P,, Ax, Ay, Az,, Lx, Ly, Lz,,R0,R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R11\n");
              for (int j=1;j<4;j++) {
                fprintf(of,"%.3f,%d,,",apCalStepTimePhase[p][i][j],j);
                fprintf(of,"%.10f,%.10f,%.10f,,",apCalStepAVelPhase[p][i][j].x,apCalStepAVelPhase[p][i][j].y,apCalStepAVelPhase[p][i][j].z);
                fprintf(of,"%.3f,%.3f,%.3f,",apCalStepLVelPhase[p][i][j].x,apCalStepLVelPhase[p][i][j].y,apCalStepLVelPhase[p][i][j].z);
                for (int k=0;k<12;k++) {
                  fprintf(of,",%.3f",apCalRatePhase[p][i][k]);
                }
                fprintf(of,"\n");
              }
            }
          }
          fclose(of);

        }
      } else {
        VECTOR3 tmpAV;
        v->GetAngularVel(tmpAV);
        tmpAV *= DEG;

        // Completed step ... update display text
        switch (apCalStep) {
        case 1:
          sprintf_s(oapiDebugString(),256,"P %.2f Y %.2f R %.2f   X %.2f Y %.2f Z %.2f   CALIBRATING - DO NOT TOUCH THRUSTERS. Phase %d, Step %d, Time Left: %.1f. Pre %s", 
            tmpAV.x, tmpAV.y, tmpAV.z, oriPosRate.x, oriPosRate.y, oriPosRate.z, apCalPhase, apCalStep, apCalTTG - simT,apCalThGName[d]); 
          break;
        case 2:
        case 3:
          sprintf_s(oapiDebugString(),256,"P %.2f Y %.2f R %.2f   X %.2f Y %.2f Z %.2f   CALIBRATING - DO NOT TOUCH THRUSTERS. Phase %d, Step %d, Time Left: %.1f. Thrust tests on %s", 
            tmpAV.x, tmpAV.y, tmpAV.z, oriPosRate.x, oriPosRate.y, oriPosRate.z, apCalPhase, apCalStep, apCalTTG - simT, apCalThGName[d]); 
          break;
        }
      }
    }
  }
  return;
}

void RVO_VCore::AutoPilotCalibrationCalcRates(int apCalDir, int apCalPass,
  VECTOR3 av1, VECTOR3 av2, VECTOR3 av3,
  VECTOR3 lv1, VECTOR3 lv2, VECTOR3 lv3,
  double t1, double t2, double t3) {



  // Master switch to calculate a calibration rate

  switch (apCalDir) {
  case 0:                                                 // Pitch Up (+Ax)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      mul(rPortOri,_V(1,0,0)) * av1,
      mul(rPortOri,_V(1,0,0)) * av2,
      mul(rPortOri,_V(1,0,0)) * av3,
      t1, t2, t3);
    break;
  case 1:                                                 // Yaw Right (-Ay)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      mul(rPortOri,_V(0,-1,0)) * av1,
      mul(rPortOri,_V(0,-1,0)) * av2,
      mul(rPortOri,_V(0,-1,0)) * av3,
      t1, t2, t3);
    break;
  case 2:                                                 // Bank Right (+Az)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      mul(rPortOri,_V(0,0,1)) * av1,
      mul(rPortOri,_V(0,0,1)) * av2,
      mul(rPortOri,_V(0,0,1)) * av3,
      t1, t2, t3);
    break;
  case 3:                                                 // Right (-Lx)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      lv1.x, lv2.x, lv3.x, t1, t2, t3);
    break;
  case 4:                                                 // Up (-Ly)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      lv1.y, lv2.y, lv3.y, t1, t2,t3);
    break;
  case 5:                                                 // Fwd (-Lz)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      lv1.z, lv2.z, lv3.z, t1, t2, t3);
    break;
  case 6:                                                 // Pitch Down (-Ax)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      mul(rPortOri,_V(-1,0,0)) * av1,
      mul(rPortOri,_V(-1,0,0)) * av2,
      mul(rPortOri,_V(-1,0,0)) * av3,
      t1, t2, t3);
    break;
  case 7:                                                 // Yaw Left (+Ay)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      mul(rPortOri,_V(0,1,0)) * av1,
      mul(rPortOri,_V(0,1,0)) * av2,
      mul(rPortOri,_V(0,1,0)) * av3,
      t1, t2, t3);
    break;
  case 8:                                                 // Bank Left (-Az)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      mul(rPortOri,_V(0,0,-1)) * av1,
      mul(rPortOri,_V(0,0,-1)) * av2,
      mul(rPortOri,_V(0,0,-1)) * av3,
      t1, t2, t3);
    break;
  case 9:                                                 // Left (+Lx)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      lv1.x, lv2.x, lv3.x, t1, t2, t3);
    break;
  case 10:                                                // Down (+Ly)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      lv1.y, lv2.y, lv3.y, t1, t2, t3);
    break;
  case 11:                                                // Back (+Lz)
    AutoPilotCalibrationCalcOneRate(apCalDir,
      lv1.z, lv2.z, lv3.z, t1, t2, t3);
    break;
  case 12:                                                 // Null
    break;
  }

  // Calc Master Rate blending 2 passes, 2 directions per each of low and high weights
  //
  int n = (apCalPass < 2? 0 : 2);                         // n is the number of the run for this direction-PAIR (0-3)
  if (apCalDir>5) n++;
  int r = apCalDir % 6;                                   // r is the master rate direction PAIR
  int w = (apCalPass%2 == 0 ? 0 : 1);                     // w is the weight flag: 0 low, 1 high
  apCalMasterRate[r][w] = (apCalMasterRate[r][w]*n + apCalRate[apCalDir]) / (n+1);
  apCalMasterRateSet[r][w] = true; 

  return;
}

void RVO_VCore::AutoPilotCalibrationCalcOneRate(int apCalDir, 
  double v1, double v2, double v3,
  double t1, double t2, double t3) {

  // Calculate an individual calibration rate
  apCalRateSet[apCalDir] = true;
  apCalRate[apCalDir] = abs((v3 - v1) / (t2 - t1));
                                                          // Between T1 and T2 we are thrusting, and we shake out any remaining dV in the quiet
                                                          // period T2 to T3. Thrust acceleration is dV between T1 to T3, over T1 to T2
  return;
}

void RVO_VCore::CalculateApRefMaster(double apRateSet[6][2], double apMassSet[2]) {
  // Loads apRefMaster coefficients
  //
  // Method ...
  // We have two sets of rates, representing deg/sec and m/sec acceleration rates for low and high weight.
  // There is a direct power correlation between them, of the form Y=AX^B. E.g. Right thrust = A times Mass ^ B
  // Taking natural logs, we get LOG(Y) = LOG(A) + B * LOG(X), which is a straight line now.
  // For the slope ... B = ( LOG(Y1) - LOG(Y0) ) / ( LOG(X1) - LOG(X0) )
  // For the intercept at LOG(X) = 0 ... LOG(A) = LOG(Y0) - B * LOG(X1) ... and trivially A = EXP(LOG(A))
  //
  double X0 = apMassSet[0];
  double X1 = apMassSet[1];

  for (int i=0; i<6; i++) {
    double Y0 = apRateSet[i][0];
    double Y1 = apRateSet[i][1];
    double B = (log(Y1) - log(Y0))/(log(X1) - log(X0));
    double LOGA = log(Y0) - (B * log(X1));
    apRefMaster[i][0] = LOGA; 
    apRefMaster[i][1] = B; 
  }
  return;
}

void RVO_VCore::CalculateApRef() {
  // Loads apRefMass, apRefRot and apRefAtt via the apRefMaster
  //
  // Method...
  // We are calculating the reference rate set from apRefMaster, for our current weight
  // The equation is Y = AX^B ... i.e. LOG(Y) = LOG(A) + B * LOG(X) ... i.e. Y = EXP(LOG(A) + B * LOG(X)). 
  // We have LOG(A) and B in the apRefMaster, so this won't take long...
  //
  apRefMass = v->GetMass();
  double LOGX = log(apRefMass);
  for (int i=0; i<3; i++) apRefRot.data[i] = exp(apRefMaster[i][0] + (apRefMaster[i][1] * LOGX));
  for (int i=3; i<6; i++) apRefAtt.data[i-3] = exp(apRefMaster[i][0] + (apRefMaster[i][1] * LOGX));
  return;
}


bool RVO_VCore::AutoPilotRateFetch() {
  // Fetch calibration data for this ship class, if found
  FILE* rf;
  char clName[128];
  char buf[256];
  char *tok;
  char *bp;
  bool goodFetch = false;
  double params[14];
  int i;

  strcpy_s(clName,128,v->GetClassName());

  fopen_s(&rf, ".\\Config\\MFD\\RVO\\RVO_Rates.cfg","r");

  while (fgets(buf,255,rf)!=NULL) {
    bp = buf;
    if (!ParseWhiteSpace(&bp,&tok)) continue;   // Skip leading tabs and any comments
    if (!ParseQuotedString(&bp,&tok)) continue; // Bad parse
    if (_stricmp(tok,clName)!=0) continue;      // Not our ship
    for (i=0; i<14;i++) {
      if (!ParseDouble(&bp,&(params[i]))) break; // Bad parse
    }
    if (i<14) continue;                         // Bad parse

    apCalMasterMass[0] = params[0];
    for (int p=0; p<6; p++) {
      apCalMasterRate[p][0] = params[p+1];
    }
    apCalMasterMass[1] = params[7];
    for (int p=0; p<6; p++) {
      apCalMasterRate[p][1] = params[p+8];
    }
    goodFetch = true;                           // Found and successfully loaded a ship rate file (note - keep going in case user has calibrated more)
  }

  fclose(rf);
  return goodFetch;
}

void RVO_VCore::AutoPilotRateStore() {
  // Store calibration data for this ship class
  FILE* rf;
  char clName[128];

  fopen_s(&rf, ".\\Config\\MFD\\RVO\\RVO_Rates.cfg","a+");
  strcpy_s(clName,128,v->GetClassName());
  fprintf(rf,"\"%s\"\t", clName);
  for (int i=0; i<2; i++) {
    fprintf(rf,"\t%.3f", apCalMasterMass[i]);
    for (int j=0; j<6; j++) {
      fprintf(rf,"\t%.6f", apCalMasterRate[j][i]);
    }
  }
  fprintf(rf,"\n");
  fclose(rf);
  return;
}



void RVO_VCore::CreatePortRotMatrix(MATRIX3 &OriMx, VECTOR3 &OriDir, VECTOR3 &OriRot) {
 // Calculates OriMx to transform ship coord system to port coord system
  //
  // This code creates a rotation matrix from ship coordinates to dock coordinates.
  // The ship is standard left-handed Orbiter orientation (+x is right, +y is up, +z is forward).
  // We need to create a new coordinate system where +x is dock right, +y is dock up (alignment vector), and +z is dock out.
  // Finally, we need to express this in a Port Orientation rotation matrix, such that you can do mul(rPortOri, SHIPCOORDS) to make DOCKCOORDS. 
  //

  // dDir is our dock direction in ship coordinates. We want to transform it to {0, 0, 1} in dock coordinates.
  // Start with taking out the pitch. Use atan2 as it understands all 4 quadrants and is safe on the cardinal points.
  // Angle of pitch is the arctan of the y (up) over the z (forward), and the pitch rotation matrix is sround the x axis (see e.g. Wikipedia on Rotation Matrix).


  VECTOR3 lDir = OriDir;

  if (lDir.y != 0) {
    if (lDir.z != 0) {
      aPitch = atan2(lDir.y, lDir.z);      // Pitch angle is the up (Y) over the Forward (Z). 
      if (aPitch < -PI/2.0) {
        aPitch += PI;                      // 3th quadrant ... make +ve angle to the back
      } else if (aPitch > PI/2.0) {
        aPitch -= PI;                      // 2nd quadrant ... make -ve angle to the back
      }
      rPitch = _M(1,0,0,  0,cos(aPitch),-sin(aPitch),  0,sin(aPitch),cos(aPitch));
    } else {
      if (lDir.y>0) {
        rPitch = _M(1,0,0,  0,0,-1,  0,1,0);    // Clean 90 deg pitch down
      } else {
        rPitch = _M(1,0,0,  0,0,1,  0,-1,0);    // Clean 90 deg pitch up
      }
    }
  } else {
    rPitch = _M(1,0,0, 0,1,0, 0,0,1);           // No pitch correction needed. Note - if dDir.z is negative, treat that as a 180 deg yaw. 
  }
  lDir = mul(rPitch, lDir);                     // Apply the pitch ... check y = 0

  // Ditto for the yaw angle and roatation ... arctan of x (right) over z (forward). 
  if (lDir.x !=0) {
    if (lDir.z != 0) {
      aYaw = -atan2(lDir.x, lDir.z);
      rYaw = _M(cos(aYaw),0,sin(aYaw),  0,1,0,  -sin(aYaw),0,cos(aYaw));
    } else {
      if (lDir.x>0) {
        rYaw = _M(0,0,-1, 0,1,0, 1,0,0);      // Clean 90 deg yaw left
      } else {
        rYaw = _M(0,0,1, 0,1,0, -1,0,0);      // Clean 90 deg yaw right
      }
    }
  } else {
    if (lDir.z<0) {
      rYaw = _M(-1,0,0, 0,1,0, 0,0,-1);       // 180 yaw neded 
    } else {
      rYaw = _M(1,0,0, 0,1,0, 0,0,1);         // No yaw correction needed. 
    }
  }
  lDir = mul(rYaw, lDir);                     // Apply the yaw ... check x and y now 0 and z is fwd

  // The combination pitch then yaw rotation matrix (the port orientation matrix) is a matrix multiply of the two. 
  OriMx = mul(rYaw, rPitch);

  // For the roll, we need to consult the dock rotation orientation which is in dRot (queried directly from the dock parameters).
  // (By the way - x and y would now be zero, so you can't get the roll by atan2(dDir.y,dDir.x) etc, which is why the rotation is a different vector).
  // Start by applying the pitch and yaw rotation to the rotation vector, in a temporary dRot transformed variable. 
  VECTOR3 dRotTrans;
  dRotTrans = mul(OriMx,OriRot);
  
  // Angle of roll now falls out of the x and y dimensions on the transformed dRot, as does the roll to make it sit up at {0,1,0}
  VECTOR3 lRot = dRotTrans;
  if (lRot.x != 0) {
    if (lRot.y != 0) {
      aRoll = atan2(lRot.x, lRot.y);            // Roll angle is arctan right over up ... negative to roll it to y=1
      rRoll = _M(cos(aRoll),-sin(aRoll),0,  sin(aRoll),cos(aRoll),0,  0,0,1);
    } else {
      if (lRot.x >0) {
        rRoll = _M(0,-1,0,  1,0,0,  0,0,1);     // Clean roll 90 left
      } else {
        rRoll = _M(0,1,0,  -1,0,0,  0,0,1);     // Clean roll 90 right
      }
    }
  } else {
    if (lRot.y<0) {
      rRoll = _M(-1,0,0,0,-1,0,0,0,1);          // 180 degrees roll correction needed. 
    } else {
      rRoll = _M(1,0,0,0,1,0,0,0,1);            // No roll correction needed. 
    }
  }
  lRot = mul(rRoll, lRot);                      // Apply the roll ... check for {0,1,0}
 
  // Complete the port orientation matrix by multiplying in the roll data.
  OriMx = mul(rRoll, OriMx);

  // Post-clean the matrix of rounding noise
  for (int i=0; i<10; i++) {
    if (abs(OriMx.data[i])<1e-5) {
      OriMx.data[i] = 0.0;
    } else if (abs(OriMx.data[i]-1.0)<1e-5) { 
      OriMx.data[i] = 1.0;
    } else if (abs(OriMx.data[i]+1.0)<1e-5) { 
      OriMx.data[i] = -1.0;
    }
  }

  return;
}

void RVO_VCore::AllStopThrust() {
  // Shut down all thrusters
  v->SetThrusterGroupLevel(THGROUP_MAIN, 0.0);
  v->SetThrusterGroupLevel(THGROUP_RETRO, 0.0);
  v->SetThrusterGroupLevel(THGROUP_HOVER, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_PITCHUP, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_PITCHDOWN, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_YAWLEFT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_YAWRIGHT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_BANKLEFT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_BANKRIGHT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_RIGHT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_LEFT, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_UP, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_DOWN, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_FORWARD, 0.0);
  v->SetThrusterGroupLevel(THGROUP_ATT_BACK, 0.0);
  return;
}



bool RVO_VCore::GetFirstGuidanceRect() {

  // Initializes the waypoint box guidance rectangle stream

  guideRectMore = false;
  if ((!tv)||(!hDock)) return false; // No target vehicle or dock selected so far
  if ((mode == 3) || (mode == -1)) return false; // No guidance paths in APC or RVEL modes

  guideRectCount = 7;
  guideRectWP = 0;                    // Initialize guidance at WP 0 (target port) and work out
  guideRectWPbox = 0;                 // Initialize guidance at range 0 and work out
  guideRectMore = true;               // More to do!


  oapiCameraGlobalDir(&cgd);          // Get COP direction ... (WHY NO oapiCameraGlobalRot function?)
  oapiCameraGlobalPos(&cgp);          // Get COP position
  cgd = cgd + cgp;                    // Flip direction into a point offset from COP
  v->Global2Local(cgd, cld);          // Switch to ship coords
  v->Global2Local(cgp, clp);          // .. for direction and position
  cld = cld - clp;                    // Pull the position out of the direction point offset to make it a direction again
  for (int i=0; i<3; i++) {
    if (abs(cld.data[i])<1e-6) cld.data[i] = 0;
  }

  VECTOR3 cldp,clrp;                  // Camera local directon and rot relative to local port
  MATRIX3 cRotP;                      // Rotation matrix to take camera view onto port view
  cldp = mul(rPortOri,cld);           // Rotate camera direction into port coords. Now looking for a rotation to make cldp into {0,0,1} in port coords

  CreatePortRotMatrix(cRotP, cldp, _V(0,0,0)); // Temporary rotation matrix to rotate the local "up" direction

  VECTOR3 checkDir = mul(cRotP,cldp);   // Expect checkDir to be _V(0,0,1);
  clrp = tmul(cRotP,_V(0,1,0));         // clrp is "up" in port rotations

  clr = tmul(rPortOri,clrp);            // And finally we can find camera "up" in ship coords
  for (int i=0; i<3; i++) {
    if (abs(clr.data[i])<1e-6) clr.data[i] = 0;
  }
 
  CreatePortRotMatrix(cRot, cld, clr);  // Make a rotation matrix from ship to COP
//sprintf_s(oapiDebugString(),256,"Camera Rot Mx { {%5.2f %5.2f %5.2f}  {%5.2f %5.2f %5.2f}  {%5.2f %5.2f %5.2f} }   Camera Angles {P %5.2f Y %5.2f R %5.2f}", cRot.m11, cRot.m12, cRot.m13, cRot.m21, cRot.m22, cRot.m23, cRot.m31, cRot.m32, cRot.m33, aPitch*DEG, aYaw*DEG, aRoll*DEG ); 
  
  if (guideRectDump) {
      fprintf(gD,"Guidance Situation\n");
      fprintf(gD,"Our Pos {%.3f,%.3f,%.3f}, Our Lat-Long {%.3f,%.3f}, Our Port Range {%.3f}, Our WP Range {%.3f}\n", rtvOurPos.x,rtvOurPos.y,rtvOurPos.z,rtvOurLat,rtvOurLong,rtvOurRange,rtvOurWPrange);
      fprintf(gD,"Our COP Pos {%.3f,%.3f,%.3f}, Our COP Dir {%.3f,%.3f,%.3f}, Our COP Rot {%.3f,%.3f,%.3f}\n", clp.x,clp.y,clp.z,cld.x,cld.y,cld.z,clr.x,clr.y,clr.z);

      for (int i=0; i<curWP; i++) {
        double j = 0.0;
        fprintf(gD,"Waypoint %d, Pos {%.3f,%.3f,%.3f}, Out-Dir {%.3f,%.3f,%.3f}, Up-Dir {%.3f,%.3f,%.3f}, Lat-Long {%.3f,%.3f}, Dist to next WP {%.3f}\n", i, rtvWPpos[i].x,rtvWPpos[i].y,rtvWPpos[i].z,
          rtvWPdir[i].x,rtvWPdir[i].y,rtvWPdir[i].z,rtvWProt[i].x,rtvWProt[i].y,rtvWProt[i].z,rtvWPlat[i],rtvWPlong[i],rtvWPdist[i]);

        while (j<=rtvWPdist[i]) {
          VECTOR3 WPp, TPp,TSp, GSp, LSp, COPp;

          WPp = _V(0,0,j);
          TPp = tmul(rtvWPoriL[i],WPp);
          TPp += rtvWPpos[i];
          TSp = tmul(rtvPortOri,TPp);
          TSp += rtvdPos;
          tv->Local2Global(TSp,GSp);
          v->Global2Local(GSp,LSp);
          COPp = LSp - clp;
          COPp = mul(cRot, COPp);


          fprintf(gD,"Waypoint %d, Box Dist %.0f, PortC {%.3f,%.3f,%.3f}, TgtC {%.3f,%.3f,%.3f}, LclC {%.3f,%.3f,%.3f}, COPC {%.3f,%.3f,%.3f}\n", i, j,
            TPp.x,TPp.y,TPp.z,
            TSp.x,TSp.y,TSp.z,
            LSp.x,LSp.y,LSp.z,
            COPp.x,COPp.y,COPp.z            
            );
          if (abs(j-rtvWPdist[i])<0.1) break;
          j += 50.0;
          if (j>rtvWPdist[i]) j = rtvWPdist[i];

        }

      }
      fclose(gD);
      guideRectDump = false;
  }


  return GetNextGuidanceRect();

}

bool RVO_VCore::GetNextGuidanceRect() {
  double guideDist;
  double guideScale;

  // Finds the next waypoint box on the sequence
  while (guideRectMore) {
    // Create rectangles for this sector of the approach
    guideRectWPbox++;



    if (guideRectWP == 0) {
      if (guideRectWPbox<5) {
        guideRectColor = (guideRectWPbox == 1? 2 : 1);
        guideDist = 10.0 * guideRectWPbox;      // Intervals of 10m for last 50m
        guideScale = 0.1 + 0.9 * (guideRectWPbox / 5.0);
      } else {
        guideDist = 50.0 + 50.0 * (guideRectWPbox - 5); // ... then 50m from 50m out
        guideScale = 1.0;
        guideRectColor = 0;
      }
    } else {
      guideScale = 1.5 + 0.2 * guideRectWP;
      if (guideRectWPbox<6) {
        guideRectColor = (guideRectWPbox == 1? 2 : 1);
        guideDist = 4.0 + 16.0 * guideRectWPbox;      // Intervals of 16m from 20m to 84m
      } else {
        guideDist = 100.0 + 50.0 * (guideRectWPbox - 6); // ... then 50m from 100m out
        guideRectColor = 0;
      }
    }



    if (guideDist > rtvWPdist[guideRectWP]) {
      // Switch to next sector of the approach
      guideRectWP++;
      guideRectWPbox =0;
      guideRectMore = (guideRectWP <= curWP);                // End of the approach path when we exceed the current waypoint
      if (!guideRectMore) return guideRectMore;
      continue;
    }

    double VX = 7.0 * guideScale;
    double VY = 5.0 * guideScale;
    double VBY = 3.0 * guideScale;

    
    // Create rectangle at this distance in port coords and frame
    rGuideRectVertex[0] = _V(-VX,  VY,guideDist);
    rGuideRectVertex[1] = _V(-VX, -VY,guideDist);
    rGuideRectVertex[2] = _V( VX, -VY,guideDist);
    rGuideRectVertex[3] = _V( VX,  VY,guideDist);
    rGuideRectVertex[4] = _V(-VX,  VY,guideDist);
    rGuideRectVertex[5] = _V(-VX,  VBY,guideDist);
    rGuideRectVertex[6] = _V( VX,  VBY,guideDist);

    for (int i=0; i<guideRectCount; i++) {
      rGuideRectVertex[i] = tmul(rtvWPoriL[guideRectWP],rGuideRectVertex[i]); // Rotate us from WP frame to target port frame
      rGuideRectVertex[i] = rGuideRectVertex[i] + rtvWPpos[guideRectWP];      // Add in the WP location in port coords
      rGuideRectVertex[i] = tmul(rtvPortOri,rGuideRectVertex[i]);       // Rotate us back from target port frame to target ship frame
      rGuideRectVertex[i] = rGuideRectVertex[i] + rtvdPos;              // Add in the target port location
      tv->Local2Global(rGuideRectVertex[i],gGuideRectVertex[i]);        // Now make target into a global coord
      v->Global2Local(gGuideRectVertex[i],lGuideRectVertex[i]);         // And flip it back into a target in OUR local coordinate system
      lGuideRectVertex[i] = lGuideRectVertex[i] - clp;                  // Translate it onto our COP origin
      lGuideRectVertex[i] = mul(cRot, lGuideRectVertex[i]);             // Rotate around our COP direction to get into our local HUD view coordinate system
      if ((lGuideRectVertex[i].z<5.0)) {                                // Rectangle is close enough to the HUD to drop it out
        guideRectMore = false;
        break;
      }
    }
    if (guideRectMore) return guideRectMore;                             // We should draw this box, so we are done
    if (guideRectWP == curWP) {
        return guideRectMore;                                            // If we are on the same segment and we have gone behind us, that's the end 
    } else {
      guideRectMore = true;                                              // ... else skip this box and continue looking
    }
  }
  return guideRectMore;
}

double RVO_VCore::id( double d ) const {
  // Internalizes distances (if in US, converts to meters)
  return (units? d : d / 3.2808399);
}

double RVO_VCore::ed( double d ) const {
  // Externalizes distances (if in US, converts to feet)
  return (units? d : d * 3.2808399);
}