// COPYRIGHT Dassault Systemes 2026
//===================================================================
//
// TPAVolumetricDVSEngine.h
// Header definition of TPAVolumetricDVSEngine
//
//===================================================================
#ifndef TPAVolumetricDVSEngine_H
#define TPAVolumetricDVSEngine_H

#include "TPACommonUtilityModule.h"
#include "CATBaseUnknown.h"
#include "CATMathPoint.h"
#include "CATBody.h"
#include "CATLISTV_CATISpecObject.h"
#include "CATGeoFactory.h"
#include "ListPOfCATBody.h"

/**
 * Class representing the core Volumetric Compliance Engine for DVS.
 * 
 * Role: Provides modular, performance-optimized solid-modeling operations
 * (SAV construction, sight cones generation, shadow projections, boolean cuts, 
 * and mass volume measurements) for Heavy Goods Vehicles.
 */
class ExportedByTPACommonUtilityModule TPAVolumetricDVSEngine
{
public:
    TPAVolumetricDVSEngine(CATGeoFactory* ipGeoFactory);
    ~TPAVolumetricDVSEngine();

    // 1. SAV Construction: Create a solid body representing the Standard Assessment Volume
    HRESULT CreateStandardAssessmentVolume(const CATMathPoint& iCabinCenter, double iGroundZ, int iStandard, CATBody*& opSAVBody);

    // 2. Cone Generation: Generate unobstructed sight cones starting from the eye points passing through the window boundaries
    HRESULT GenerateSightCones(const CATMathPoint& iEyePoint, const CATLISTV(CATISpecObject_var)& iGlazingSurfaces, CATBody*& opConeBody);

    // 3. Shadow Casting: Project the silhouette of each obstacle away from the eyes to generate solid shadow volumes
    HRESULT GenerateObstacleShadows(const CATMathPoint& iEyePoint, const CATLISTV(CATISpecObject_var)& iObstacleFeatures, double iLimitDistance, CATLISTP(CATBody)& oShadowBodies);

    // 4. Boolean Cut: Subtract the obstacle and shadow solids from the sight cones, then intersect the remaining volume with the SAV
    HRESULT ComputeVisibilityVolume(CATBody* ipSightCone, const CATLISTP(CATBody)& iObstacles, const CATLISTP(CATBody)& iShadows, CATBody* ipSAV, CATBody*& opVisibleVolume);

    // 5. Measurement: Use CATDynMassProperties3D to calculate the final visible volume in cubic meters
    HRESULT MeasureVisibleVolume(CATBody* ipVisibleVolume, double& oVolumeM3);

    // 6. Dynamic Eye Point Generation: Calculate left, mid, right eye points based on AHP, Seat Mid Point, and Length Offset
    HRESULT GenerateEyePointsFromAHP(const CATMathPoint& iAHP, const CATMathPoint& iSeatMidPoint, double iLengthOffset, const CATMathVector& iForwardDir, const CATMathVector& iUpDir, CATMathPoint& oLeftEye, CATMathPoint& oMidEye, CATMathPoint& oRightEye);

private:
    CATGeoFactory* _pGeoFactory;
};

#endif
