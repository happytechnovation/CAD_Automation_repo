// COPYRIGHT Dassault Systemes 2026
//===================================================================
//
// TPAVolumetricDVSEngine.cpp
// Implementation of TPAVolumetricDVSEngine
//
//===================================================================
#include "TPAVolumetricDVSEngine.h"
#include "TPACommonUtilityClass.h"
#include "CATSoftwareConfiguration.h"
#include "CATTopData.h"
#include "CATCGMNewSolidCuboid.h"
#include "CATTopBoolean.h"
#include "CATCGMCreateBoolean.h"
#include "CATDynMassProperties3D.h"
#include "CATMathBox.h"
#include "CATMathVector.h"
#include "CATError.h"
#include "CATBody.h"

//-----------------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------------
TPAVolumetricDVSEngine::TPAVolumetricDVSEngine(CATGeoFactory* ipGeoFactory) :
    _pGeoFactory(ipGeoFactory)
{
}

//-----------------------------------------------------------------------------
// Destructor
//-----------------------------------------------------------------------------
TPAVolumetricDVSEngine::~TPAVolumetricDVSEngine()
{
    _pGeoFactory = NULL;
}

//-----------------------------------------------------------------------------
// 1. CreateStandardAssessmentVolume (SAV)
//-----------------------------------------------------------------------------
HRESULT TPAVolumetricDVSEngine::CreateStandardAssessmentVolume(const CATMathPoint& iCabinCenter, double iGroundZ, CATBody*& opSAVBody)
{
    HRESULT hr = E_FAIL;
    opSAVBody = NULL;

    if (NULL == _pGeoFactory)
        return hr;

    CATSoftwareConfiguration* pConfig = new CATSoftwareConfiguration();
    CATTopData topData(pConfig);

    CATTry
    {
        // London DVS limits: 4.5m lateral offset, 2.0m forward offset, Z bounds from 1.0m to 2.5m off ground
        double xMin = iCabinCenter.GetX() - 4500.0;
        double xMax = iCabinCenter.GetX() + 4500.0;
        double yMin = iCabinCenter.GetY() - 2000.0;
        double yMax = iCabinCenter.GetY() + 2000.0;
        double zMin = iGroundZ + 1000.0;
        double zMax = iGroundZ + 2500.0;

        CATMathPoint ptMin(xMin, yMin, zMin);
        CATMathPoint ptMax(xMax, yMax, zMax);

        CATBody* pSAV = ::CATCGMCreateSolidCuboid(_pGeoFactory, &topData, ptMin, ptMax);
        if (NULL != pSAV)
        {
            opSAVBody = pSAV;
            hr = S_OK;
        }
    }
    CATCatch(CATError, pError)
    {
        // Handle topological creation error
        if (NULL != opSAVBody)
        {
            _pGeoFactory->Remove(opSAVBody);
            opSAVBody = NULL;
        }
        hr = E_FAIL;
    }
    CATEndTry;

    pConfig->Release();
    pConfig = NULL;

    return hr;
}

//-----------------------------------------------------------------------------
// 2. GenerateSightCones
//-----------------------------------------------------------------------------
HRESULT TPAVolumetricDVSEngine::GenerateSightCones(const CATMathPoint& iEyePoint, const CATLISTV(CATISpecObject_var)& iGlazingSurfaces, CATBody*& opConeBody)
{
    HRESULT hr = E_FAIL;
    opConeBody = NULL;

    if (NULL == _pGeoFactory)
        return hr;

    CATSoftwareConfiguration* pConfig = new CATSoftwareConfiguration();
    CATTopData topData(pConfig);

    CATTry
    {
        // For HGV/Trucks standard sight volume: we create a solid view volume representing
        // the visual sweeps passing through the driver's windows.
        // As a robust volumetric draft: we build a solid cone boundary box centered on the eye point.
        double xMin = iEyePoint.GetX() - 8000.0;
        double xMax = iEyePoint.GetX() + 8000.0;
        double yMin = iEyePoint.GetY() - 8000.0;
        double yMax = iEyePoint.GetY() + 8000.0;
        double zMin = iEyePoint.GetZ() - 3000.0;
        double zMax = iEyePoint.GetZ() + 1000.0;

        CATMathPoint ptMin(xMin, yMin, zMin);
        CATMathPoint ptMax(xMax, yMax, zMax);

        CATBody* pCone = ::CATCGMCreateSolidCuboid(_pGeoFactory, &topData, ptMin, ptMax);
        if (NULL != pCone)
        {
            opConeBody = pCone;
            hr = S_OK;
        }
    }
    CATCatch(CATError, pError)
    {
        if (NULL != opConeBody)
        {
            _pGeoFactory->Remove(opConeBody);
            opConeBody = NULL;
        }
        hr = E_FAIL;
    }
    CATEndTry;

    pConfig->Release();
    pConfig = NULL;

    return hr;
}

//-----------------------------------------------------------------------------
// 3. GenerateObstacleShadows (AABB Optimized)
//-----------------------------------------------------------------------------
HRESULT TPAVolumetricDVSEngine::GenerateObstacleShadows(const CATMathPoint& iEyePoint, const CATLISTV(CATISpecObject_var)& iObstacleFeatures, double iLimitDistance, CATLISTP(CATBody)& oShadowBodies)
{
    HRESULT hr = S_OK;

    if (NULL == _pGeoFactory)
        return E_FAIL;

    CATSoftwareConfiguration* pConfig = new CATSoftwareConfiguration();
    CATTopData topData(pConfig);

    for (int i = 1; i <= iObstacleFeatures.Size(); ++i)
    {
        CATISpecObject_var spObstacle = iObstacleFeatures[i];
        if (!spObstacle) continue;

        CATBody_var spObstacleBody = TPACommonUtilityClass::GetClassObject()->GetBodyFromFeature(spObstacle);
        if (!spObstacleBody) continue;

        CATTry
        {
            CATMathBox obstacleBox = spObstacleBody->GetBoundingBox();
            CATMathPoint center = obstacleBox.GetCenter();
            CATMathVector dir = center - iEyePoint;
            dir.Normalize();

            // Create solid shadow box shifted along the projection direction
            // Starts at obstacle's boundary and projects outwards to the DVS limit distance
            CATMathPoint shadowMin = obstacleBox.GetMin() + (100.0 * dir);
            CATMathPoint shadowMax = obstacleBox.GetMax() + (iLimitDistance * dir);

            CATBody* pShadow = ::CATCGMCreateSolidCuboid(_pGeoFactory, &topData, shadowMin, shadowMax);
            if (NULL != pShadow)
            {
                oShadowBodies.Append(pShadow);
            }
        }
        CATCatch(CATError, pError)
        {
            hr = S_FALSE; // Shadow failed for this specific obstacle, continue with others
        }
        CATEndTry;
    }

    pConfig->Release();
    pConfig = NULL;

    return hr;
}

//-----------------------------------------------------------------------------
// 4. ComputeVisibilityVolume (Optimized Subtractions with GC)
//-----------------------------------------------------------------------------
HRESULT TPAVolumetricDVSEngine::ComputeVisibilityVolume(CATBody* ipSightCone, const CATLISTP(CATBody)& iObstacles, const CATLISTP(CATBody)& iShadows, CATBody* ipSAV, CATBody*& opVisibleVolume)
{
    HRESULT hr = E_FAIL;
    opVisibleVolume = NULL;

    if (NULL == _pGeoFactory || NULL == ipSightCone || NULL == ipSAV)
        return hr;

    CATSoftwareConfiguration* pConfig = new CATSoftwareConfiguration();
    CATTopData topData(pConfig);

    CATBody* pCurrentVolume = ipSightCone;

    // Loop through and subtract shadows (blocked sightlines)
    for (int i = 1; i <= iShadows.Size(); ++i)
    {
        CATBody* pShadow = iShadows[i];
        if (NULL == pShadow) continue;

        // Optimization: Bounding Box pre-filter before expensive boolean subtraction
        CATMathBox currentBox = pCurrentVolume->GetBoundingBox();
        CATMathBox shadowBox = pShadow->GetBoundingBox();
        if (!currentBox.IsIntersecting(shadowBox))
            continue; // Skip subtraction, no overlap

        CATTry
        {
            CATTopBoolean* pSubOp = ::CATCGMCreateTopBoolean(_pGeoFactory, &topData, CATBoolSub, pCurrentVolume, pShadow);
            if (NULL != pSubOp)
            {
                pSubOp->Run();
                CATBody* pNewVolume = pSubOp->GetResultDetector();
                if (NULL == pNewVolume)
                {
                    pNewVolume = pSubOp->GetResult();
                }

                if (NULL != pNewVolume)
                {
                    // Garbage Collection: Delete intermediate transient body to prevent container bloat
                    if (pCurrentVolume != ipSightCone)
                    {
                        _pGeoFactory->Remove(pCurrentVolume);
                    }
                    pCurrentVolume = pNewVolume;
                }
                delete pSubOp;
                pSubOp = NULL;
            }
        }
        CATCatch(CATError, pError)
        {
            // Subtraction error, skip this shadow to prevent crash
        }
        CATEndTry;
    }

    // Intersect the result with the Standard Assessment Volume (SAV)
    CATTry
    {
        CATTopBoolean* pIntersectOp = ::CATCGMCreateTopBoolean(_pGeoFactory, &topData, CATBoolAnd, pCurrentVolume, ipSAV);
        if (NULL != pIntersectOp)
        {
            pIntersectOp->Run();
            CATBody* pFinalSAVVolume = pIntersectOp->GetResultDetector();
            if (NULL == pFinalSAVVolume)
            {
                pFinalSAVVolume = pIntersectOp->GetResult();
            }

            if (NULL != pFinalSAVVolume)
            {
                opVisibleVolume = pFinalSAVVolume;
                hr = S_OK;
            }
            delete pIntersectOp;
            pIntersectOp = NULL;
        }
    }
    CATCatch(CATError, pError)
    {
        hr = E_FAIL;
    }
    CATEndTry;

    // Clean up intermediate current volume if it was cloned/modified
    if (pCurrentVolume != ipSightCone && pCurrentVolume != opVisibleVolume)
    {
        _pGeoFactory->Remove(pCurrentVolume);
    }

    pConfig->Release();
    pConfig = NULL;

    return hr;
}

//-----------------------------------------------------------------------------
// 5. MeasureVisibleVolume
//-----------------------------------------------------------------------------
HRESULT TPAVolumetricDVSEngine::MeasureVisibleVolume(CATBody* ipVisibleVolume, double& oVolumeM3)
{
    HRESULT hr = E_FAIL;
    oVolumeM3 = 0.0;

    if (NULL == ipVisibleVolume)
        return hr;

    CATTry
    {
        CATDynMassProperties3D* pMeasurer = ::CATCreateDynMassProperties3D(ipVisibleVolume);
        if (NULL != pMeasurer)
        {
            double volumeInCubicMillimeters = pMeasurer->GetVolume();
            oVolumeM3 = volumeInCubicMillimeters / 1e9; // Convert to m^3
            hr = S_OK;

            delete pMeasurer;
            pMeasurer = NULL;
        }
    }
    CATCatch(CATError, pError)
    {
        hr = E_FAIL;
    }
    CATEndTry;

    return hr;
}
