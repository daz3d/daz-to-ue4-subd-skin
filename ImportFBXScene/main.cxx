/****************************************************************************************

   Copyright (C) 2015 Autodesk, Inc.
   All rights reserved.

   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.

****************************************************************************************/

/////////////////////////////////////////////////////////////////////////
//
// This example illustrates how to detect if a scene is password 
// protected, import and browse the scene to access node and animation 
// information. It displays the content of the FBX file which name is 
// passed as program argument. You can try it with the various FBX files 
// output by the export examples.
//
/////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

#include "../Common/Common.h"
#include "DisplayCommon.h"
#include "DisplayHierarchy.h"
#include "DisplayAnimation.h"
#include "DisplayMarker.h"
#include "DisplaySkeleton.h"
#include "DisplayMesh.h"
#include "DisplayNurb.h"
#include "DisplayPatch.h"
#include "DisplayLodGroup.h"
#include "DisplayCamera.h"
#include "DisplayLight.h"
#include "DisplayGlobalSettings.h"
#include "DisplayPose.h"
#include "DisplayPivotsAndLimits.h"
#include "DisplayUserProperties.h"
#include "DisplayGenericInfo.h"

#include "SubdivideMesh.h"

using json = nlohmann::json;

// Local function prototypes.
void DisplayContent(FbxScene* pScene);
void DisplayContent(FbxNode* pNode);
void DisplayTarget(FbxNode* pNode);
void DisplayTransformPropagation(FbxNode* pNode);
void DisplayGeometricTransform(FbxNode* pNode);
void DisplayMetaData(FbxScene* pScene);

bool gVerbose = false;
json gDtuJson;

// This utility program reads DazToUnreal dtu file.
// Then, it loads the topology from the base mesh, interpolates skin weights, 
// and saves them to subdivided HD mesh.
int main(int argc, char** argv)
{
    FbxString dtuFilePath("");
    for (int i = 1, c = argc; i < c; ++i)
    {
        FbxString arg = FbxString(argv[i]);
        if (arg == "-verbose") gVerbose = true;
        else if (dtuFilePath.IsEmpty())
        {
            dtuFilePath = arg;
        }
    }

	if(dtuFilePath.IsEmpty() )
	{
        FBXSDK_printf("\n\nUsage: ImportScene [-verbose] <DTU file name>\n\n");
        return -1;
	}

    // read the dtu JSON file
    DisplayString("Loading .dtu JSON file: ", dtuFilePath);
    std::ifstream dtu(dtuFilePath);
    dtu >> gDtuJson;

    // get "FBX File" entry from json
    std::string fbxFile = gDtuJson["FBX File"];
    FbxString outFilePath(fbxFile.c_str());

    FbxString baseFilePath(outFilePath);
    baseFilePath.FindAndReplace(".fbx", "_base.fbx");

    FbxString hdFilePath(outFilePath);
    hdFilePath.FindAndReplace(".fbx", "_HD.fbx");

    FbxManager* lSdkManager = NULL;
    FbxScene* baseMeshScene = NULL;
    bool lResult;

    // Prepare the FBX SDK and load base mesh scene
    InitializeSdkObjects(lSdkManager, baseMeshScene);

    DisplayString("Loading base mesh FBX file: ", baseFilePath);
    lResult = LoadScene(lSdkManager, baseMeshScene, baseFilePath.Buffer(), gVerbose);

    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while loading the scene...");
    }
    else 
    {
        // Display the scene.
        if (gVerbose)
        {
            DisplayMetaData(baseMeshScene);

            FBXSDK_printf("\n\n---------------------\nGlobal Light Settings\n---------------------\n\n");
            DisplayGlobalLightSettings(&baseMeshScene->GetGlobalSettings());

            FBXSDK_printf("\n\n----------------------\nGlobal Camera Settings\n----------------------\n\n");
            DisplayGlobalCameraSettings(&baseMeshScene->GetGlobalSettings());

            FBXSDK_printf("\n\n--------------------\nGlobal Time Settings\n--------------------\n\n");
            DisplayGlobalTimeSettings(&baseMeshScene->GetGlobalSettings());

            FBXSDK_printf("\n\n---------\nHierarchy\n---------\n\n");
            DisplayHierarchy(baseMeshScene);

            FBXSDK_printf("\n\n------------\nNode Content\n------------\n\n");
            DisplayContent(baseMeshScene);

            FBXSDK_printf("\n\n----\nPose\n----\n\n");
            DisplayPose(baseMeshScene);

            FBXSDK_printf("\n\n---------\nAnimation\n---------\n\n");
            DisplayAnimation(baseMeshScene);

            //now display generic information
            FBXSDK_printf("\n\n---------\nGeneric Information\n---------\n\n");
            DisplayGenericInfo(baseMeshScene);
        }
    }

    // subdivide
    lResult = ProcessScene(baseMeshScene);

    // load HD mesh scene
    DisplayString("Loading HD mesh FBX file:   ", hdFilePath);
    FbxScene* hdMeshScene = FbxScene::Create(lSdkManager, "HD Mesh Scene");
    lResult = LoadScene(lSdkManager, hdMeshScene, hdFilePath.Buffer(), gVerbose);
    if (lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while loading the scene...");
    }

    // save clusters to the scene object
    lResult = SaveClustersToScene(hdMeshScene);

    DisplayString("Saving the output mesh FBX file:  ", outFilePath);
    int fileFormat = lSdkManager->GetIOPluginRegistry()->GetNativeWriterFormat(); // binary file format
    lResult = SaveScene(lSdkManager, hdMeshScene, outFilePath, fileFormat);
    if (lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while saving the scene...");
    }

    // Destroy all objects created by the FBX SDK.
    DestroySdkObjects(lSdkManager, lResult);

    return 0;
}

void DisplayContent(FbxScene* pScene)
{
    int i;
    FbxNode* lNode = pScene->GetRootNode();

    if(lNode)
    {
        for(i = 0; i < lNode->GetChildCount(); i++)
        {
            DisplayContent(lNode->GetChild(i));
        }
    }
}

void DisplayContent(FbxNode* pNode)
{
    FbxNodeAttribute::EType lAttributeType;
    int i;

    if(pNode->GetNodeAttribute() == NULL)
    {
        FBXSDK_printf("NULL Node Attribute\n\n");
    }
    else
    {
        lAttributeType = (pNode->GetNodeAttribute()->GetAttributeType());

        switch (lAttributeType)
        {
	    default:
	        break;
        case FbxNodeAttribute::eMarker:  
            DisplayMarker(pNode);
            break;

        case FbxNodeAttribute::eSkeleton:  
            DisplaySkeleton(pNode);
            break;

        case FbxNodeAttribute::eMesh:      
            DisplayMesh(pNode);
            break;

        case FbxNodeAttribute::eNurbs:      
            DisplayNurb(pNode);
            break;

        case FbxNodeAttribute::ePatch:     
            DisplayPatch(pNode);
            break;

        case FbxNodeAttribute::eCamera:    
            DisplayCamera(pNode);
            break;

        case FbxNodeAttribute::eLight:     
            DisplayLight(pNode);
            break;

        case FbxNodeAttribute::eLODGroup:
            DisplayLodGroup(pNode);
            break;
        }   
    }

    DisplayUserProperties(pNode);
    DisplayTarget(pNode);
    DisplayPivotsAndLimits(pNode);
    DisplayTransformPropagation(pNode);
    DisplayGeometricTransform(pNode);

    for(i = 0; i < pNode->GetChildCount(); i++)
    {
        DisplayContent(pNode->GetChild(i));
    }
}


void DisplayTarget(FbxNode* pNode)
{
    if(pNode->GetTarget() != NULL)
    {
        DisplayString("    Target Name: ", (char *) pNode->GetTarget()->GetName());
    }
}

void DisplayTransformPropagation(FbxNode* pNode)
{
    FBXSDK_printf("    Transformation Propagation\n");

    // 
    // Rotation Space
    //
    EFbxRotationOrder lRotationOrder;
    pNode->GetRotationOrder(FbxNode::eSourcePivot, lRotationOrder);

    FBXSDK_printf("        Rotation Space: ");

    switch (lRotationOrder)
    {
    case eEulerXYZ: 
        FBXSDK_printf("Euler XYZ\n");
        break;
    case eEulerXZY:
        FBXSDK_printf("Euler XZY\n");
        break;
    case eEulerYZX:
        FBXSDK_printf("Euler YZX\n");
        break;
    case eEulerYXZ:
        FBXSDK_printf("Euler YXZ\n");
        break;
    case eEulerZXY:
        FBXSDK_printf("Euler ZXY\n");
        break;
    case eEulerZYX:
        FBXSDK_printf("Euler ZYX\n");
        break;
    case eSphericXYZ:
        FBXSDK_printf("Spheric XYZ\n");
        break;
    }

    //
    // Use the Rotation space only for the limits
    // (keep using eEulerXYZ for the rest)
    //
    FBXSDK_printf("        Use the Rotation Space for Limit specification only: %s\n",
        pNode->GetUseRotationSpaceForLimitOnly(FbxNode::eSourcePivot) ? "Yes" : "No");


    //
    // Inherit Type
    //
    FbxTransform::EInheritType lInheritType;
    pNode->GetTransformationInheritType(lInheritType);

    FBXSDK_printf("        Transformation Inheritance: ");

    switch (lInheritType)
    {
    case FbxTransform::eInheritRrSs:
        FBXSDK_printf("RrSs\n");
        break;
    case FbxTransform::eInheritRSrs:
        FBXSDK_printf("RSrs\n");
        break;
    case FbxTransform::eInheritRrs:
        FBXSDK_printf("Rrs\n");
        break;
    }
}

void DisplayGeometricTransform(FbxNode* pNode)
{
    FbxVector4 lTmpVector;

    FBXSDK_printf("    Geometric Transformations\n");

    //
    // Translation
    //
    lTmpVector = pNode->GetGeometricTranslation(FbxNode::eSourcePivot);
    FBXSDK_printf("        Translation: %f %f %f\n", lTmpVector[0], lTmpVector[1], lTmpVector[2]);

    //
    // Rotation
    //
    lTmpVector = pNode->GetGeometricRotation(FbxNode::eSourcePivot);
    FBXSDK_printf("        Rotation:    %f %f %f\n", lTmpVector[0], lTmpVector[1], lTmpVector[2]);

    //
    // Scaling
    //
    lTmpVector = pNode->GetGeometricScaling(FbxNode::eSourcePivot);
    FBXSDK_printf("        Scaling:     %f %f %f\n", lTmpVector[0], lTmpVector[1], lTmpVector[2]);
}


void DisplayMetaData(FbxScene* pScene)
{
    FbxDocumentInfo* sceneInfo = pScene->GetSceneInfo();
    if (sceneInfo)
    {
        FBXSDK_printf("\n\n--------------------\nMeta-Data\n--------------------\n\n");
        FBXSDK_printf("    Title: %s\n", sceneInfo->mTitle.Buffer());
        FBXSDK_printf("    Subject: %s\n", sceneInfo->mSubject.Buffer());
        FBXSDK_printf("    Author: %s\n", sceneInfo->mAuthor.Buffer());
        FBXSDK_printf("    Keywords: %s\n", sceneInfo->mKeywords.Buffer());
        FBXSDK_printf("    Revision: %s\n", sceneInfo->mRevision.Buffer());
        FBXSDK_printf("    Comment: %s\n", sceneInfo->mComment.Buffer());

        FbxThumbnail* thumbnail = sceneInfo->GetSceneThumbnail();
        if (thumbnail)
        {
            FBXSDK_printf("    Thumbnail:\n");

            switch (thumbnail->GetDataFormat())
            {
            case FbxThumbnail::eRGB_24:
                FBXSDK_printf("        Format: RGB\n");
                break;
            case FbxThumbnail::eRGBA_32:
                FBXSDK_printf("        Format: RGBA\n");
                break;
            }

            switch (thumbnail->GetSize())
            {
	        default:
	            break;
            case FbxThumbnail::eNotSet:
                FBXSDK_printf("        Size: no dimensions specified (%ld bytes)\n", thumbnail->GetSizeInBytes());
                break;
            case FbxThumbnail::e64x64:
                FBXSDK_printf("        Size: 64 x 64 pixels (%ld bytes)\n", thumbnail->GetSizeInBytes());
                break;
            case FbxThumbnail::e128x128:
                FBXSDK_printf("        Size: 128 x 128 pixels (%ld bytes)\n", thumbnail->GetSizeInBytes());
            }
        }
    }
}

