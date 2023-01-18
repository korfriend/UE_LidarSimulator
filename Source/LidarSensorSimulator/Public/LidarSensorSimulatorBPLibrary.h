// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"

#include "Engine/Engine.h"
#include "Components/StaticMeshComponent.h"

#include "LidarPointCloud.h"

#include "LidarSensorSimulatorBPLibrary.generated.h"

// Our delegate to return our value
DECLARE_DYNAMIC_DELEGATE_OneParam(FAsyncDelegate, FString, StringOut);
/* 
*	Function library class.
*	Each function in it is expected to be static and represents blueprint node that can be called in any blueprint.
*
*	When declaring function you can define metadata for the node. Key function specifiers will be BlueprintPure and BlueprintCallable.
*	BlueprintPure - means the function does not affect the owning object in any way and thus creates a node without Exec pins.
*	BlueprintCallable - makes a function which can be executed in Blueprints - Thus it has Exec pins.
*	DisplayName - full name of the node, shown when you mouse over the node and in the blueprint drop down menu.
*				Its lets you name the node using characters not allowed in C++ function names.
*	CompactNodeTitle - the word(s) that appear on the node.
*	Keywords -	the list of keywords that helps you to find node when you search for it using Blueprint drop-down menu. 
*				Good example is "Print String" node which you can find also by using keyword "log".
*	Category -	the category your node will be under in the Blueprint drop-down menu.
*
*	For more info on custom blueprint nodes visit documentation:
*	https://wiki.unrealengine.com/Custom_Blueprint_Node_Creation
*/
UCLASS()
class ULidarSensorSimulatorBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_UCLASS_BODY()

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Execute Sample function", Keywords = "LidarSensorSimulator sample test testing"), Category = "LidarSensorSimulatorTesting")
	static float LidarSensorSimulatorSampleFunction(float Param);

	UFUNCTION(BlueprintCallable, Category = "LidarSensorSimulatorTesting", meta = (DisplayName = "LidarScan1", Keywords = "LidarSensorSimulator Async Scan"))
		static void LidarSensorAsyncScan1(
			const TArray<FLidarPointCloudPoint>& lidarPoints, UStaticMeshComponent* sensorMesh,
			TArray<FLidarPointCloudPoint>& lidarPointsOut, TArray<float>& depthArrayOut, FAsyncDelegate Out,
			const bool asyncScan = true, const int lidarChannels = 1, const float fovDegree = 360.f, const int lidarResolution = 1, const float lidarRange = 1000.f, const float channelInterval = 1.f
		);

	UFUNCTION(BlueprintCallable, Category = "LidarSensorSimulatorTesting", meta = (DisplayName = "LidarScan2", Keywords = "LidarSensorSimulator Async Scan"))
		static void LidarSensorAsyncScan2(
			const TArray<FLidarPointCloudPoint>& lidarPoints, UStaticMeshComponent* sensorMesh,
			TArray<FLidarPointCloudPoint>& lidarPointsOut, TArray<float>& depthArrayOut, FAsyncDelegate Out,
			const bool asyncScan = true,
			const float vFovSDeg = -15.f, const float vFovEDeg = 15.f, const int lidarChannels = 8, const float hfovDeg = 360.f, const int lidarResolution = 1, const float lidarRange = 1000.f);
};
