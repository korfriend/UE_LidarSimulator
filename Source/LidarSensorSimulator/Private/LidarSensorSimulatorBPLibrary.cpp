// Copyright Epic Games, Inc. All Rights Reserved.

#include "LidarSensorSimulatorBPLibrary.h"
#include "LidarSensorSimulator.h"

#include "Async/Async.h"
#include "Misc/ScopeLock.h"
//#include <mutex>

ULidarSensorSimulatorBPLibrary::ULidarSensorSimulatorBPLibrary(const FObjectInitializer& ObjectInitializer)
: Super(ObjectInitializer)
{

}

float ULidarSensorSimulatorBPLibrary::LidarSensorSimulatorSampleFunction(float Param)
{
	return -1;
}

static FCriticalSection critical;
static bool isCompleted = true;

auto LidarScan1 = [](const TArray<FLidarPointCloudPoint>& lidarPoints, UStaticMeshComponent* sensorMesh,
	TArray<FLidarPointCloudPoint>& lidarPointsOut, TArray<float>& depthArrayOut,
	FAsyncDelegate Out, const int lidarChannels, const float fovDegree, const int lidarResolution, const float lidarRange, const float channelInterval)
{
	UWorld* currentWorld = sensorMesh->GetWorld(); 

	//FWorldContext* world = GEngine->GetWorldContextFromGameViewport(GEngine->GameViewport);
	//currentWorld = world->World();

	//UWorld* currentWorld = GEngine->GetWorld(); // note GEngine does not know what the current world is

	if (currentWorld == nullptr) {
		return;
	}

	lidarPointsOut = lidarPoints; // copy data

	FVector posLidarSensorWS = sensorMesh->GetComponentLocation();
	FRotator rotLidarSensor2World = sensorMesh->GetComponentRotation();
	//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, FString::SanitizeFloat(lidarRange));
	//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, posLidarSensorWS.ToString());

	const float deltaDeg = fovDegree / (float)lidarResolution;

	FCollisionObjectQueryParams targetObjTypes;
	targetObjTypes.AddObjectTypesToQuery(ECC_WorldStatic);
	targetObjTypes.AddObjectTypesToQuery(ECC_WorldDynamic);

	depthArrayOut.Init(-1.f, lidarChannels* lidarResolution);

	for (int chIdx = 0; chIdx < lidarChannels; chIdx++) {

		FVector posChannelStart = posLidarSensorWS + FVector(0, 0, 1.f) * channelInterval * (float)chIdx;

		for (int x = 0; x < lidarResolution; x++) {
			float rotDeg = deltaDeg * (float)x;
			FVector vecRot = FVector(1.f, 0, 0).RotateAngleAxis(rotDeg, FVector(0, 0, 1));
			FVector dirVec = rotLidarSensor2World.RotateVector(vecRot);
			FVector posChannelEnd = posChannelStart + dirVec * lidarRange;

			// LineTrace
			FHitResult outHit;
			if (currentWorld->LineTraceSingleByObjectType(outHit, posChannelStart, posChannelEnd, targetObjTypes)) {
				FVector3f hitPosition = FVector3f(outHit.Location);
				FLidarPointCloudPoint pcPoint(hitPosition);
				//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, outHit.Location.ToString());
				lidarPointsOut.Add(pcPoint);

				depthArrayOut[x + chIdx * lidarResolution] = FVector::Dist(posChannelStart, FVector(hitPosition));
			}
		}
	}

	GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Red, FString::FromInt(lidarPointsOut.Num()));

	//FScopeLock lock(&critical);
	//pointCloud->SetData(lidarPointsOut);
	//lock.Unlock();

	AsyncTask(ENamedThreads::GameThread, [Out]()
		{
			//if (Out != nullptr) {
				// We execute the delegate along with the param
				Out.ExecuteIfBound(FString("Finished"));
			//}
			isCompleted = true;

			//FScopeLock lock(&critical);
			//pointCloud->SetData(lidarPointsOut);
		}
	);
};

void ULidarSensorSimulatorBPLibrary::LidarSensorAsyncScan1(
	const TArray<FLidarPointCloudPoint>& lidarPoints, UStaticMeshComponent* sensorMesh,
	TArray<FLidarPointCloudPoint>& lidarPointsOut, TArray<float>& depthArrayOut,
	FAsyncDelegate Out, const bool asyncScan, const int lidarChannels,
	const float fovDegree, const int lidarResolution, const float lidarRange, const float channelInterval
)
{
	if (!isCompleted) {
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("Wait..."));
		return;
	}
	isCompleted = false;

	if (asyncScan) {
		// Schedule a thread
		// Pass in our parameters to the lambda expression
		// note that those parameters from the main thread can be released out during this async process
		// pointer parameters are safe because they are conservative
		AsyncTask(ENamedThreads::GameThread, [&lidarPoints, sensorMesh, &lidarPointsOut, &depthArrayOut, Out, asyncScan, lidarChannels, fovDegree, lidarResolution, lidarRange, channelInterval]() { // AnyHiPriThreadNormalTask
			LidarScan1(lidarPoints, sensorMesh, lidarPointsOut, depthArrayOut, Out,
				lidarChannels, fovDegree, lidarResolution, lidarRange, channelInterval);
			});
	}
	else {
		LidarScan1(lidarPoints, sensorMesh, lidarPointsOut, depthArrayOut, Out,
			lidarChannels, fovDegree, lidarResolution, lidarRange, channelInterval);
	}
}


auto LidarScan2 = [](const TArray<FLidarPointCloudPoint>& lidarPoints, UStaticMeshComponent* sensorMesh,
	TArray<FLidarPointCloudPoint>& lidarPointsOut, TArray<float>& depthArrayOut,
	FAsyncDelegate Out,
	const float vFovSDeg, const float vFovEDeg, const int lidarChannels, const float hfovDeg, const int lidarResolution, const float lidarRange)
{
	UWorld* currentWorld = sensorMesh->GetWorld();

	if (currentWorld == nullptr) {
		return;
	}

	lidarPointsOut = lidarPoints; // copy data

	FVector posLidarSensorWS = sensorMesh->GetComponentLocation();
	FRotator rotLidarSensor2World = sensorMesh->GetComponentRotation();

	const float deltaDeg = hfovDeg / (float)lidarResolution;

	FCollisionObjectQueryParams targetObjTypes;
	targetObjTypes.AddObjectTypesToQuery(ECC_WorldStatic);
	targetObjTypes.AddObjectTypesToQuery(ECC_WorldDynamic);
	targetObjTypes.AddObjectTypesToQuery(ECC_PhysicsBody);
	targetObjTypes.AddObjectTypesToQuery(ECC_Vehicle);
	targetObjTypes.AddObjectTypesToQuery(ECC_Destructible);

	depthArrayOut.Init(-1.f, lidarChannels * lidarResolution);

	const float vRotDelta = fabs(vFovEDeg - vFovSDeg) / std::max((lidarChannels - 1), 1);
	for (int chIdx = 0; chIdx < lidarChannels; chIdx++) {

		float vRotDeg = vFovSDeg + (float)chIdx * vRotDelta;
		FVector dirStart = FVector(1.f, 0, 0).RotateAngleAxis(vRotDeg, FVector(0, -1, 0));
		FVector posChannelStart = posLidarSensorWS;// +FVector(0, 0, 1.f) * channelInterval * (float)chIdx;

		for (int x = 0; x < lidarResolution; x++) {
			float rotDeg = deltaDeg * (float)x;
			FVector vecRot = dirStart.RotateAngleAxis(rotDeg, FVector(0, 0, 1));
			FVector dirVec = rotLidarSensor2World.RotateVector(vecRot);
			FVector posChannelEnd = posChannelStart + dirVec * lidarRange;

			// LineTrace
			FHitResult outHit;
			if (currentWorld->LineTraceSingleByObjectType(outHit, posChannelStart, posChannelEnd, targetObjTypes)) {
				FVector3f hitPosition = FVector3f(outHit.Location);
				FLidarPointCloudPoint pcPoint(hitPosition);
				//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, outHit.Location.ToString());
				lidarPointsOut.Add(pcPoint);

				depthArrayOut[x + chIdx * lidarResolution] = FVector::Dist(posChannelStart, FVector(hitPosition));
			}
		}
	}

	GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Red, FString::FromInt(lidarPointsOut.Num()));

	//FScopeLock lock(&critical);
	//pointCloud->SetData(lidarPointsOut);
	//lock.Unlock();

	AsyncTask(ENamedThreads::GameThread, [Out]()
		{
			//if (Out != nullptr) {
				// We execute the delegate along with the param
			Out.ExecuteIfBound(FString("Finished"));
			//}
			isCompleted = true;

			//FScopeLock lock(&critical);
			//pointCloud->SetData(lidarPointsOut);
		}
	);
};

void ULidarSensorSimulatorBPLibrary::LidarSensorAsyncScan2(
	const TArray<FLidarPointCloudPoint>& lidarPoints, UStaticMeshComponent* sensorMesh,
	TArray<FLidarPointCloudPoint>& lidarPointsOut, TArray<float>& depthArrayOut, FAsyncDelegate Out,
	const bool asyncScan,
	const float vFovSDeg, const float vFovEDeg, const int lidarChannels, const float hfovDeg, const int lidarResolution, const float lidarRange
)
{
	if (!isCompleted) {
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("Wait..."));
		return;
	}
	isCompleted = false;

	if (asyncScan) {
		// Schedule a thread
		// Pass in our parameters to the lambda expression
		// note that those parameters from the main thread can be released out during this async process
		// pointer parameters are safe because they are conservative
		AsyncTask(ENamedThreads::GameThread, [&lidarPoints, sensorMesh, &lidarPointsOut, &depthArrayOut, Out, asyncScan, vFovSDeg, vFovEDeg, lidarChannels, hfovDeg, lidarResolution, lidarRange]() { // AnyHiPriThreadNormalTask
			LidarScan2(lidarPoints, sensorMesh, lidarPointsOut, depthArrayOut, Out,
				vFovSDeg, vFovEDeg, lidarChannels, hfovDeg, lidarResolution, lidarRange);
			});
	}
	else {
		LidarScan2(lidarPoints, sensorMesh, lidarPointsOut, depthArrayOut, Out,
			vFovSDeg, vFovEDeg, lidarChannels, hfovDeg, lidarResolution, lidarRange);
	}
}
/**/