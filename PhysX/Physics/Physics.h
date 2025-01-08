#pragma once
#include <memory>
#include <PxPhysicsAPI.h>
#include <vector>

using namespace physx;


class Physics
{
public:
	Physics();
	PxMaterial*		CreateMaterial(const PxF32& _staticFriction, const PxF32& _dynamicFriction, const PxF32& _resitution);
	PxRigidStatic*	CreatePlane(const PxPlane& _plane, PxMaterial& _material);
	PxShape*		CreateBox(const PxTransform& _transform, const PxReal& _halfExtent, const PxMaterial& _material);
	void			CreateStack(const PxTransform& t, PxU32 size, PxReal halfExtent, const PxMaterial& _material);
	
	PxShape*		CreateSphere(const PxTransform& _transform, const PxMaterial& _material);
	PxRigidDynamic* CreateDynamicSphere(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity, PxMaterial& _material);
	void			AddObject(PxActor& shape);
	
	void run();

private:
	static PxFilterFlags contactReportFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
		PxFilterObjectAttributes attributes1, PxFilterData filterData1,
		PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize);
	
	class ContactReportCallback : public PxSimulationEventCallback
	{
	public:

		std::vector<PxVec3> gContactPositions;
		std::vector<PxVec3> gContactImpulses;

		void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count)			{ PX_UNUSED(constraints); PX_UNUSED(count); }
		void onWake(PxActor** actors, PxU32 count)									{ PX_UNUSED(actors); PX_UNUSED(count); }
		void onSleep(PxActor** actors, PxU32 count)									{ PX_UNUSED(actors); PX_UNUSED(count); }
		void onTrigger(PxTriggerPair* pairs, PxU32 count)							{ PX_UNUSED(pairs); PX_UNUSED(count); }
		void onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32)	{}
		void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
		{
			PX_UNUSED((pairHeader));
			std::vector<PxContactPairPoint> contactPoints;

			for (PxU32 i = 0; i < nbPairs; i++)
			{
				PxU32 contactCount = pairs[i].contactCount;
				if (contactCount)
				{
					contactPoints.resize(contactCount);
					pairs[i].extractContacts(&contactPoints[0], contactCount);

					for (PxU32 j = 0; j < contactCount; j++)
					{
						gContactPositions.push_back(contactPoints[j].position);
						gContactImpulses.push_back(contactPoints[j].impulse);
					}
				}
			}
		}
	};
	ContactReportCallback mContactReportCallback;

	void InitSystem();

	// used once
	PxScene*		CreateScene(const PxVec3& _gravity);	
	

	// setup
	physx::PxDefaultAllocator		mDefaultAllocatorCallback;
	physx::PxDefaultErrorCallback	mDefaultErrorCallback;
	physx::PxTolerancesScale		mToleranceScale;
	physx::PxDefaultCpuDispatcher*  mDispatcher = NULL;
	physx::PxFoundation*			mFoundation = NULL;
	physx::PxPvd*					mPvd = NULL;
	physx::PxPvdTransport*			mTransport = NULL;
	PxCudaContextManager*			gCudaContextManager = NULL;


	// Usables
	physx::PxPhysics*				mPhysics = NULL;
	physx::PxScene*					mScene = NULL;

};
