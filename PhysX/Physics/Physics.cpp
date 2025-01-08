#include "Physics.h"


void Physics::InitSystem()
{
	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mDefaultAllocatorCallback, mDefaultErrorCallback);

	mPvd = PxCreatePvd(*mFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	mPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, mPvd);
	mDispatcher = PxDefaultCpuDispatcherCreate(2);

	mScene = this->CreateScene(PxVec3(0.0f, -9.81f, 0.0f));

	PxPvdSceneClient* pvdClient = mScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
}

PxScene* Physics::CreateScene(const PxVec3& _gravity)
{
	PxSceneDesc sceneDescription(mPhysics->getTolerancesScale());
	sceneDescription.gravity = _gravity;
	sceneDescription.cpuDispatcher = mDispatcher;
	//sceneDescription.filterShader = PxDefaultSimulationFilterShader;
	sceneDescription.filterShader = contactReportFilterShader;
	sceneDescription.simulationEventCallback = &mContactReportCallback;
	mScene = mPhysics->createScene(sceneDescription);

	return mScene;
}

PxMaterial* Physics::CreateMaterial(const PxF32& _staticFriction, const PxF32& _dynamicFriction, const PxF32& _resitution)
{
	return mPhysics->createMaterial(_staticFriction, _dynamicFriction, _resitution);
}

PxRigidStatic* Physics::CreatePlane(const PxPlane& _plane, PxMaterial& _material)
{
	return PxCreatePlane(*mPhysics, _plane, _material);
}

PxShape* Physics::CreateBox(const PxTransform& _transform, const PxReal& _halfExtent, const PxMaterial& _material)
{
	PxShape* shape = mPhysics->createShape(PxBoxGeometry(_halfExtent, _halfExtent, _halfExtent), _material);
	PxRigidDynamic* body = mPhysics->createRigidDynamic(_transform);
	body->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
	AddObject(*body);
	return shape;
}

void Physics::CreateStack(const PxTransform& t, PxU32 size, PxReal halfExtent, const PxMaterial& _material)
{
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			CreateBox(t.transform(localTm), halfExtent, _material);
		}
	}
}

PxShape* Physics::CreateSphere(const PxTransform& _transform, const PxMaterial& _material)
{
	// todo: check this....
	PxShape* shape = mPhysics->createShape(PxSphereGeometry(), _material);
	PxRigidDynamic* body = mPhysics->createRigidDynamic(_transform);
	body->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
	AddObject(*body);

	return shape;
}

PxRigidDynamic* Physics::CreateDynamicSphere(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity, PxMaterial& _material)
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*mPhysics, t, geometry, _material, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);

	AddObject(*dynamic);
	return dynamic;

}

void Physics::AddObject(PxActor& shape)
{
	mScene->addActor(shape);
}

Physics::Physics()
{
	InitSystem();
	CreateScene(PxVec3(0.0f, -9.81f, 0.0f));

	PxMaterial* floorMaterial = CreateMaterial(0.5f, 0.5f, 0.6f);
	auto plane = CreatePlane(PxPlane(0.0f, 1.0f, 0.0f, 0.0f), *floorMaterial);

	AddObject(*plane);
}

void Physics::run()
{
	mContactReportCallback.gContactPositions.clear();
	mContactReportCallback.gContactImpulses.clear();

	mScene->simulate(1.0f / 60.0f);
	mScene->fetchResults(true);

	printf("%d contact reports\n", PxU32(mContactReportCallback.gContactPositions.size()));
}

PxFilterFlags Physics::contactReportFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0, PxFilterObjectAttributes attributes1, PxFilterData filterData1, PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

	// all initial and persisting reports for everything, with per-point data
	pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT
		| PxPairFlag::eNOTIFY_TOUCH_FOUND
		| PxPairFlag::eNOTIFY_TOUCH_PERSISTS
		| PxPairFlag::eNOTIFY_CONTACT_POINTS;
	return PxFilterFlag::eDEFAULT;
}
