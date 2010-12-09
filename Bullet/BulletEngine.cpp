#include <Bullet/BulletEngine.h>

#include <Bullet/DebugDrawer.h>
#include <Bullet/BulletDebugNode.h>
#include <Bullet/Util.h>

#include <Geometry/AABB.h>
#include <Geometry/Sphere.h>
#include <Geometry/TriangleMesh.h>
#include <Geometry/CompoundShape.h>
#include <Geometry/HeightfieldTerrainShape.h>
#include <Geometry/TriangleIterator.h>
#include <Geometry/BoundingSphere.h>
#include <Core/Exceptions.h>
#include <Logging/Logger.h>
#include <Physics/RigidBody.h>
#include <Physics/DynamicBody.h>
#include <Physics/CarConfig.h>
#include <Physics/IRayResultCallback.h>
#include <Physics/Car.h>
#include <Bullet/BulletRayResultCallback.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <btBulletDynamicsCommon.h>
#include <Resources/DataBlock.h>

// NOTE: USE_LIBSPE2 is PS3 SPU stuff
#ifdef BULLET_MULTITHREADED
    //Platform stuff
    #define uint64_t IS_ALREADY_DEFINED
    #include <BulletMultiThreaded/PlatformDefinitions.h>
    #undef uint64_t // Dont fuck up everything
    //Dispatcher
    #include <BulletMultiThreaded/SpuGatheringCollisionDispatcher.h>
    //Thread lib
    #ifdef USE_LIBSPE2
        #include <BulletMultiThreaded/SpuLibspe2Support.h>
    #else
        #if defined (_WIN32)
            #include <BulletMultiThreaded/Win32ThreadSupport.h>
        #elif defined (USE_PTHREADS)
            #include <BulletMultiThreaded/PosixThreadSupport.h>
        #endif //_WIN32 && USE_PTHREADS
        #include <BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h>
    #endif //USE_LIBSPE2
    //Solver
    #include <BulletMultiThreaded/btParallelConstraintSolver.h>
    #include <BulletMultiThreaded/SequentialThreadSupport.h>

    btThreadSupportInterface* createSolverThreadSupport(int maxNumThreads)
    {
        btThreadSupportInterface* threadSupport;
        //#define SEQUENTIAL
        #ifdef SEQUENTIAL
	        SequentialThreadSupport::SequentialThreadConstructionInfo tci(
                    "solverThreads",
                    SolverThreadFunc,
                    SolverlsMemoryFunc);
	        threadSupport = new SequentialThreadSupport(tci);
        #else //SEQUENTIAL
            #ifdef _WIN32
	            Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo(
                        "solverThreads",
                        SolverThreadFunc,
                        SolverlsMemoryFunc,
                        maxNumThreads);
	            threadSupport = new Win32ThreadSupport(threadConstructionInfo);
            #elif defined (USE_PTHREADS) //_WIN32
	            PosixThreadSupport::ThreadConstructionInfo solverConstructionInfo(
                        "solver",
                        SolverThreadFunc,
                        SolverlsMemoryFunc,
                        maxNumThreads);
	            threadSupport = new PosixThreadSupport(solverConstructionInfo);
            #else //_WIN32 && USE_PTHREADS
	            SequentialThreadSupport::SequentialThreadConstructionInfo tci(
                        "solverThreads",
                        SolverThreadFunc,
                        SolverlsMemoryFunc);
	            threadSupport = new SequentialThreadSupport(tci);
            #endif //_WIN32 && USE_PTHREADS
        #endif // SEQUENTIAL
        threadSupport->startSPU();
	    return threadSupport;
    }
#endif

using OpenEngine::Core::NotImplemented;
using namespace OpenEngine::Physics;
using namespace OpenEngine::Geometry;
using namespace OpenEngine::Logging;
using namespace boost;
using namespace std;

namespace OpenEngine 
{
    namespace Bullet
    {

        BulletEngine::BulletEngine(OpenEngine::Geometry::AABB & worldAabb, Vector<3,float> gravity) 
        {
            #ifdef BULLET_MULTITHREADED
	            m_threadSupportSolver = 0;
	            m_threadSupportCollision = 0;
                int maxNumOutstandingTasks = 8;
            #endif
            m_dispatcher=0;
            m_collisionConfiguration = new btDefaultCollisionConfiguration();

            #ifdef BULLET_MULTITHREADED
                #ifdef USE_WIN32_THREADING
                    m_threadSupportCollision = new Win32ThreadSupport(
                            Win32ThreadSupport::Win32ThreadConstructionInfo(
                                "collision",
                                processCollisionTask,
                                createCollisionLocalStoreMemory,
                                maxNumOutstandingTasks));
                #else //USE_WIN32_THREADING
                    #ifdef USE_LIBSPE2
                        spe_program_handle_t *program_handle;
                        #ifndef USE_CESOF
                            program_handle = spe_image_open("./spuCollision.elf");
                            if (program_handle == NULL)
                                logger.error << "SPU OPEN IMAGE ERROR" << logger.end;
                            else
                                logger.info << "IMAGE OPENED" << logger.end;
                        #else //USE_CESOF
                            extern spe_program_handle_t spu_program;
                            program_handle = &spu_program;
                        #endif //USE_CESOF
                    SpuLibspe2Support* threadSupportCollision = 
                        new SpuLibspe2Support(program_handle, maxNumOutstandingTasks);
                    #elif defined (USE_PTHREADS) //USE_LIBSPE2
                        PosixThreadSupport::ThreadConstructionInfo constructionInfo(
                                "collision",
                                processCollisionTask,
                                createCollisionLocalStoreMemory,
                                maxNumOutstandingTasks);
                        m_threadSupportCollision = new PosixThreadSupport(constructionInfo);
                    #else //USE_LIBSPE2 && USE_PTHREADS
                        SequentialThreadSupport::SequentialThreadConstructionInfo colCI(
                                "collision",
                                processCollisionTask,
                                createCollisionLocalStoreMemory);
	                    SequentialThreadSupport* m_threadSupportCollision = new SequentialThreadSupport(colCI);
                    #endif //USE_LIBSPE2 && USE_PTHREADS
                #endif //USE_WIN32_THREADING
                m_dispatcher = new SpuGatheringCollisionDispatcher(
                        m_threadSupportCollision,
                        maxNumOutstandingTasks,
                        m_collisionConfiguration);
            #else //BULLET_MULTITHREADED
                m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
            #endif //BULLET_MULTITHREADED
            
            btVector3 worldAabbMin = toBtVec(worldAabb.GetCorner(0,0,0));
            btVector3 worldAabbMax = toBtVec(worldAabb.GetCorner(1,1,1));

            m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

            #ifdef BULLET_MULTITHREADED
                m_threadSupportSolver = createSolverThreadSupport(maxNumOutstandingTasks);
	            m_solver = new btParallelConstraintSolver(m_threadSupportSolver);
            #else //BULLET_MULTITHREADED
                m_solver = new btSequentialImpulseConstraintSolver();
            #endif //BULLET_MULTITHREADED

            m_dynamicsWorld = new btDiscreteDynamicsWorld(
                    m_dispatcher,
                    m_broadphase,
                    m_solver,
                    m_collisionConfiguration);

            #ifdef BULLET_MULTITHREADED
                //m_dynamicsWorld->getSimulationIslandManager()->setSplitIslands(false);
		        m_dynamicsWorld->getSolverInfo().m_numIterations = 4;
		        m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD+SOLVER_USE_WARMSTARTING;//+SOLVER_RANDMIZE_ORDER;
                m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
            #endif //BULLET_MULTITHREADED 
            m_dynamicsWorld->setGravity(toBtVec(gravity));
            timer.Start();
        }

        BulletEngine::~BulletEngine()
        {
            #ifdef BULLET_MULTITHREADED
            	if (m_threadSupportSolver)
		            delete m_threadSupportSolver;
                if (m_threadSupportCollision)
		            delete m_threadSupportCollision;
            #endif //BULLET_MULTITHREADED 
            delete m_dispatcher;
            delete m_broadphase;
            delete m_solver;
            delete m_dynamicsWorld;
            delete m_collisionConfiguration;
        }

        void BulletEngine::Initialize(){}

        void BulletEngine::Process(const float deltaTime) 
        {
            for(list< BodyPair >::iterator it = bodies.begin(); it != bodies.end(); it++)
            {
                IRigidBody & oeBody = *(*it).get<0>();
                btRigidBody & btBody = *(*it).get<1>();

                if(typeid(DynamicBody) == typeid(oeBody))
                {
                    DynamicBody & dynBody = *dynamic_cast<DynamicBody*>((*it).get<0>());

                    // change body state:
                    if(dynBody.IsStateChanged())
                    {
                        btBody.setDamping(dynBody.GetLinearDamping() ,
                                dynBody.GetAngularDamping());

                        if(dynBody.IsDisableDeactivation())
                        {
                            btBody.setActivationState(DISABLE_DEACTIVATION); // how to switch this off???
                        }

                        btTransform trans;
                        trans.setIdentity();
                        trans.setRotation(toBtQuat(dynBody.GetRotation()));
                        trans.setOrigin(toBtVec(dynBody.GetPosition()));

                        btMotionState* myMotionState = btBody.getMotionState();
                        myMotionState->setWorldTransform(trans);
                        btBody.setMotionState(myMotionState);

                        btBody.setLinearVelocity(toBtVec(dynBody.GetLinearVelocity()));
                        btBody.setAngularVelocity(toBtVec(dynBody.GetAngularVelocity()));
                    }

                    // apply forces:
                    btBody.clearForces();
                    btBody.applyTorque(toBtVec(dynBody.GetTorque()));

                    for(list<DynamicBody::ForceAtPosition>::iterator it = dynBody.GetForces().begin(); it != dynBody.GetForces().end(); it++)
                    {
                        DynamicBody::ForceAtPosition & f = *it;
                        btBody.applyForce(toBtVec(f.get<0>()),toBtVec(f.get<1>()));
                    }
                    dynBody.ResetForces();
                }
            }

            for(list< CarPair >::iterator it = cars.begin(); it != cars.end(); it++)
            {
                Car & oeCar = *(*it).get<0>();
                btRaycastVehicle & btCar = *(*it).get<1>();

                CarConfig & config = oeCar.GetConfig();
                const float turnPercent = (fabs(oeCar.GetTurn()) < 0.01 ? 0 : oeCar.GetTurn());
                const float forcePercent = oeCar.GetEngineForce();
                const float breakForcePercent = oeCar.GetBrake();

                if(oeCar.GetConfig().tankSteer)
                {
                    const float leftEngine = 
                        (forcePercent * config.maxEngineForce) +
                        (-turnPercent * config.maxSteerAngle * config.maxEngineForce);
                    const float rightEngine = 
                        (forcePercent * config.maxEngineForce)+
                        ( turnPercent * config.maxSteerAngle * config.maxEngineForce);

                    for (int i=0; i<btCar.getNumWheels(); i++)
                    {
                        btCar.applyEngineForce(( i%2==0 ? leftEngine : rightEngine),i);
                        btCar.setBrake(breakForcePercent*config.maxBreakForce,i);
                    }
                }
                else
                {
                    btCar.applyEngineForce(forcePercent * config.maxEngineForce,0);
                    btCar.applyEngineForce(forcePercent * config.maxEngineForce,1);
                    btCar.applyEngineForce(forcePercent * config.maxEngineForce,2);
                    btCar.applyEngineForce(forcePercent * config.maxEngineForce,3);

                    const float breakingBalance = oeCar.GetConfig().GetBreakingBalance();

                    // Breaking on all four wheels causes jitter when standing still
                    btCar.setBrake(breakForcePercent * config.maxBreakForce * breakingBalance,0);
                    btCar.setBrake(breakForcePercent * config.maxBreakForce * breakingBalance,1);
                    btCar.setBrake(breakForcePercent * config.maxBreakForce * (1-breakingBalance),2);
                    btCar.setBrake(breakForcePercent * config.maxBreakForce * (1-breakingBalance),3);
                    //logger.info << breakForcePercent << logger.end;

                    btCar.setSteeringValue(-1 * turnPercent * config.maxSteerAngle,0);
                    btCar.setSteeringValue(-1 * turnPercent * config.maxSteerAngle,1);
                    if(oeCar.GetConfig().allWheelSteer)
                    {
                        btCar.setSteeringValue(turnPercent * config.maxSteerAngle,2);
                        btCar.setSteeringValue(turnPercent * config.maxSteerAngle,3);
                    }
                }
                //logger.info << turnPercent << logger.end;
            }

            float sec = deltaTime/100000.0;
            sec = timer.GetElapsedTimeAndReset().AsInt()/100000.0;
            //logger.info << sec << logger.end;

            m_dynamicsWorld->stepSimulation(sec, 10);

            for(list< BodyPair >::iterator it = bodies.begin(); it != bodies.end(); it++)
            {
                IRigidBody & oeBody = *(*it).get<0>();
                btRigidBody & btBody = *(*it).get<1>();

                if(typeid(DynamicBody) == typeid(oeBody))
                {
                    DynamicBody & dynBody = *dynamic_cast<DynamicBody*>((*it).get<0>());
                    dynBody.SetPosition(toOEVec(btBody.getCenterOfMassPosition()));
                    dynBody.SetRotation(toOEQuat(btBody.getOrientation()));
                    dynBody.SetAngularVelocity(toOEVec(btBody.getAngularVelocity()));
                    dynBody.SetLinearVelocity(toOEVec(btBody.getLinearVelocity()));
                    dynBody.SetStateChanged(false);

                    // logger.info << toOEVec(btBody.getCenterOfMassPosition()) << logger.end;
                }  
            }
            for(list< CarPair >::iterator it = cars.begin(); it != cars.end(); it++) 
            {
                Car & oeCar = *(*it).get<0>();
                DynamicBody * oeChassis = oeCar.GetChassis();
                btRaycastVehicle & btCar = *(*it).get<1>();
                btRigidBody * btChassis = btCar.getRigidBody();

                oeChassis->SetPosition(toOEVec(btChassis->getCenterOfMassPosition()));
                oeChassis->SetRotation(toOEQuat(btChassis->getOrientation()));
                oeChassis->SetAngularVelocity(toOEVec(btChassis->getAngularVelocity()));
                oeChassis->SetLinearVelocity(toOEVec(btChassis->getLinearVelocity()));
                oeChassis->SetStateChanged(false);

                //  for (int i=0; i<btCar.getNumWheels(); i++ )
                //  {
                //      btCar.updateWheelTransform(i);
                //  }
            }
        }


        void BulletEngine::Deinitialize() { }

        void BulletEngine::ClientResetScene(btRigidBody * chassis) {

            chassis->setCenterOfMassTransform(btTransform::getIdentity());
            chassis->setLinearVelocity(btVector3(0,0,0));
            chassis->setAngularVelocity(btVector3(0,0,0));
            m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(chassis->getBroadphaseHandle(),
                    m_dynamicsWorld->getDispatcher());

        }

        void BulletEngine::CreateDynamicBody(IRigidBody * body, btCollisionShape * shape, btTransform trans) 
        {
            /*
               if(typeid(btHeightfieldTerrainShape) == typeid(*shape) ||
               typeid(btBvhTriangleMeshShape) == typeid(*shape)) {
               throw NotImplemented("dynamic concave bodies not supported");
               }
               */
            DynamicBody & dynBody = *dynamic_cast<DynamicBody*>(body);

            btRigidBody* btBody = localCreateRigidBody(dynBody.GetMass(), trans, shape);

            if(dynBody.IsDisableDeactivation())
            {
                btBody->setActivationState(DISABLE_DEACTIVATION);
            }

            btBody->setLinearVelocity(toBtVec(dynBody.GetLinearVelocity()));
            btBody->setAngularVelocity(toBtVec(dynBody.GetAngularVelocity()));

            bodies.push_back(BodyPair(body, btBody));
        }

        void BulletEngine::CreateStaticBody(OpenEngine::Physics::IRigidBody * body, 
                btCollisionShape * shape,
                btTransform trans) 
        {
            btRigidBody* btBody = localCreateRigidBody(0.0f,trans,shape);

            //         btBody->setCollisionFlags( btBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
            //         btBody->setActivationState(DISABLE_DEACTIVATION);

            bodies.push_back(BodyPair(body,btBody));
        }

        void BulletEngine::AddRigidBody(IRigidBody * body)
        {
            // set transformation
            btTransform trans;
            trans.setIdentity();
            trans.setOrigin(toBtVec(body->GetPosition()));
            trans.setRotation(toBtQuat(body->GetRotation()));

            // set shape
            btCollisionShape* shape;
            if(typeid(DynamicBody) == typeid(*body))
               shape = ConvertShape(body->GetShape(), true);
            else
               shape = ConvertShape(body->GetShape(), false);
            
            // create body:
            if(typeid(DynamicBody) == typeid(*body))
            {
                CreateDynamicBody(body, shape, trans);
            }
            // create car:
            else if(typeid(Car) == typeid(*body))
            {

                if(typeid(*body->GetShape()) != typeid(AABB))
                {
                    throw NotImplemented("Only AABB implemented as shape for a car.");
                }
                Car * car = dynamic_cast<Car*>(body);
                DynamicBody * chassis = car->GetChassis();
                btCollisionShape* chassisShape = ConvertShape(chassis->GetShape());
                CarConfig & carConfig(car->GetConfig());

                // create rigid body
                btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
                btVector3 localInertia(0,0,0);
                chassisShape->calculateLocalInertia(chassis->GetMass(),localInertia);
                btRigidBody::btRigidBodyConstructionInfo cInfo(chassis->GetMass(),myMotionState,chassisShape,localInertia);
                btRigidBody* btBody = new btRigidBody(cInfo);
                btBody->setActivationState(DISABLE_DEACTIVATION);
                m_dynamicsWorld->addRigidBody(btBody);
                btBody->setLinearVelocity(toBtVec(chassis->GetLinearVelocity()));
                btBody->setAngularVelocity(toBtVec(chassis->GetAngularVelocity()));

                // create vehicle
                btRaycastVehicle::btVehicleTuning * tuning =
                    new btRaycastVehicle::btVehicleTuning();
                btVehicleRaycaster * raycaster =
                    new btDefaultVehicleRaycaster(m_dynamicsWorld);
                btRaycastVehicle * vehicle = new btRaycastVehicle(*tuning,btBody,raycaster);
                cars.push_back(CarPair(dynamic_cast<Car*>(body),vehicle));
                m_dynamicsWorld->addVehicle(vehicle);
                vehicle->setCoordinateSystem(carConfig.rightIndex,
                        carConfig.upIndex,
                        carConfig.forwardIndex);

                bool isFrontWheel = true;
                bool isLeftWheel = true;

                const int numWheels = carConfig.GetNumberOfWheels();

                //@todo: unused variable: const float wheelSpacing = (carConfig.wheelDistanceLength*2)/((numWheels/2)-1);


                for (int i = 0; i < numWheels; i++) {

                    //@todo: unused variable: int sideIndex = (i%2 == 0) ? i/2 : (i-1)/2;

                    //@todo: unused variable: float position = carConfig.wheelDistanceLength - (wheelSpacing*sideIndex);

                    //           logger.info << "sideIndex " << sideIndex << logger.end;
                    //           logger.info << "position " << position << logger.end;

                    isFrontWheel = (i < 2);
                    isLeftWheel = (i%2 == 0);

                    btVector3 connectionPointCS0((isFrontWheel ? 1 : -1)*carConfig.wheelDistanceLength,
                            carConfig.connectionHeight,
                            (isLeftWheel ? 1 : -1)*carConfig.wheelDistanceWidth);

                    vehicle->addWheel(connectionPointCS0,
                            toBtVec(carConfig.wheelDirectionCS0),
                            toBtVec(carConfig.wheelAxleCS),
                            carConfig.suspensionRestLength,
                            carConfig.wheelRadius,
                            *tuning,
                            true);


                }

                //        logger.info << "numWheels" << vehicle->getNumWheels() << logger.end;


                for (int i=0;i<vehicle->getNumWheels();i++)
                {
                    btWheelInfo& wheel = vehicle->getWheelInfo(i);
                    wheel.m_suspensionStiffness = carConfig.suspensionStiffness;
                    wheel.m_wheelsDampingRelaxation = carConfig.suspensionDamping;
                    wheel.m_wheelsDampingCompression = carConfig.suspensionCompression;
                    wheel.m_frictionSlip = carConfig.wheelFriction;
                    wheel.m_rollInfluence = carConfig.rollInfluence;
                }

            }
            else
            {
                if(typeid(RigidBody) != typeid(*body))
                {
                    logger.warning << "Body " << body->GetName() << " added as static body" << logger.end;
                }
                CreateStaticBody(body,shape,trans);
            }
        }    

        struct FindBtBody {

            FindBtBody(OpenEngine::Physics::IRigidBody * body) : body(body) {}

            bool operator() (const tuple<IRigidBody*,btRigidBody*> & pair) {
                return pair.get<0>() == body;
            }

            OpenEngine::Physics::IRigidBody * body;
        };

        struct FindOEBody {

            FindOEBody(btRigidBody * body) : body(body) {}

            bool operator() (const tuple<IRigidBody*,btRigidBody*> & pair) {
                return pair.get<1>() == body;
            }

            btRigidBody * body;
        };

        struct FindBtVehicle {

            FindBtVehicle(OpenEngine::Physics::Car * car) : car(car) {}

            bool operator() (const tuple<Car*,btRaycastVehicle*> & pair) {
                return pair.get<0>() == car;
            }

            OpenEngine::Physics::Car * car;
        };

        void BulletEngine::RemoveRigidBody(OpenEngine::Physics::IRigidBody * body) 
        {
            if(typeid(*body)!=typeid(Car)) {
                FindBtBody pred(body);
                list< BodyPair >::iterator it = find_if(bodies.begin(),bodies.end(),pred);
                if(it != bodies.end()) {
                    BodyPair& pair = *it;
                    bodies.remove_if(pred);
                    m_dynamicsWorld->removeRigidBody(pair.get<1>());
                    delete pair.get<1>();
                }
                else {
                    logger.warning << "Could not remove rigid body " << body->GetName() << logger.end;
                }
            }
            else {
                FindBtVehicle pred(dynamic_cast<Car*>(body));
                list< CarPair >::iterator it = find_if(cars.begin(),cars.end(),pred);

                if(it != cars.end()) {
                    CarPair& pair = *it;
                    cars.remove_if(pred);
                    m_dynamicsWorld->removeVehicle(pair.get<1>());
                    delete pair.get<1>();
                }
                else {
                    logger.warning << "Could not remove rigid body " << body->GetName() << logger.end;
                }
            }      
        }

        IRigidBody * BulletEngine::LookUp(btRigidBody * body) {
            list< BodyPair >::iterator it = find_if(bodies.begin(),bodies.end(),FindOEBody(body));
            if(it != bodies.end()) {
                BodyPair& pair = *it;
                return pair.get<0>();
            }
            return NULL;
        }

        btRigidBody * BulletEngine::LookUp(IRigidBody * body) {
            list< BodyPair >::iterator it = find_if(bodies.begin(),bodies.end(),FindBtBody(body));
            if(it != bodies.end()) {
                BodyPair& pair = *it;
                return pair.get<1>();
            }
            return NULL;
        }

        void BulletEngine::RayTest(const Vector<3,float> & begin, const Vector<3,float> & end, IRayResultCallback * callback) {
            BulletRayResultCallback btcallback(callback,this);
            btVector3 btBegin = toBtVec(begin);
            btVector3 btEnd = toBtVec(end);
            m_dynamicsWorld->rayTest(btBegin,btEnd,btcallback);
        }

        OpenEngine::Scene::RenderNode * BulletEngine::getRenderNode(OpenEngine::Renderers::IRenderer * renderer) 
        {
            DebugDrawer * draw = new DebugDrawer(renderer);
            draw->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE);
            m_dynamicsWorld->setDebugDrawer(draw);
            return new BulletDebugNode(m_dynamicsWorld);
        }

        btRigidBody* BulletEngine::localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape)
        {
            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = (mass != 0.f);

            btVector3 localInertia(0,0,0);
            if (isDynamic)
            {
                shape->calculateLocalInertia(mass,localInertia);
            }

            btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

            btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
            btRigidBody* body = new btRigidBody(cInfo);

            m_dynamicsWorld->addRigidBody(body);

            return body;
        }

        btCollisionShape * BulletEngine::ConvertShape(OpenEngine::Geometry::GeometryBase * geom, bool isDynamic)
        {
            if(typeid(BoundingSphere) == typeid(*geom))
            {
                Sphere sphere = ((BoundingSphere*)geom)->getSphere();
                return new btSphereShape(sphere.GetRadius());
            }
            // case Box
            else if(typeid(AABB) == typeid(*geom))
            {
                const AABB & box = *dynamic_cast<const AABB* >(geom);
                Vector<3,float> size = box.GetCorner();
                return new btBoxShape(toBtVec(size));
            }
            else if(typeid(TriangleMesh) == typeid(*geom))
            {
                TriangleMesh* triangleMesh = dynamic_cast<TriangleMesh*>(geom);
                btTriangleMesh* triMesh = new btTriangleMesh();

                for(unsigned int x=0; x<triangleMesh->GetSize(); x++)
                {
                    //Get Iterator
                    TriangleIterator TI(triangleMesh->GetMesh(x));

                    for(TI.first(); TI.isGood(); TI++)
                    {
                        triMesh->addTriangle(toBtVec(TI.vec[0]), toBtVec(TI.vec[1]), toBtVec(TI.vec[2]));
                    }
                }
                //cout << "NumTriangles in triangleMesh: " << triMesh->getNumTriangles() << std::endl;

                #if OE_SAFE
                if(triMesh->getNumTriangles()==0)
                    throw Exception("ERROR ADDED TRIANGLE MESH (PHYSICS)");
                #endif

                if(isDynamic)
                    return new btConvexTriangleMeshShape(triMesh);
                
                // make a new triangle mesh shape
                bool useQuantizedBvhTree = true;
                btCollisionShape* shape = new btBvhTriangleMeshShape(triMesh, useQuantizedBvhTree);
                //((btBvhTriangleMeshShape*)shape)->recalcLocalAabb(); // do we need to do this?
                return shape;
            }
            else if(typeid(CompoundShape) == typeid(*geom))
            {
                CompoundShape * compound = dynamic_cast<CompoundShape*>(geom);
                btCompoundShape * cShape = new btCompoundShape();
                for(int i = 0; i < compound->getNumChildShapes(); i++) {      
                    OpenEngine::Geometry::GeometryBase * shape = compound->getChildShape(i);
                    TransformationNode * tn = compound->getChildTransform(i);
                    btCollisionShape * collShape = ConvertShape(shape);
                    btTransform * btTrans = new btTransform(toBtQuat(tn->GetRotation()), toBtVec(tn->GetPosition()));
                    cShape->addChildShape(*btTrans, collShape);
                }
                return cShape;
            }
            else if(typeid(HeightfieldTerrainShape) == typeid(*geom))
            {
                HeightfieldTerrainShape * hts = dynamic_cast<HeightfieldTerrainShape*>(geom);
                //FloatTexture2DPtr tex = hts->GetTextureResource();
                IDataBlockPtr blk = hts->GetDataBlock();
                //logger.info << "tex: " << tex->GetWidth() << "x" << tex->GetHeight() << logger.end;

                int w = hts->GetWidth();
                int d = hts->GetDepth();

                float *input = (float*)blk->GetVoidDataPtr();

                float *arr = new float[w * d]; 

                // Magic indexing :-)
                for (int x=0;x<w;x++) {
                    for (int y=0;y<d;y++) {
                        int idx = x*w + y;
                        int idy = (w-1-x)*w*4+y*4+1;
                        arr[idx] = input[idy];
                        logger.info << arr[idx] << logger.end;
                    }
                }


                btHeightfieldTerrainShape * bhts 
                    = new btHeightfieldTerrainShape(w,
                            d,
                            arr,
                            1,
                            0,
                            hts->GetMaxHeight(),
                            hts->GetUpAxis(),
                            hts->UsesFloatData()?PHY_FLOAT:PHY_UCHAR,
                            hts->FlippedQuadEdges() );
                bhts->setUseDiamondSubdivision(true);
                float scaling = hts->GetScaling();
                btVector3 localScaling(scaling, scaling, scaling);
                localScaling[hts->GetUpAxis()]=1.f;
                bhts->setLocalScaling(localScaling);

                //delete[] arr; // Will it be copied? - I think not

                return bhts;


            }
            else
            {
                throw NotImplemented();
            }
        }
    }
}
