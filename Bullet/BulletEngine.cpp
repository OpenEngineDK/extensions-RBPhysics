#include <Bullet/BulletEngine.h>

#include <Bullet/DebugDrawer.h>
#include <Bullet/BulletDebugNode.h>
#include <Bullet/Util.h>
#include <Core/IGameFactory.h>
#include <Geometry/AABB.h>
#include <Geometry/Sphere.h>
#include <Geometry/TriangleMesh.h>
#include <Geometry/CompoundShape.h>
#include <Geometry/HeightfieldTerrainShape.h>
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
      m_collisionConfiguration = new btDefaultCollisionConfiguration();

      m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

      btVector3 worldAabbMin = toBtVec(worldAabb.GetCorner(0,0,0));
      btVector3 worldAabbMax = toBtVec(worldAabb.GetCorner(1,1,1));

      m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

      m_solver = new btSequentialImpulseConstraintSolver();

      btDiscreteDynamicsWorld* world =
	new btDiscreteDynamicsWorld(m_dispatcher,
				    m_broadphase,
				    m_solver,
				    m_collisionConfiguration);
      m_dynamicsWorld = world;

      m_dynamicsWorld->setGravity(toBtVec(gravity));
    }

    BulletEngine::~BulletEngine() {
      delete m_dispatcher;
      delete m_broadphase;
      delete m_solver;
      delete m_dynamicsWorld;
    }

    void BulletEngine::Initialize(){}


    void BulletEngine::Process(const float deltaTime, const float percent) 
    {
      for(list< BodyPair >::iterator it = bodies.begin();
          it != bodies.end(); it++) {
        IRigidBody & oeBody = *(*it).get<0>();
        btRigidBody & btBody = *(*it).get<1>();

        if( typeid(DynamicBody) == typeid(oeBody)) {
          DynamicBody & dynBody = *dynamic_cast<DynamicBody*>((*it).get<0>());
          
          // change body state:
          if(dynBody.IsStateChanged()) {
            btBody.setDamping( dynBody.GetLinearDamping() ,
                               dynBody.GetAngularDamping());
            
            if(dynBody.IsDisableDeactivation()) {
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

          for(list<DynamicBody::ForceAtPosition>::iterator it = dynBody.GetForces().begin();
              it != dynBody.GetForces().end(); it++) {
            DynamicBody::ForceAtPosition & f = *it;
            btBody.applyForce(toBtVec(f.get<0>()),toBtVec(f.get<1>()));
          }
          dynBody.ResetForces();
        }
      }

      for(list< CarPair >::iterator it = cars.begin();
          it != cars.end(); it++) {
        Car & oeCar = *(*it).get<0>();
        btRaycastVehicle & btCar = *(*it).get<1>();

        CarConfig & config = oeCar.GetConfig();

        const float turnPercent = (fabs(oeCar.GetTurn()) < 0.01 ? 0 : oeCar.GetTurn());
        const float forcePercent = oeCar.GetEngineForce();
        const float breakForcePercent = oeCar.GetBrake();

        if(oeCar.GetConfig().tankSteer) {

          const float leftEngine = 
            (forcePercent * config.maxEngineForce) +
            (-turnPercent * config.maxSteerAngle * config.maxEngineForce);
          const float rightEngine = 
            (forcePercent * config.maxEngineForce)+
            ( turnPercent * config.maxSteerAngle * config.maxEngineForce);

          for (int i=0;i<btCar.getNumWheels();i++) {
            btCar.applyEngineForce(( i%2==0 ? leftEngine : rightEngine),i);
            btCar.setBrake(breakForcePercent*config.maxBreakForce,i);
          }
        }
        else {
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
          if(oeCar.GetConfig().allWheelSteer) {
            btCar.setSteeringValue(turnPercent * config.maxSteerAngle,2);
            btCar.setSteeringValue(turnPercent * config.maxSteerAngle,3);
          }
        }
        //logger.info << turnPercent << logger.end;
      }

      m_dynamicsWorld->stepSimulation(1.0f/deltaTime,1);

      for(list< BodyPair >::iterator it = bodies.begin();
          it != bodies.end(); it++) {

        IRigidBody & oeBody = *(*it).get<0>();
        btRigidBody & btBody = *(*it).get<1>();

        if(typeid(DynamicBody) == typeid(oeBody)) {
          DynamicBody & dynBody = *dynamic_cast<DynamicBody*>((*it).get<0>());
          dynBody.SetPosition(toOEVec(btBody.getCenterOfMassPosition()));
          dynBody.SetRotation(toOEQuat(btBody.getOrientation()));
          dynBody.SetAngularVelocity(toOEVec(btBody.getAngularVelocity()));
          dynBody.SetLinearVelocity(toOEVec(btBody.getLinearVelocity()));
          dynBody.SetStateChanged(false);

          //	  logger.info << toOEVec(btBody.getCenterOfMassPosition()) << logger.end;
        }
        
      }
      for(list< CarPair >::iterator it = cars.begin();
          it != cars.end(); it++) {

        Car & oeCar = *(*it).get<0>();
        DynamicBody * oeChassis = oeCar.GetChassis();
        btRaycastVehicle & btCar = *(*it).get<1>();
        btRigidBody * btChassis = btCar.getRigidBody();

        oeChassis->SetPosition(toOEVec(btChassis->getCenterOfMassPosition()));
        oeChassis->SetRotation(toOEQuat(btChassis->getOrientation()));
        oeChassis->SetAngularVelocity(toOEVec(btChassis->getAngularVelocity()));
        oeChassis->SetLinearVelocity(toOEVec(btChassis->getLinearVelocity()));
        oeChassis->SetStateChanged(false);

// 	for (int i=0; i<btCar.getNumWheels(); i++ ) {
// 	  btCar.updateWheelTransform(i);
// 	}
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
      if(typeid(btHeightfieldTerrainShape) == typeid(*shape) ||
         typeid(btBvhTriangleMeshShape) == typeid(*shape)) {
        throw NotImplemented("dynamic concave bodies not supported");
      }
      DynamicBody & dynBody = *dynamic_cast<DynamicBody*>(body);
        
      btRigidBody* btBody = localCreateRigidBody(dynBody.GetMass(),trans,shape);

      if(dynBody.IsDisableDeactivation()) {
        btBody->setActivationState(DISABLE_DEACTIVATION);
      }
        
      btBody->setLinearVelocity(toBtVec(dynBody.GetLinearVelocity()));
      btBody->setAngularVelocity(toBtVec(dynBody.GetAngularVelocity()));

      bodies.push_back(BodyPair(body,btBody));
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
      btCollisionShape* shape = ConvertShape(body->GetShape());
      
      // create body:
      if(typeid(DynamicBody) == typeid(*body) ) {
        CreateDynamicBody(body,shape,trans);
      }
      // create car:
      else if(typeid(Car) == typeid(*body) ) {

        if(typeid(*body->GetShape()) != typeid(AABB)) {
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

        const float wheelSpacing = (carConfig.wheelDistanceLength*2)/((numWheels/2)-1);
        
        
        for(int i = 0; i < numWheels; i++) {

          int sideIndex = (i%2 == 0) ? i/2 : (i-1)/2;

          float position = carConfig.wheelDistanceLength - (wheelSpacing*sideIndex);

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
      else {
        if(typeid(RigidBody) != typeid(*body)) {
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

    OpenEngine::Renderers::IRenderNode * BulletEngine::getRenderNode(OpenEngine::Renderers::IRenderer * renderer) 
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
      if (isDynamic) {
        shape->calculateLocalInertia(mass,localInertia);
      }

      btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

      btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

      btRigidBody* body = new btRigidBody(cInfo);


      m_dynamicsWorld->addRigidBody(body);
	
      return body;
    }

    btCollisionShape * BulletEngine::ConvertShape(OpenEngine::Geometry::Geometry * geom)
    {
      
      if(typeid(Sphere) == typeid(*geom)) {
        const Sphere & sphere = *dynamic_cast<const Sphere* >(geom);
        return new btSphereShape(sphere.GetRadius());
      }
      // case Box
      else if(typeid(AABB) == typeid(*geom)) {
        const AABB & box = *dynamic_cast<const AABB* >(geom);
        Vector<3,float> size = box.GetCorner();
        return new btBoxShape (toBtVec(size));
      }
      else if(typeid(TriangleMesh) == typeid(*geom)) {
        TriangleMesh * triangleMesh = dynamic_cast<TriangleMesh* >(geom);
        FaceSet * faces = triangleMesh->GetFaceSet();
        btTriangleMesh * triMesh = new btTriangleMesh();
        // fill triangle mesh with faces from the static geometry
        for(FaceList::iterator it = faces->begin(); it != faces->end(); it++) {
          FacePtr tmp = (FacePtr)*it;
          triMesh->addTriangle(toBtVec(tmp->vert[0]), toBtVec(tmp->vert[1]), toBtVec(tmp->vert[2]));
        }
        //cout << "NumTriangles in triangleMesh: " << triMesh.getNumTriangles() << std::endl;
        // make a new triangle mesh shape
        bool useQuantizedBvhTree = true;
        btCollisionShape* shape = NULL;
        shape = new btBvhTriangleMeshShape(triMesh, useQuantizedBvhTree);
        ((btBvhTriangleMeshShape*)shape)->recalcLocalAabb(); // do we need to do this?
        return shape;
      }
      else if(typeid(CompoundShape) == typeid(*geom)) {
        CompoundShape * compound = dynamic_cast<CompoundShape*>(geom);
        btCompoundShape * cShape = new btCompoundShape();
        for(int i = 0; i < compound->getNumChildShapes(); i++) {	  
          OpenEngine::Geometry::Geometry * shape = compound->getChildShape(i);
          TransformationNode * tn = compound->getChildTransform(i);
          btCollisionShape * collShape = ConvertShape(shape);
          btTransform * btTrans = new btTransform(toBtQuat(tn->GetRotation()), toBtVec(tn->GetPosition()));
          cShape->addChildShape(*btTrans, collShape);
        }
        return cShape;
      }
      else if(typeid(HeightfieldTerrainShape) == typeid(*geom)) {
        HeightfieldTerrainShape * hts = dynamic_cast<HeightfieldTerrainShape*>(geom);
        ITextureResourcePtr tex = hts->GetTextureResource();
        cout << "tex: " << tex->GetWidth() << tex->GetHeight() << endl;
        btHeightfieldTerrainShape * bhts = new btHeightfieldTerrainShape(tex->GetWidth(),
                                                                         tex->GetHeight(),
                                                                         tex->GetData(),
                                                                         hts->GetMaxHeight(),
                                                                         hts->GetUpAxis(),
                                                                         hts->UsesFloatData(),
                                                                         hts->FlippedQuadEdges() );
        bhts->setUseDiamondSubdivision(true);
        float scaling = hts->GetScaling();
        btVector3 localScaling(scaling, scaling, scaling);
        localScaling[hts->GetUpAxis()]=1.f;
        bhts->setLocalScaling(localScaling);

        return bhts;
	

      }
      else {
        throw NotImplemented();
      }
    }
  }
}
