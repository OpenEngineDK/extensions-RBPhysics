#include "BulletEngine.h"
#include "DebugDrawer.h"
#include "BulletDebugNode.h"
#include "Util.h"
#include <Core/IGameFactory.h>
#include <Geometry/AABB.h>
#include <Geometry/Sphere.h>
#include <Geometry/TriangleMesh.h>
#include <Geometry/CompoundShape.h>
#include <Geometry/HeightfieldTerrainShape.h>
#include <Core/Exceptions.h>
#include <Logging/Logger.h>
#include "../Physics/RigidBody.h"
#include "../Physics/DynamicBody.h"
#include "BulletRayResultCallback.h"
#include <src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

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

        btCar.applyEngineForce(oeCar.GetEngineForce(),0); // right rear wheel
        btCar.applyEngineForce(oeCar.GetEngineForce(),1); // right front wheel
        btCar.applyEngineForce(oeCar.GetEngineForce(),2); // left front wheel
        btCar.applyEngineForce(oeCar.GetEngineForce(),3); // left rear wheel
	//logger.info << oeCar.GetEngineForce() << logger.end;

	// Breaking on all four wheels causes jitter when standing still
        btCar.setBrake(oeCar.GetBrake(),0);
        //btCar.setBrake(oeCar.GetBrake(),1);
	//btCar.setBrake(oeCar.GetBrake(),2);
        btCar.setBrake(oeCar.GetBrake(),3);
	//logger.info << oeCar.GetBrake() << logger.end;

        btCar.setSteeringValue(oeCar.GetTurn(),1);
        btCar.setSteeringValue(oeCar.GetTurn(),2);
        //logger.info << oeCar.GetTurn() << logger.end;
      }

      m_dynamicsWorld->stepSimulation(1.0f/deltaTime,5);

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

	int i;
	for ( i=0; i<btCar.getNumWheels(); i++ ) {
	  btCar.updateWheelTransform(i);
	}
      }
    }
    

    void BulletEngine::Deinitialize() { }

    void BulletEngine::ClientResetScene(btRigidBody * chassis) {

      chassis->setCenterOfMassTransform(btTransform::getIdentity());
      chassis->setLinearVelocity(btVector3(0,0,0));
      chassis->setAngularVelocity(btVector3(0,0,0));
      m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(chassis->getBroadphaseHandle(),m_dynamicsWorld->getDispatcher());

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
        DynamicBody * chassis = (dynamic_cast<Car*>(body))->GetChassis();

	btCollisionShape* chassisShape = new btBoxShape(btVector3(20.f,10.f,20.f));
	btCompoundShape* compound = new btCompoundShape();
	btTransform localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(0,10,0));
	compound->addChildShape(localTrans,chassisShape);
	//trans.setOrigin(btVector3(0,0.f,0));
	btRigidBody * btBody = localCreateRigidBody(chassis->GetMass(),trans,compound);

	//        btRigidBody * btBody = localCreateRigidBody(chassis->GetMass(),trans,shape);

        //	ClientResetScene(btBody);

        btBody->setLinearVelocity(toBtVec(chassis->GetLinearVelocity()));
        btBody->setAngularVelocity(toBtVec(chassis->GetAngularVelocity()));

        btRaycastVehicle::btVehicleTuning * tuning =
          new btRaycastVehicle::btVehicleTuning();

        btVehicleRaycaster * raycaster =
          new btDefaultVehicleRaycaster(m_dynamicsWorld);

        btRaycastVehicle * vehicle = new btRaycastVehicle(*tuning,btBody,raycaster);

        cars.push_back(CarPair(dynamic_cast<Car*>(body),vehicle));

        btBody->setActivationState(DISABLE_DEACTIVATION);

        m_dynamicsWorld->addVehicle(vehicle);

        int rightIndex = 0;
        int upIndex = 1;
        int forwardIndex = 2;
        float	wheelRadius = 13.5f;
        float	wheelWidth = 5.4f;
        float	wheelFriction = 5;
        btVector3 wheelDirectionCS0(0,-1,0);
        btVector3 wheelAxleCS(0,0,1);
        float connectionHeight = 10.f;
        btScalar suspensionRestLength(0.0);
        float	suspensionStiffness = 200.f;
        float	suspensionDamping = 20.3f;
        float	suspensionCompression = 0.4f;
        float	rollInfluence = 1.3f;

        vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

#define CUBE_HALF_EXTENTS 15

        bool isFrontWheel=true;

        btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
        vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,*tuning,isFrontWheel);

        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
        vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,*tuning,isFrontWheel);

        isFrontWheel = false;

        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
        vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,*tuning,isFrontWheel);

        connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
        vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,*tuning,isFrontWheel);
		
        for (int i=0;i<vehicle->getNumWheels();i++)
          {
            btWheelInfo& wheel = vehicle->getWheelInfo(i);
            wheel.m_suspensionStiffness = suspensionStiffness;
            wheel.m_wheelsDampingRelaxation = suspensionDamping;
            wheel.m_wheelsDampingCompression = suspensionCompression;
            wheel.m_frictionSlip = wheelFriction;
            wheel.m_rollInfluence = rollInfluence;
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
      if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

      //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

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
