This is the OpenEngine (www.openengine.dk) extension for rigid bodies.

A darcs repository is available at:
http://www.daimi.au.dk/~dodger/RBPhysics/
If you really want to be up to date you can check out our svn repo:
svn co svn+ssh://fh.daimi.au.dk/users/dodger/.SVNHOME/oephys_ext

You need to extract the following file to your extensions dir:
http://www.daimi.au.dk/~dodger/bullet.zip

To use this extension create a Physics::PhysicsFacde, add it to your
game engine as IModule. Then you can start to add objects to the
physics world. 
You only need the classes from the Physics and Geometry
folder.
Physics::RigidBody can be used to add static geometry to the
scene, while Physics::DynamicBody is intended for dynamic
bodies. These can be affected through methods like AddForce() and
AddTorque().

The shapes that are available now are AABB (Axis Aligned Bounding
Box), Sphere, Triangle meshes and Compound Shapes.

If you like to see an example of how the whole thing can be set up, you
can check out our test project we are using for developing the engine.

svn repo:
svn co svn+ssh://fh.daimi.au.dk/users/dodger/.SVNHOME/oephys_projprojects/phystest
working copy:
/users/dodger/oephys2/projects/phystest

This extension can not be used together with FixedTimeStepsPhysics,
since some of our classes have the same name.

