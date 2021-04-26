#include <random>

#include <dart/dart.hpp>
#include <dart/gui/glut/glut.hpp>

enum SoftShapeType
{
  SOFT_BOX = 0,
  SOFT_CYLINDER,
  SOFT_ELLIPSOID
};

void setupRing(const dart::dynamics::SkeletonPtr& ring);
void setAllColors(const dart::dynamics::SkeletonPtr& object, const Eigen::Vector3d& color);

dart::dynamics::SkeletonPtr createBall();
dart::dynamics::SkeletonPtr createRigidChain();
dart::dynamics::SkeletonPtr createRigidRing();
dart::dynamics::SkeletonPtr createSoftBody();
dart::dynamics::SkeletonPtr createHybridBody();
dart::dynamics::SkeletonPtr createGround();
dart::dynamics::SkeletonPtr createWall();
