/* mlhole - a small example for modelling a nail and a hole
   Copyright (c) 2016, Friedrich Beckmann, Hochschule Augsburg

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>. */


#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "controller.hpp"

const double default_height = 1.0; // m
const double default_width = 0.2;  // m
const double default_depth = 0.2;  // m

const double default_torque = 5.0; // N-m
const double default_force =  15.0; // N
const int default_countdown = 20;  // Number of timesteps for applying force

const double default_step = 0.001;
const double default_angle = 1.0;

using namespace dart::dynamics;
using namespace dart::simulation;

class MyWindow : public dart::gui::SimWindow
{
public:

  /// Constructor
  MyWindow(WorldPtr world,
           Controller* ctrl)
    : mBallConstraint(nullptr),
      mPositiveSign(true),
      mBodyForce(false),
      singlestep(false)
  {
    setWorld(world);

    // Find the Skeleton named "pendulum" within the World
    mStick = world->getSkeleton("stick");

    // Make sure that the pendulum was found in the World
    assert(mStick != nullptr);

    mFloor = world->getSkeleton("floor");
    mdata.resize(1000);
    mForceCountDown.resize(mStick->getNumDofs(), 0);

    mController = ctrl;

    mTargetPosition << 30.0*M_PI/180.0, 0.0, 0.0, 0.2, 0.3, 0.2;

    ArrowShape::Properties arrow_properties;
    arrow_properties.mRadius = 0.05;
    mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
                                                        Eigen::Vector3d(-default_height, 0.0, default_height / 2.0),
                                                        Eigen::Vector3d(-default_width / 2.0, 0.0, default_height / 2.0),
                                                        arrow_properties, dart::Color::Orange(1.0)));
  }

  void applyForce(std::size_t index)
  {
    if(index < mForceCountDown.size())
      mForceCountDown[index] = default_countdown;
  }

  void drawWorld() const {
    /* Enable filled polygons not just wireframes */
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    SimWindow::drawWorld();
  }

  void initLights() {
    static float ambient[]             = {0.0, 0.0, 0.0, 1.0};
    static float diffuse[]             = {1.0, 1.0, 1.0, 1.0};
    static float front_mat_shininess[] = {60.0};
    static float front_mat_specular[]  = {0.2, 0.2,  0.2,  1.0};
    static float front_mat_diffuse[]   = {0.5, 0.28, 0.38, 1.0};
    static float lmodel_ambient[]      = {0.2, 0.2,  0.2,  1.0};
    static float lmodel_twoside[]      = {GL_FALSE};

    GLfloat position[] = {0.0, 1.0, 2.0, 0.0};
    GLfloat position1[] = {-1.0, 0.0, 0.0, 0.0};

    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    /*
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodel_ambient);
    glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, position1);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);

    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  front_mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   front_mat_diffuse);
    */
    
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glEnable(GL_NORMALIZE);
  }
  
  void render()
  {
    if (mShow3D) {
      SimWindow::render();
      return;
    }
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(-1.0, 1.0, -1 * 480.0/640.0, 1 * 480.0/640.0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    initGL();

    glScalef(mZoom, mZoom, mZoom);
    glTranslatef(mTrans[0]/640.0*2.0, mTrans[1]/480.0*2.0*48.0/64.0, mTrans[2]/480.0*2.0*48.0/64.0);
    glRotatef(90,-1.0,0,0); // Rotate around x axis to look at pendulum
    initLights();
    draw();
    glutSwapBuffers();
  }

  void click(int _button, int _state, int _x, int _y) {
    if (mShow3D) {
      Win3D::click(_button, _state, _x, _y);
      return;
    }
    mMouseDown = !mMouseDown;
    if (mMouseDown) {
      if (_button == 3 && _state == GLUT_DOWN) {  // mouse wheel up
        // each scroll generates a down and an immediate up,
        // so ignore ups
        mZoom *= 1.1;
      } else if (_button == 4 && _state == GLUT_DOWN) {  // mouse wheel down?
        // each scroll generates a down and an immediate up,
        // so ignore ups
        mZoom *= 0.9;
      }
      mMouseX = _x;
      mMouseY = _y;
    }
    glutPostRedisplay();
  }

  void drag(int _x, int _y) {
    if (mShow3D) {
      Win3D::drag(_x, _y);
      return;
    }
    double deltaX = _x - mMouseX;
    double deltaY = _y - mMouseY;

    mMouseX = _x;
    mMouseY = _y;
    mTrans += Eigen::Vector3d(deltaX / mZoom, -deltaY / mZoom, 0.0);

    glutPostRedisplay();
  }
  
  /// Handle keyboard input
  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
      {
      case '-':
        mPositiveSign = !mPositiveSign;
        break;
      case '1':
        applyForce(0);
        break;
      case '2':
        applyForce(1);
        break;
      case '3':
        applyForce(2);
        break;
      case '4':
        applyForce(3);
        break;
      case '5':
        applyForce(4);
        break;
      case '6':
        applyForce(5);
        break;
      case '7':
        applyForce(6);
        break;
      case '8':
        applyForce(7);
        break;
      case '9':
        applyForce(8);
        break;
      case '0':
        applyForce(9);
        break;

      case 'q':
        mTargetPosition(3) += default_step;
        break;
      case 'a':
        mTargetPosition(3) -= default_step;
        break;

      case 'w':
        mTargetPosition(4) += default_step;
        break;
      case 's':
        mTargetPosition(4) -= default_step;
        break;

      case 'e':
        mTargetPosition(5) += default_step;
        break;
      case 'd':
        mTargetPosition(5) -= default_step;
        break;

      case 'r':
        mTargetPosition(0) += default_angle * M_PI/180.0;
        break;
      case 'f':
        mTargetPosition(0) -= default_angle * M_PI/180.0;
        break;
      case 'm':
        singlestep = !singlestep;
        break;
      case 'n':
        mController->update(mTargetPosition);
        mWorld->step();
        break;
      case 'p':
        plot(mdata);
        break;
      case 'l':
        for(int i = 0; i<1000;i++)
          mWorld->step();
        break;
                
      case 'v':
        mShow3D = !mShow3D;
        break;
      default:
        SimWindow::keyboard(key, x, y);
      }
  }

  void timeStepping() override
  {

    if (singlestep)
      return;

    //std::cout << "FC: " << mStick->getBodyNode("cylinder_link")->getFrictionCoeff() << std::endl;
    
    mController->update(mTargetPosition);

    if(!mBodyForce)
      {
        // Apply joint torques based on user input, and color the Joint shape red
        for(std::size_t i = 0; i < mStick->getNumDofs(); ++i)
          {
            if(mForceCountDown[i] > 0)
              {
                DegreeOfFreedom* dof = mStick->getDof(i);
                dof->setForce( mPositiveSign? default_torque : -default_torque );

                //BodyNode* bn = dof->getChildBodyNode();
                //auto visualShapeNodes = bn->getShapeNodesWith<VisualAspect>();
                //visualShapeNodes[0]->getVisualAspect()->setColor(dart::Color::Red());

                --mForceCountDown[i];
              }
          }
      }
    else
      {
        Eigen::Vector3d force = Eigen::Vector3d::Zero();
        bool apply = false;
        for(std::size_t i = 0; i < 3;i++)
          {
            if (mForceCountDown[i] > 0)
              {
                force[i] = mPositiveSign ? 10.0 : -10.0;
                apply = true;
                --mForceCountDown[i];
              }
          }
        std::cout << "Force: " << force << std::endl;
        if (apply)
          mStick->getBodyNode("cylinder_link")->addExtForce(force);
      
      }

    // Show Forces
    {
      Eigen::VectorXd corforces = mStick->getConstraintForces();
      Eigen::VectorXd gforces = mStick->getGravityForces();
      //Eigen::VectorXd conforces = m->getConstraintForces();
      std::cout << "Constraint Forces: " << corforces << std::endl;
      //std::cout << "Gravity Forces: " << gforces << std::endl;
      //std::cout << "Constraint Forces: " << conforces << std::endl;

      //BodyNode* bn = mFloor->getBodyNode("cylinder_link");
      //Eigen::Vector6d bodyforce = bn->getBodyForce();
      //std::cout << "Body Forces: " << bodyforce << std::endl;
      //mdata(mdataidx++ % mdata.size()) = corforces(5);
      
    }

    // Show Position in World Coordinates
    {
      BodyNodePtr bn = mStick->getBodyNode("cylinder_link");
      Eigen::Vector3d pos = bn->getWorldTransform().translation();
      //std::cout << "Pos: " << pos << std::endl;
      Eigen::AngleAxisd aa(bn->getWorldTransform().linear());
      //std::cout << "aa: " << aa << std::endl;
      //std::cout << "Angle: " << aa.angle() << "Axis : " << aa.axis() << std::endl;
      dart::dynamics::Joint *j = mStick->getJoint("cylinder_joint");
      //std::cout << "Pos: " << j->getPositions() << std::endl;
    }

    // Step the simulation forward
    SimWindow::timeStepping();
  }

protected:

  /// An arrow shape that we will use to visualize applied forces
  std::shared_ptr<ArrowShape> mArrow;

  /// The stick
  SkeletonPtr mStick;

  SkeletonPtr mFloor;

  Eigen::Vector6d mTargetPosition;

  int mdataidx = 0;
  Eigen::VectorXd mdata;

  bool singlestep;

  /// Pointer to the ball constraint that we will be turning on and off
  dart::constraint::BallJointConstraintPtr mBallConstraint;

  /// Number of iterations before clearing a force entry
  std::vector<int> mForceCountDown;

  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;

  /// True if 1-9 should be used to apply a body force. Otherwise, 1-9 will be
  /// used to apply a joint torque.
  bool mBodyForce;

  bool mShow3D = false;

  Controller* mController;
};

SkeletonPtr createFloor()
{
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr floor =
    loader.parseSkeleton("/home/fritz/mlhole/data/floor/floor.urdf");
  floor->setName("floor");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);

  floor->getJoint("world_joint")->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createStick()
{
  SkeletonPtr stick = Skeleton::create("stick");

  BodyNode::Properties bodyProp;
  bodyProp.mName = "cylinder_link";
  bodyProp.mInertia.setMass(1.0);
  // Do not consider friction - switch to frictionless mode
  bodyProp.mFrictionCoeff = 1e-8;

  FreeJoint::Properties jointProp;
  jointProp.mName = "cylinder_joint";
  
  // Give the floor a body
  BodyNodePtr body =
    stick->createJointAndBodyNodePair<FreeJoint>(nullptr, jointProp, bodyProp).second;

  body->setGravityMode(false);

  // Give the body a shape
  double radius = 0.005;
  double height = 0.2;
  std::shared_ptr<CylinderShape> cylinder(
                                new CylinderShape(radius, height));
  auto shapeNode
    = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(cylinder);
  shapeNode->getVisualAspect()->setColor(dart::Color::Green());

  // Put the body into position
  /*
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.2, 0.3, 0.0);
  tf.rotate (Eigen::AngleAxisd(30.0*M_PI/180.0, Eigen::Vector3d::UnitX()));
  body->getParentJoint()->setTransformFromParentBodyNode(tf);
  */
  dart::dynamics::Joint *j = body->getParentJoint();
  j->setPosition(0,30.0*M_PI/180.0);
  j->setPosition(3,0.2);
  j->setPosition(4,0.3);
  j->setPosition(5,0.2);

  return stick;
}
  

int main(int argc, char* argv[])
{

  // Create the stick
  SkeletonPtr stick = createStick();
  
  // Create a world and add the stick to the world
  WorldPtr world(new World);
  world->addSkeleton(stick);

  // Add the floor
  SkeletonPtr floor = createFloor();
  world->addSkeleton(floor);

  // Create a window for rendering the world and handling user input
  MyWindow window(world,
                  new Controller(stick));

  // Print instructions
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': replay simulation" << std::endl;
  std::cout << "'1' -> '9': apply torque to a pendulum body" << std::endl;
  std::cout << "'-': Change sign of applied joint torques" << std::endl;
  std::cout << "'q': Increase joint rest positions" << std::endl;
  std::cout << "'a': Decrease joint rest positions" << std::endl;
  std::cout << "'w': Increase joint spring stiffness" << std::endl;
  std::cout << "'s': Decrease joint spring stiffness" << std::endl;
  std::cout << "'e': Increase joint damping" << std::endl;
  std::cout << "'d': Decrease joint damping" << std::endl;
  std::cout << "'r': add/remove constraint on the end of the chain" << std::endl;
  std::cout << "'f': switch between applying joint torques and body forces" << std::endl;

  // Initialize glut, initialize the window, and begin the glut event loop
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Multi-Pendulum Tutorial");
  glutSetCursor(GLUT_CURSOR_CROSSHAIR);
  glutMainLoop();
}
