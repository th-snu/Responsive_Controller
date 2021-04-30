
#include "MainInterface.h"
#include "CollisionTest.h"
#include <iostream>

MainInterface::
	MainInterface(std::string bvh, std::string ppo) : GLUTWindow(),
													  mRD(),
													  mMT(mRD()),
													  mDistribution(-1.0, std::nextafter(1.0, 2.0)),
													  mSkelCount(0)
{
	this->character_path = std::string(PROJECT_DIR) + std::string("/character/") + std::string(REF_CHARACTER_TYPE) + std::string(".xml");

	this->frame_no = 0;

	mCamera = new Camera();

	this->drag_mouse_r = 0;
	this->drag_mouse_l = 0;
	this->last_mouse_x = 0;
	this->last_mouse_y = 0;

	this->mDisplayTimeout = 33;
	this->on_animation = false;

	DPhy::Character *ref = new DPhy::Character(character_path);
	mReferenceManager = new DPhy::ReferenceManager(ref);
	mReferenceManager->LoadMotionFromBVH(std::string("/motion/") + bvh);

	// std::vector<Eigen::VectorXd> pos;

	// phase = 0;

	// for (int i = 0; i < 1000; i++)
	// {
	// 	Eigen::VectorXd p = mReferenceManager->GetPosition(phase);
	// 	pos.push_back(p);
	// 	phase += mReferenceManager->GetTimeStep(phase);
	// }

	// mMotion_bvh = pos;

	this->mCurFrame = 0;
	this->mTotalFrame = 0;

	if (bvh != "")
	{
		this->mSkel = DPhy::SkeletonBuilder::BuildFromFile(character_path).first;
		DPhy::SetSkeletonColor(mSkel, Eigen::Vector4d(164. / 255., 235. / 255., 243. / 255., 1.0));

		this->render_bvh = true;
	}

	if (ppo != "")
	{
		this->mSkel_sim = DPhy::SkeletonBuilder::BuildFromFile(character_path).first;
		DPhy::SetSkeletonColor(mSkel_sim, Eigen::Vector4d(164. / 255., 235. / 255., 13. / 255., 1.0));

		initNetworkSetting(ppo);
		this->render_sim = true;
	}

	this->mBall = createBall();
}

void MainInterface::
	display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	mCamera->viewupdate();

	DrawGround();

	DrawSkeletons();
	glutSwapBuffers();

	GUI::DrawStringOnScreen(0.8, 0.9, std::to_string(mCurFrame), true, Eigen::Vector3d::Zero());
}

void MainInterface::
	SetFrame(int n)
{
	if (render_bvh){
		if(!mMotion_bvh[n].hasNaN()) mSkel->setPositions(mMotion_bvh[n]);
	}
	if (render_sim){
		if(!mMotion_sim[n].hasNaN()) mSkel_sim->setPositions(mMotion_sim[n]);
	}
}

void MainInterface::
	DrawSkeletons()
{
	glPushMatrix();
	glTranslated(0.0, 0, 0);
	if (render_bvh)
		GUI::DrawSkeleton(this->mSkel, 0);
	if (render_sim)
		GUI::DrawSkeleton(this->mSkel_sim, 0);
	for (int i = 0; i < perturbance.size(); i++)
		GUI::DrawSkeleton(perturbance[i], 0);
	glPopMatrix();
}

void MainInterface::
	DrawGround()
{
	float size = 3.0f;
	int num_x = 30, num_z = 30;
	double ox, oz;

	// the tiled floor
	glBegin(GL_QUADS);
	glNormal3d(0.0, 1.0, 0.0);
	ox = -(num_x * size) / 2;
	for (int x = 0; x < num_x; x++, ox += size)
	{
		oz = -(num_z * size) / 2;
		for (int z = 0; z < num_z; z++, oz += size)
		{
			if (((x + z) % 2) == 0)
				glColor3f(1.0, 1.0, 1.0);
			else
				glColor3f(0.7, 0.7, 0.7);
			glVertex3d(ox, 0.0, oz);
			glVertex3d(ox, 0.0, oz + size);
			glVertex3d(ox + size, 0.0, oz + size);
			glVertex3d(ox + size, 0.0, oz);
		}
	}
	glEnd();
}

void MainInterface::
	initNetworkSetting(std::string ppo)
{
	Py_Initialize();
	try
	{
		if (ppo != "")
		{
			this->mController = new DPhy::Controller(mReferenceManager, this->character_path, true); //adaptive=true, bool parametric=true, bool record=true
			//mController->SetGoalParameters(mReferenceManager->GetParamCur());

			py::object sys_module = py::module::import("sys");
			py::str module_dir = (std::string(PROJECT_DIR) + "/network").c_str();
			sys_module.attr("path").attr("insert")(1, module_dir);

			py::object ppo_main = py::module::import("ppo");
			this->mPPO = ppo_main.attr("PPO")();
			std::string path = std::string(PROJECT_DIR) + std::string("/network/output/") + ppo;

			this->mPPO.attr("initRun")(path,
									   this->mController->GetNumState(),
									   this->mController->GetNumAction());
			RunPPO();
		}
	}
	catch (const py::error_already_set &)
	{
		PyErr_Print();
	}
}

void MainInterface::
	step()
{
	if(this->mController->IsTerminalState()) return;
	Eigen::VectorXd state = this->mController->GetState();

	py::array_t<double> na = this->mPPO.attr("run")(DPhy::toNumPyArray(state));
	Eigen::VectorXd action = DPhy::toEigenVector(na, this->mController->GetNumAction());

	this->mController->SetAction(action);

	if(!this->mController->Step()){
		return;
	}

	this->mTiming.push_back(this->mController->GetCurrentLength());

	mTotalFrame = this->mController->GetRecordSize();

	for (int i = this->mMotion_bvh.size(); i < mTotalFrame; i++){
		Eigen::VectorXd position = this->mController->GetPositions(i);
		Eigen::VectorXd position_bvh = this->mController->GetBVHPositions(i);
		position_bvh[3] -= 1.5;

		mMotion_sim.push_back(position);
		mMotion_bvh.push_back(position_bvh);
	}
}

void MainInterface::
	RunPPO()
{
	this->render_sim = true;

	mController->Reset(false);
	this->mTiming = std::vector<double>();

	step();
}

void MainInterface::
	skeyboard(int key, int x, int y)
{
}

void MainInterface::
	motion(int mx, int my)
{
	if ((drag_mouse_l == 0) && (drag_mouse_r == 0))
		return;

	if (drag_mouse_r == 1)
	{
		mCamera->Translate(mx, my, last_mouse_x, last_mouse_y);
	}
	else if (drag_mouse_l == 1)
	{
		mCamera->Rotate(mx, my, last_mouse_x, last_mouse_y);
	}
	last_mouse_x = mx;
	last_mouse_y = my;
}

void MainInterface::
	removeFirstPerturbance()
{
	auto pfirst = perturbance.front();
	// remove pfirst from the world
	mController->GetWorld()->removeSkeleton(pfirst);
	perturbance.erase(perturbance.begin());
	perturbance_timestamp.erase(perturbance_timestamp.begin());
}

void MainInterface::
	perturb()
{
	if (this->mController->IsTerminalState()) return;

	auto ball = mBall->cloneSkeleton();
	bool success = addObject(ball);

	if (success)
	{
		// ball->getBodyNode(0)->setFrictionCoeff(0.5);
		perturbance.push_back(ball);
		perturbance_timestamp.push_back(this->mCurFrame);
		
		// double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()/1000000.;
		if (perturbance.size() > 10) removeFirstPerturbance();
	}
}

bool MainInterface::
	/// Add an object to the world and toss it at the wall
	addObject(const dart::dynamics::SkeletonPtr &object)
{
	const double default_start_height = 1.0;						// m
	const double minimum_launch_angle = dart::math::toRadian(30.0); // rad
	const double maximum_launch_angle = dart::math::toRadian(70.0); // rad
	const double default_launch_angle = dart::math::toRadian(45.0); // rad
	const double default_spawn_range = 0.9 * 3;

	const double minimum_start_v = 2.5; // m/s
	const double maximum_start_v = 4.0; // m/s
	const double default_start_v = 3.5; // m/s

	const double maximum_start_w = 6 * dart::math::constantsd::pi(); // rad/s
	const double default_start_w = 3 * dart::math::constantsd::pi(); // rad/s

	// Set the starting position for the object
	Eigen::Vector6d positions(Eigen::Vector6d::Zero());

	// If randomization is on, we will randomize the starting y-location
	positions[3] = default_spawn_range * mDistribution(mMT);
	positions[4] = default_start_height;
	positions[5] = default_spawn_range * mDistribution(mMT);

	auto charskel = mController->GetSkeleton();

	positions.segment(3, 3) += charskel->getCOM();
	object->getJoint(0)->setPositions(positions);

	// Add the object to the world
	object->setName(object->getName() + std::to_string(mSkelCount++));
	for(int i = 0; i < object->getNumShapeNodes(); i++){
		object->getShapeNode(i)->setName(object->getName() + "_shape" + std::to_string(i));
	}

	// Look through the collisions to see if the new object would start in
	// collision with something
	auto mWorld = mController->GetWorld();

	auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
	auto collisionGroup = mWorld->getConstraintSolver()->getCollisionGroup();
	auto newGroup = collisionEngine->createCollisionGroup(object.get());

	// bullet collide has error
	dart::collision::CollisionOption option;
	dart::collision::CollisionResult result;
	bool collision = collisionGroup->collide(newGroup.get(), option, &result);

	// If the new object is not in collision
	if (!collision)
	{
		mWorld->addSkeleton(object);
	}
	else
	{
		// or refuse to add the object if it is in collision
		// std::cout << "The new object spawned in a collision. "
		// 		  << "It will not be added to the world." << std::endl;
		return addObject(object);
	}

	// Create reference frames for setting the initial velocity
	Eigen::Isometry3d centerTf(Eigen::Isometry3d::Identity());
	centerTf.translation() = object->getCOM();
	dart::dynamics::SimpleFrame center(dart::dynamics::Frame::World(), "center", centerTf);

	// Set the velocities of the reference frames so that we can easily give the
	// Skeleton the linear and angular velocities that we want
	double angle = default_launch_angle;
	double speed = default_start_v;
	double angular_speed = default_start_w;

	Eigen::Vector3d direction = charskel->getCOM() - object->getCOM() + Eigen::Vector3d(mDistribution(mMT), mDistribution(mMT), mDistribution(mMT)) / 5.0;
	direction.normalize();

	speed = (mDistribution(mMT) * 5 + 5) / 1.0;

	angular_speed = mDistribution(mMT) * maximum_start_w;

	Eigen::Vector3d v = speed * direction;
	Eigen::Vector3d w = angular_speed * Eigen::Vector3d::UnitY();
	center.setClassicDerivatives(v, w);

	dart::dynamics::SimpleFrame ref(&center, "root_reference");
	ref.setRelativeTransform(object->getBodyNode(0)->getTransform(&center));

	// Use the reference frames to set the velocity of the Skeleton's root
	object->getJoint(0)->setVelocities(ref.getSpatialVelocity());

	return true;
}

void MainInterface::
	mouse(int button, int state, int mx, int my)
{
	if (button == 3 || button == 4)
	{
		if (button == 3)
		{
			mCamera->Zoom(1);
		}
		else
		{
			mCamera->Zoom(-1);
		}
	}

	else if (state == GLUT_DOWN)
	{
		if (button == GLUT_LEFT_BUTTON)
			drag_mouse_l = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			drag_mouse_r = 1;
		last_mouse_x = mx;
		last_mouse_y = my;
	}

	else if (state == GLUT_UP)
	{
		drag_mouse_l = 0;
		drag_mouse_r = 0;
	}
}
void MainInterface::
	Reset()
{
	while (!perturbance.empty())
	{
		auto pLast = perturbance.back();
		// remove pfirst from the world
		mController->GetWorld()->removeSkeleton(pLast);
		perturbance.pop_back();
		perturbance_timestamp.pop_back();
	}
	this->mMotion_bvh = std::vector<Eigen::VectorXd>();
	this->mMotion_sim = std::vector<Eigen::VectorXd>();
	mTotalFrame = 0;
	mCurFrame = 0;
	mController->Reset(false);
	this->mTiming = std::vector<double>();

	step();
}

void MainInterface::
	keyboard(unsigned char key, int mx, int my)
{
	if (key == 27)
		exit(0);

	if (key == ' ')
	{
		if (!on_animation)
			on_animation = true;
		else if (on_animation)
			on_animation = false;
	}
	if (key == '1')
		this->render_bvh = (this->render_bvh == false);
	if (key == '2')
		this->render_sim = (this->render_sim == false);
	if (!on_animation){
		if (key == 'a')
			if (mCurFrame > 0) mCurFrame -= 1;
		if (key == 'd') {
			while (this->mCurFrame >= this->mTotalFrame - 1 && !this->mController->IsTerminalState())
				this->step();

			if (this->mCurFrame < this->mTotalFrame - 1){
				this->mCurFrame++;
			}
		}
	}
	if (key == 't')
		this->perturb();
	if (key == 'r')
		Reset();
}

void MainInterface::
	Timer(int value)
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	//double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()/1000000.;

	if (on_animation)
	{
		while (this->mCurFrame >= this->mTotalFrame - 1 && !this->mController->IsTerminalState())
			this->step();

		if (this->mCurFrame < this->mTotalFrame - 1){
			this->mCurFrame++;
		}

		while (!perturbance_timestamp.empty() && perturbance_timestamp[0] + 60 < this->mCurFrame){
			removeFirstPerturbance();
		}
	}

	SetFrame(this->mCurFrame);

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.;

	glutTimerFunc(std::max(0.0, mDisplayTimeout - elapsed), TimerEvent, 1);
	glutPostRedisplay();
}

void MainInterface::
	reshape(int w, int h)
{
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30, (double)w / h, 1, 300);
}