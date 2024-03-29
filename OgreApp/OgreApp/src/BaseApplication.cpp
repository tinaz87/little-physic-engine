
#include "BaseApplication.h"


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#include "../res/resource.h"
#endif

/*
*	Cause performance problems max_object is limited to 9.
*	
*	Necessary to OPTIMIZE collision detection
*/
#define MAX_OBJECT 9


//-------------------------------------------------------------------------------------
BaseApplication::BaseApplication(void)
    : mRoot(0),
    mCamera(0),
    mSceneMgr(0),
    mWindow(0),
    mResourcesCfg(Ogre::StringUtil::BLANK),
    mPluginsCfg(Ogre::StringUtil::BLANK),
    mTrayMgr(0),
    mCameraMan(0),
    mDetailsPanel(0),
    mCursorWasVisible(false),
    mShutDown(false),
    mInputManager(0),
    mMouse(0),
	mKeyboard(0),resolver(maxContacts*8),prevTime(0),ballEnt(NULL),	ballVelocitySlider(NULL),
	boxVelocitySlider(NULL)
{
	boxes.reserve(MAX_OBJECT);
	balls.reserve(MAX_OBJECT);

	tweak = true;
}

void BaseApplication::clearAll(){

	for(ballsIterator it= balls.begin();it!=balls.end();++it){

		delete (*it)->body;
		delete *it;
	}

	for(sceneNodeIterator it= ballsNode.begin();it!=ballsNode.end();++it){
	
		mSceneMgr->getRootSceneNode()->removeChild(*it);
	}

	balls.clear();

	for(boxesIterator it= boxes.begin();it!=boxes.end();++it){

		delete (*it)->body;
		delete *it;
	}

	for(sceneNodeIterator it= boxesNode.begin();it!=boxesNode.end();++it){

		mSceneMgr->getRootSceneNode()->removeChild(*it);
	}

	boxes.clear();

	ballsNode.clear();
	boxesNode.clear();
}


//-------------------------------------------------------------------------------------
BaseApplication::~BaseApplication(void)
{

	clearAll();

    if (mTrayMgr) delete mTrayMgr;
    if (mCameraMan) delete mCameraMan;

    //Remove ourself as a Window listener
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    windowClosed(mWindow);
    delete mRoot;
}

//-------------------------------------------------------------------------------------
bool BaseApplication::configure(void)
{
    // Show the configuration dialog and initialise the system
    // You can skip this and use root.restoreConfig() to load configuration
    // settings if you were sure there are valid ones saved in ogre.cfg
    if(mRoot->showConfigDialog())
    {
        // If returned true, user clicked OK so initialise
        // Here we choose to let the system create a default rendering window by passing 'true'
        mWindow = mRoot->initialise(true, "OgreApp Render Window");

        // Let's add a nice window icon
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        HWND hwnd;
        mWindow->getCustomAttribute("WINDOW", (void*)&hwnd);
        LONG iconID   = (LONG)LoadIcon( GetModuleHandle(0), MAKEINTRESOURCE(IDI_APPICON) );
        SetClassLong( hwnd, GCL_HICON, iconID );
#endif
        return true;
    }
    else
    {
        return false;
    }
}
//-------------------------------------------------------------------------------------
void BaseApplication::chooseSceneManager(void)
{
    // Get the SceneManager, in this case a generic one
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
}
//-------------------------------------------------------------------------------------
void BaseApplication::createCamera(void)
{
    // Create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");

	
    // Position it at 500 in Z direction
    mCamera->setPosition(Ogre::Vector3(0,100,80));
    // Look back along -Z
    mCamera->lookAt(Ogre::Vector3(0,100,0));
    mCamera->setNearClipDistance(5);

    mCameraMan = new OgreBites::SdkCameraMan(mCamera);   // create a default camera controller
}
//-------------------------------------------------------------------------------------
void BaseApplication::createFrameListener(void)
{
    Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
    OIS::ParamList pl;
    size_t windowHnd = 0;
    std::ostringstream windowHndStr;

    mWindow->getCustomAttribute("WINDOW", &windowHnd);
    windowHndStr << windowHnd;
    pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

    mInputManager = OIS::InputManager::createInputSystem( pl );

    mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, true ));
    mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, true ));

    mMouse->setEventCallback(this);
    mKeyboard->setEventCallback(this);

    //Set initial mouse clipping size
    windowResized(mWindow);

    //Register as a Window listener
    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

    mTrayMgr = new OgreBites::SdkTrayManager("InterfaceName", mWindow, mMouse, this);
    mTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
	mTrayMgr->showLogo(OgreBites::TL_BOTTOMRIGHT);
    //mTrayMgr->hideCursor();

	Ogre::DisplayString s(" - Press SPACE To Launch a Sphere\n - Press X To Launch a Box		     \n - Press CANC to Reset				      ");
	label = mTrayMgr->createLabel(OgreBites::TL_TOPLEFT,"InfoLabel",s,300);
	mTrayMgr->moveWidgetToTray(label, OgreBites::TL_TOPLEFT, 0);
	label->show();

	ballVelocitySlider = mTrayMgr->createThickSlider(OgreBites::TL_LEFT,"SphereVelSlider","Ball Starting Velocity",250,60,25,200,176);
	ballVelocitySlider->setValue(25);
	mTrayMgr->moveWidgetToTray(ballVelocitySlider, OgreBites::TL_LEFT, 0);
	ballVelocitySlider->show();

	boxVelocitySlider = mTrayMgr->createThickSlider(OgreBites::TL_LEFT,"BoxVelSlider","Box Starting Velocity",250,60,25,200,176);
	ballVelocitySlider->setValue(25);
	mTrayMgr->moveWidgetToTray(ballVelocitySlider, OgreBites::TL_LEFT, 0);
	ballVelocitySlider->show();
	
	// create a params panel for displaying sample details
    Ogre::StringVector items;
    items.push_back("cam.pX");
    items.push_back("cam.pY");
    items.push_back("cam.pZ");
    items.push_back("");
    items.push_back("cam.oW");
    items.push_back("cam.oX");
    items.push_back("cam.oY");
    items.push_back("cam.oZ");
    items.push_back("");
    items.push_back("Filtering");
    items.push_back("Poly Mode");

    mDetailsPanel = mTrayMgr->createParamsPanel(OgreBites::TL_NONE, "DetailsPanel", 200, items);
    mDetailsPanel->setParamValue(9, "Bilinear");
    mDetailsPanel->setParamValue(10, "Solid");
    mDetailsPanel->hide();

    mRoot->addFrameListener(this);
}
//-------------------------------------------------------------------------------------
void BaseApplication::destroyScene(void)
{
}
//-------------------------------------------------------------------------------------
void BaseApplication::createViewports(void)
{
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0,0,0));

    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(
        Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}
//-------------------------------------------------------------------------------------
void BaseApplication::setupResources(void)
{
    // Load resource paths from config file
    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                archName, typeName, secName);
        }
    }
}
//-------------------------------------------------------------------------------------
void BaseApplication::createResourceListener(void)
{

}
//-------------------------------------------------------------------------------------
void BaseApplication::loadResources(void)
{
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}
//-------------------------------------------------------------------------------------
void BaseApplication::go(void)
{
#ifdef _DEBUG
    mResourcesCfg = "resources_d.cfg";
    mPluginsCfg = "plugins_d.cfg";
#else
    mResourcesCfg = "resources.cfg";
    mPluginsCfg = "plugins.cfg";
#endif

    if (!setup())
        return;
	showWin32Console();

    mRoot->startRendering();

    // clean up
    destroyScene();
}
//-------------------------------------------------------------------------------------
bool BaseApplication::setup(void)
{
    mRoot = new Ogre::Root(mPluginsCfg);

    setupResources();

    bool carryOn = configure();
    if (!carryOn) return false;

    chooseSceneManager();
    createCamera();
    createViewports();

    // Set default mipmap level (NB some APIs ignore this)
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

    // Create any resource listeners (for loading screens)
    createResourceListener();
    // Load resources
    loadResources();

	cData.contactArray = contacts;
	
	plane.direction = Vector3(0,1,0);
	plane.offset = 0;
	
	// Create the scene
	createScene();
	
	generateBall(Vector3(0,0,0),Vector3(0,100,0));
	generateBox(Vector3(0,0,0),Vector3(30,100,0));


    createFrameListener();
    
	return true;
};

void BaseApplication::generateBall(Vector3& velocity,Vector3& position){

	// Physic
	if (balls.size()<MAX_OBJECT)
	{
	
		CollisionSphere* ball = new CollisionSphere();
		ball->body = new RigidBody();
		ball->radius = 3.8f;
		ball->body->setMass(20);
		ball->body->setPosition(position);
		ball->body->setRotation(Vector3(0,0,0));
	
		ball->body->setAcceleration(Vector3::GRAVITY * 6);

		ball->body->setVelocity(velocity);	
		
		Ogre::Vector3 vu = -mCamera->getUp();
		Ogre::Vector3 vr = mCamera->getRight();

		Ogre::Vector3 v1 = vu.crossProduct(vr);

		physic::Vector3 velocity(v1.x ,v1.y,v1.z );
		ball->body->addRotation(velocity * 20);

		ball->body->setAwake(true);

		ball->body->setDamping(0.99f,0.55f);
		ball->body->clearAccumulators();


		Matrix3 tensor;
		float coeff = 0.4f*ball->body->getMass()*ball->radius;
		tensor.SetInertiaTensorCoeffs(coeff,coeff,coeff);
		ball->body->setInertiaTensor(tensor);

		ball->body->setCanSleep(true);

		physic::SetSleepEpsilon(2.0f);
		physic::SetIntegrationStep(0.0015f);

		// Clear the force accumulators
		ball->body->calculateDerivedData();
		ball->calculateInternals();

		balls.push_back(ball);

		// Graphics

		ballEnt = mSceneMgr->createEntity("pokeball_dae.mesh");

		Ogre::SceneNode* ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		ballNode->scale(0.02f,0.02f,0.02f);
		Ogre::Vector3 v(ball->body->getPosition().x,ball->body->getPosition().y,ball->body->getPosition().z);
		ballNode->setPosition(v);
		ballNode->attachObject(ballEnt);

		ballsNode.push_back(ballNode);
	}
}


void BaseApplication::generateBox(Vector3& velocity,Vector3& position){

	// Physic
	if (boxes.size()<MAX_OBJECT)
	{

		CollisionBox* box = new CollisionBox();

		box->halfSize = Vector3(11,11,11);
		box->body = new RigidBody();
		
		box->body->setMass(20);
		box->body->setPosition(position);
		box->body->setRotation(Vector3(0,0,0));

		box->body->setAcceleration(Vector3::GRAVITY * 6);

		box->body->setVelocity(velocity);	

		box->body->addRotation(Vector3(0,0,0));

		box->body->setAwake(true);

		box->body->setDamping(0.99f,0.7f);
		box->body->clearAccumulators();


		Matrix3 tensor;
		
		tensor.SetBlockInertiaTensor(box->halfSize, box->body->getMass());

		box->body->setInertiaTensor(tensor);

		box->body->setCanSleep(true);

		physic::SetSleepEpsilon(1.0f);
		physic::SetIntegrationStep(0.0019f);

		// Clear the force accumulators
		box->body->calculateDerivedData();
		box->calculateInternals();

		boxes.push_back(box);

		// Graphics

		ballEnt = mSceneMgr->createEntity("cube.mesh");

		Ogre::SceneNode* boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		boxNode->scale(0.2f,0.2f,0.2f);
		Ogre::Vector3 v(box->body->getPosition().x,box->body->getPosition().y,box->body->getPosition().z);
		boxNode->setPosition(v);
		boxNode->attachObject(ballEnt);
		ballEnt->setMaterialName("Box/Grass");

		boxesNode.push_back(boxNode);
	}
}
//-------------------------------------------------------------------------------------
bool BaseApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if(mWindow->isClosed())
        return false;

    if(mShutDown)
        return false;

    //Need to capture/update each device
    mKeyboard->capture();
    mMouse->capture();

    mTrayMgr->frameRenderingQueued(evt);

    if (!mTrayMgr->isDialogVisible())
    {
        mCameraMan->frameRenderingQueued(evt);   // if dialog isn't up, then update the camera
        if (mDetailsPanel->isVisible())   // if details panel is visible, then update its contents
        {
            mDetailsPanel->setParamValue(0, Ogre::StringConverter::toString(mCamera->getDerivedPosition().x));
            mDetailsPanel->setParamValue(1, Ogre::StringConverter::toString(mCamera->getDerivedPosition().y));
            mDetailsPanel->setParamValue(2, Ogre::StringConverter::toString(mCamera->getDerivedPosition().z));
            mDetailsPanel->setParamValue(4, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().w));
            mDetailsPanel->setParamValue(5, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().x));
            mDetailsPanel->setParamValue(6, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().y));
            mDetailsPanel->setParamValue(7, Ogre::StringConverter::toString(mCamera->getDerivedOrientation().z));
        }
    }
	
	if (tweak)
	{
		mTrayMgr->toggleAdvancedFrameStats();
		tweak = false;
	}

		
	updateObject(evt.timeSinceLastFrame);
	
	generateContacts(evt.timeSinceLastFrame);

	for (unsigned i=0;i<balls.size();++i)
	{
		Ogre::Vector3 v(balls.at(i)->body->getPosition().x,balls.at(i)->body->getPosition().y,balls.at(i)->body->getPosition().z);
		ballsNode.at(i)->setPosition(v);

		Ogre::Quaternion q(balls.at(i)->body->getOrientation().r,balls.at(i)->body->getOrientation().i,balls.at(i)->body->getOrientation().j,balls.at(i)->body->getOrientation().k);
		ballsNode.at(i)->setOrientation(q);
	
	}

	for (unsigned i=0;i<boxes.size();++i)
	{
		Ogre::Vector3 v(boxes.at(i)->body->getPosition().x,boxes.at(i)->body->getPosition().y,boxes.at(i)->body->getPosition().z);
		boxesNode.at(i)->setPosition(v);

		Ogre::Quaternion q(boxes.at(i)->body->getOrientation().r,boxes.at(i)->body->getOrientation().i,boxes.at(i)->body->getOrientation().j,boxes.at(i)->body->getOrientation().k);
		boxesNode.at(i)->setOrientation(q);

	}

    return true;
}
//-------------------------------------------------------------------------------------
bool BaseApplication::keyPressed( const OIS::KeyEvent &arg )
{
    if (mTrayMgr->isDialogVisible()) return true;   // don't process any more keys if dialog is up

	 if (arg.key == OIS::KC_SPACE)   // toggle visibility of advanced frame stats
    {
		Ogre::Vector3 v = mCamera->getDirection().normalisedCopy();
		physic::Vector3 velocity(v.x ,v.y,v.z );
		Ogre::Vector3 p = mCamera->getPosition();
		physic::Vector3 position(p.x + v.x*30,p.y + v.y*30,p.z + v.z*30);
		int vel = ballVelocitySlider->getValue();
 		generateBall(velocity*( vel ),position);
    }

	 if (arg.key == OIS::KC_X)   // toggle visibility of advanced frame stats
	 {
		 Ogre::Vector3 v = mCamera->getDirection().normalisedCopy();
		 physic::Vector3 velocity(v.x,v.y,v.z);
		 Ogre::Vector3 p = mCamera->getPosition();
		 physic::Vector3 position(p.x + v.x*50,p.y + v.y*50,p.z + v.z*50);
		 int vel = boxVelocitySlider->getValue();
		 generateBox(velocity*( vel ),position);
	 }

	 if (arg.key == OIS::KC_DELETE)   // toggle visibility of advanced frame stats
	 {
		clearAll();
	 }

    if (arg.key == OIS::KC_F)   // toggle visibility of advanced frame stats
    {
        mTrayMgr->toggleAdvancedFrameStats();
    }
    else if (arg.key == OIS::KC_G)   // toggle visibility of even rarer debugging details
    {
        if (mDetailsPanel->getTrayLocation() == OgreBites::TL_NONE)
        {
            mTrayMgr->moveWidgetToTray(mDetailsPanel, OgreBites::TL_TOPRIGHT, 0);
            mDetailsPanel->show();
        }
        else
        {
            mTrayMgr->removeWidgetFromTray(mDetailsPanel);
            mDetailsPanel->hide();
        }
    }
    else if (arg.key == OIS::KC_T)   // cycle polygon rendering mode
    {
        Ogre::String newVal;
        Ogre::TextureFilterOptions tfo;
        unsigned int aniso;

        switch (mDetailsPanel->getParamValue(9).asUTF8()[0])
        {
        case 'B':
            newVal = "Trilinear";
            tfo = Ogre::TFO_TRILINEAR;
            aniso = 1;
            break;
        case 'T':
            newVal = "Anisotropic";
            tfo = Ogre::TFO_ANISOTROPIC;
            aniso = 8;
            break;
        case 'A':
            newVal = "None";
            tfo = Ogre::TFO_NONE;
            aniso = 1;
            break;
        default:
            newVal = "Bilinear";
            tfo = Ogre::TFO_BILINEAR;
            aniso = 1;
        }

        Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(tfo);
        Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(aniso);
        mDetailsPanel->setParamValue(9, newVal);
    }
    else if (arg.key == OIS::KC_R)   // cycle polygon rendering mode
    {
        Ogre::String newVal;
        Ogre::PolygonMode pm;

        switch (mCamera->getPolygonMode())
        {
        case Ogre::PM_SOLID:
            newVal = "Wireframe";
            pm = Ogre::PM_WIREFRAME;
            break;
        case Ogre::PM_WIREFRAME:
            newVal = "Points";
            pm = Ogre::PM_POINTS;
            break;
        default:
            newVal = "Solid";
            pm = Ogre::PM_SOLID;
        }

        mCamera->setPolygonMode(pm);
        mDetailsPanel->setParamValue(10, newVal);
    }
    else if(arg.key == OIS::KC_F5)   // refresh all textures
    {
        Ogre::TextureManager::getSingleton().reloadAll();
    }
    else if (arg.key == OIS::KC_SYSRQ)   // take a screenshot
    {
        mWindow->writeContentsToTimestampedFile("screenshot", ".jpg");
    }
    else if (arg.key == OIS::KC_ESCAPE)
    {
        mShutDown = true;
    }

    mCameraMan->injectKeyDown(arg);
    return true;
}

bool BaseApplication::keyReleased( const OIS::KeyEvent &arg )
{
    mCameraMan->injectKeyUp(arg);
    return true;
}

bool BaseApplication::mouseMoved( const OIS::MouseEvent &arg )
{
    if (mTrayMgr->injectMouseMove(arg)) return true;
    mCameraMan->injectMouseMove(arg);
    return true;
}

bool BaseApplication::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    if (mTrayMgr->injectMouseDown(arg, id)) return true;
    mCameraMan->injectMouseDown(arg, id);
    return true;
}

bool BaseApplication::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    if (mTrayMgr->injectMouseUp(arg, id)) return true;
    mCameraMan->injectMouseUp(arg, id);
    return true;
}

//Adjust mouse clipping area
void BaseApplication::windowResized(Ogre::RenderWindow* rw)
{
    unsigned int width, height, depth;
    int left, top;
    rw->getMetrics(width, height, depth, left, top);

    const OIS::MouseState &ms = mMouse->getMouseState();
    ms.width = width;
    ms.height = height;
}

//Unattach OIS before window shutdown (very important under Linux)
void BaseApplication::windowClosed(Ogre::RenderWindow* rw)
{
    //Only close for window that created OIS (the main window in these demos)
    if( rw == mWindow )
    {
        if( mInputManager )
        {
            mInputManager->destroyInputObject( mMouse );
            mInputManager->destroyInputObject( mKeyboard );

            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = 0;
        }
    }
}

void BaseApplication::createScene(void)
{
	
	// Set ambient light
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

	// Create a light
	Ogre::Light* l = mSceneMgr->createLight("MainLight");
	l->setPosition(20,80,50);

	Ogre::Plane plane(Ogre::Vector3::UNIT_Y,0);

	Ogre::MeshManager::getSingleton().createPlane("plane",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,plane,
		1500,1500,200,200,true,1,5,5,Ogre::Vector3::UNIT_Z);

	Ogre::Entity* ent= mSceneMgr->createEntity("LightPlaneEntity","plane");

	mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(ent);

	ent->setMaterialName("Examples/BeachStones");

	Ogre::Light* light2= mSceneMgr->createLight("Light2");

	light2->setType(Ogre::Light::LT_DIRECTIONAL);

	light2->setDirection(Ogre::Vector3(1,-1,0));

}


void BaseApplication::updateObject(float dt){

	float elapsedTime = prevTime + dt;
	while (elapsedTime > GetIntegrationStep() )
	{
		for(ballsIterator it= balls.begin();it!=balls.end();++it){

			(*it)->body->integrate(GetIntegrationStep());
			(*it)->calculateInternals();
		}

		for(boxesIterator it= boxes.begin();it!=boxes.end();++it){

			(*it)->body->integrate(GetIntegrationStep());
			(*it)->calculateInternals();
		}


		elapsedTime -= GetIntegrationStep();
	}
	
	prevTime = elapsedTime;
}

void BaseApplication::generateContacts(float dt){

	/*
		Balls-Plane Collision
	*/
	for(ballsIterator it= balls.begin();it!=balls.end();++it){

		CollisionSphere* ball = *it;
		
		// Set up the collision data structure
		cData.reset(256U);
		cData.friction = (float)0.9;
		cData.restitution = (float)0.6;
		cData.tolerance = (float)0.2;

		unsigned contacts = 0;
	
		if (!cData.hasMoreContacts()) return;

		if(CollisionDetector::sphereAndTruePlane(*ball,plane,&cData)){
			++contacts;
		}

		resolver.setIterations(contacts * 4);
		resolver.resolveContacts(cData.contactArray, cData.contactCount, dt);
	}

	/*
		Boxes-Plane Collision
	*/
	for(boxesIterator it= boxes.begin();it!=boxes.end();++it){

		CollisionBox* box = *it;

		// Set up the collision data structure
		cData.reset(256U);
		cData.friction = (float)0.9;
		cData.restitution = (float)0.1;
		cData.tolerance = (float)0.1;

		unsigned contacts = 0;

		if (!cData.hasMoreContacts()) return;

		if(CollisionDetector::boxAndHalfSpace(*box,plane,&cData)){
			++contacts;
		}

		resolver.setIterations(contacts * 4);
		resolver.resolveContacts(cData.contactArray, cData.contactCount, dt);
	}

	/*
		Ball-BAll Collision
	*/
	for(ballsIterator it= balls.begin();it!=balls.end();++it){

		CollisionSphere* ball = *it;
		
		// Set up the collision data structure
		cData.reset(256U);
		cData.friction = (float)0.9;
		cData.restitution = (float)0.6;
		cData.tolerance = (float)0.2;		
	
		if (!cData.hasMoreContacts()) return;

		for(ballsIterator it2= balls.begin();it2!=balls.end();++it2){

			unsigned contacts = 0;

			if (it != it2)
			{
			
				if(CollisionDetector::sphereAndSphere(*ball,*(*it2),&cData)){
					++contacts;
				}

				resolver.setIterations(contacts * 4);
				resolver.resolveContacts(cData.contactArray, cData.contactCount, dt);
			}
		}
	}

	/*
		Box-Ball Collision
	*/
	for(ballsIterator it= balls.begin();it!=balls.end();++it){

		CollisionSphere* ball = *it;

		// Set up the collision data structure
		cData.reset(256U);
		cData.friction = (float)0.9;
		cData.restitution = (float)0.5;
		cData.tolerance = (float)0.2;		

		if (!cData.hasMoreContacts()) return;

		for(boxesIterator it2= boxes.begin();it2!=boxes.end();++it2){

			unsigned contacts = 0;

			if(CollisionDetector::boxAndSphere(*(*it2),*(*it),&cData)){
				++contacts;
			}

			resolver.setIterations(contacts * 4);
			resolver.resolveContacts(cData.contactArray, cData.contactCount, dt);

		}
	}

	

}