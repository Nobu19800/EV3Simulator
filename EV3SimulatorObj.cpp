/*!
* @file  EV3SimulatorObj.cpp
* @brief シミュレーション関連のクラス
*
*/

#include "EV3SimulatorObj.h"

EV3SimulatorObj *obj = NULL;


/**
*@brief シミュレーションの操作をするためのクラスのコンストラクタ
*/
EV3SimulatorObj::EV3SimulatorObj()
{
	st = 0.01;
	gravity = 9.8;
	pause = false;

	

	dInitODE();
	world        = dWorldCreate();
	space        = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	ground       = dCreatePlane(space,0,0,1,0);

	dWorldSetGravity(world, 0, 0, -9.8);
	dWorldSetCFM(world, 1e-6);
	dWorldSetERP(world, 1.0);

	makeParam();
	makeRobot();

	obj = this;
}

/**
*@brief シミュレーションの操作をするためのクラスのデストラクタ
*/
EV3SimulatorObj::~EV3SimulatorObj()
{
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
}

/**
*@brief 各パラメータの初期化を行う
*/
void EV3SimulatorObj::makeParam()
{
	EV3Block.m = DEFAULT_BODY_MASS;
	EV3Block.lx = DEFAULT_BODY_LENGTH;
	EV3Block.ly = DEFAULT_BODY_WIDTH;
	EV3Block.lz = DEFAULT_BODY_HEIGHT;
	EV3Block.x = DEFAULT_BODY_X;
	EV3Block.y = DEFAULT_BODY_Y;
	EV3Block.z = DEFAULT_BODY_Z;
	EV3Block.red = 1.;
	EV3Block.green = 1.;
	EV3Block.blue = 0.;

	wheelLeft.m = DEFAULT_WHEEL_MASS;
	wheelLeft.lx = DEFAULT_WHEEL_RADIUS;
	wheelLeft.lz = DEFAULT_WHEEL_WIDTH;
	wheelLeft.x = DEFAULT_WHEEL_X;
	wheelLeft.y = DEFAULT_WHEEL_Y;
	wheelLeft.z = DEFAULT_WHEEL_Z;
	wheelLeft.jx = wheelLeft.x;
	wheelLeft.jy = wheelLeft.y;
	wheelLeft.jz = wheelLeft.z;
	wheelLeft.axisx = 0;
	wheelLeft.axisy = 1;
	wheelLeft.axisz = 0;
	wheelLeft.red = 1.;
	wheelLeft.green = 1.;
	wheelLeft.blue = 0.;

	wheelRight.m = DEFAULT_WHEEL_MASS;
	wheelRight.lx = DEFAULT_WHEEL_RADIUS;
	wheelRight.lz = DEFAULT_WHEEL_WIDTH;
	wheelRight.x = DEFAULT_WHEEL_X;
	wheelRight.y = DEFAULT_BODY_Y - (DEFAULT_WHEEL_Y - DEFAULT_BODY_Y);
	wheelRight.z = DEFAULT_WHEEL_Z;
	wheelRight.jx = wheelRight.x;
	wheelRight.jy = wheelRight.y;
	wheelRight.jz = wheelRight.z;
	wheelRight.axisx = 0;
	wheelRight.axisy = 1;
	wheelRight.axisz = 0;
	wheelRight.red = 1.;
	wheelRight.green = 1.;
	wheelRight.blue = 0.;

	wheelBall.m = DEFAULT_BALL_MASS;
	wheelBall.lz = DEFAULT_BALL_RADIUS;
	wheelBall.x = DEFAULT_BALL_X;
	wheelBall.y = DEFAULT_BALL_Y;
	wheelBall.z = DEFAULT_BALL_Z;
	wheelBall.jx = wheelBall.x;
	wheelBall.jy = wheelBall.y;
	wheelBall.jz = wheelBall.z;
	wheelBall.red = 1.;
	wheelBall.green = 1.;
	wheelBall.blue = 0.;

	const double touchsensor_stick_length = 0.7;
	
	const double touchsensor_stick_distance = (DEFAULT_TOUCHSENSOR_X - (DEFAULT_BODY_X + DEFAULT_BODY_LENGTH / 2.0));

	touchSensorLeft[0].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorLeft[0].lx = touchsensor_stick_distance * touchsensor_stick_length;
	touchSensorLeft[0].ly = DEFAULT_STICK_WIDTH;
	touchSensorLeft[0].lz = DEFAULT_STICK_WIDTH;
	touchSensorLeft[0].x = DEFAULT_BODY_X + DEFAULT_BODY_LENGTH / 2.0 + touchSensorLeft[0].lx / 2.0;
	touchSensorLeft[0].y = DEFAULT_BODY_Y + DEFAULT_BODY_WIDTH / 2.0 + DEFAULT_STICK_WIDTH/2.0;
	touchSensorLeft[0].z = DEFAULT_BODY_Z - touchsensor_stick_distance*(1.0 - touchsensor_stick_length)*sin(DEFAULT_STICK_RADIUS);
	touchSensorLeft[0].red = 1.;
	touchSensorLeft[0].green = 1.;
	touchSensorLeft[0].blue = 0.;

	touchSensorLeft[1].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorLeft[1].lx = touchsensor_stick_distance*(1.0 - touchsensor_stick_length)/cos(DEFAULT_STICK_RADIUS);
	touchSensorLeft[1].ly = DEFAULT_STICK_WIDTH;
	touchSensorLeft[1].lz = DEFAULT_STICK_WIDTH;
	touchSensorLeft[1].x = touchSensorLeft[0].x + touchSensorLeft[0].lx / 2.0 + touchsensor_stick_distance*(1.0 - touchsensor_stick_length) / 2.0;
	touchSensorLeft[1].y = touchSensorLeft[0].y + DEFAULT_STICK_WIDTH;
	touchSensorLeft[1].z = touchSensorLeft[0].z + (DEFAULT_TOUCHSENSOR_Z - touchSensorLeft[0].z)/2.0;
	touchSensorLeft[1].red = 1.;
	touchSensorLeft[1].green = 1.;
	touchSensorLeft[1].blue = 0.;

	touchSensorLeft[2].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorLeft[2].lx = DEFAULT_TOUCHSENSOR_LENGTH;
	touchSensorLeft[2].ly = DEFAULT_TOUCHSENSOR_WIDTH;
	touchSensorLeft[2].lz = DEFAULT_TOUCHSENSOR_HEIGHT;
	touchSensorLeft[2].x = DEFAULT_TOUCHSENSOR_X;
	touchSensorLeft[2].y = DEFAULT_TOUCHSENSOR_Y;
	touchSensorLeft[2].z = DEFAULT_TOUCHSENSOR_Z;
	touchSensorLeft[2].red = 1.;
	touchSensorLeft[2].green = 1.;
	touchSensorLeft[2].blue = 0.;


	touchSensorRight[0].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorRight[0].lx = touchsensor_stick_distance * touchsensor_stick_length;
	touchSensorRight[0].ly = DEFAULT_STICK_WIDTH;
	touchSensorRight[0].lz = DEFAULT_STICK_WIDTH;
	touchSensorRight[0].x = DEFAULT_BODY_X + DEFAULT_BODY_LENGTH / 2.0 + touchSensorRight[0].lx / 2.0;
	touchSensorRight[0].y = DEFAULT_BODY_Y - DEFAULT_BODY_WIDTH / 2.0 - DEFAULT_STICK_WIDTH / 2.0;
	touchSensorRight[0].z = DEFAULT_BODY_Z - touchsensor_stick_distance*(1.0 - touchsensor_stick_length)*sin(DEFAULT_STICK_RADIUS);
	touchSensorRight[0].red = 1.;
	touchSensorRight[0].green = 1.;
	touchSensorRight[0].blue = 0.;

	touchSensorRight[1].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorRight[1].lx = touchsensor_stick_distance*(1.0 - touchsensor_stick_length) / cos(DEFAULT_STICK_RADIUS);
	touchSensorRight[1].ly = DEFAULT_STICK_WIDTH;
	touchSensorRight[1].lz = DEFAULT_STICK_WIDTH;
	touchSensorRight[1].x = touchSensorRight[0].x + touchSensorRight[0].lx / 2.0 + touchsensor_stick_distance*(1.0 - touchsensor_stick_length) / 2.0;
	touchSensorRight[1].y = touchSensorRight[0].y - DEFAULT_STICK_WIDTH;
	touchSensorRight[1].z = touchSensorRight[0].z + (DEFAULT_TOUCHSENSOR_Z - touchSensorLeft[0].z) / 2.0;
	touchSensorRight[1].red = 1.;
	touchSensorRight[1].green = 1.;
	touchSensorRight[1].blue = 0.;

	touchSensorRight[2].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorRight[2].lx = DEFAULT_TOUCHSENSOR_LENGTH;
	touchSensorRight[2].ly = DEFAULT_TOUCHSENSOR_WIDTH;
	touchSensorRight[2].lz = DEFAULT_TOUCHSENSOR_HEIGHT;
	touchSensorRight[2].x = DEFAULT_TOUCHSENSOR_X;
	touchSensorRight[2].y = DEFAULT_BODY_Y - (DEFAULT_TOUCHSENSOR_Y - DEFAULT_BODY_Y);
	touchSensorRight[2].z = DEFAULT_TOUCHSENSOR_Z;
	touchSensorRight[2].red = 1.;
	touchSensorRight[2].green = 1.;
	touchSensorRight[2].blue = 0.;

	touchSensorLeft[3].m = DEFAULT_TOUCHSENSOR2_MASS;
	touchSensorLeft[3].lx = DEFAULT_TOUCHSENSOR2_LENGTH;
	touchSensorLeft[3].ly = DEFAULT_TOUCHSENSOR2_WIDTH;
	touchSensorLeft[3].lz = DEFAULT_TOUCHSENSOR2_HEIGHT;
	touchSensorLeft[3].x = DEFAULT_TOUCHSENSOR2_X;
	touchSensorLeft[3].y = DEFAULT_TOUCHSENSOR2_Y;
	touchSensorLeft[3].z = DEFAULT_TOUCHSENSOR2_Z;
	touchSensorLeft[3].axisx = 1;
	touchSensorLeft[3].axisy = 0;
	touchSensorLeft[3].axisz = 0;
	touchSensorLeft[3].red = 1.;
	touchSensorLeft[3].green = 1.;
	touchSensorLeft[3].blue = 0.;

	touchSensorRight[3].m = DEFAULT_TOUCHSENSOR2_MASS;
	touchSensorRight[3].lx = DEFAULT_TOUCHSENSOR2_LENGTH;
	touchSensorRight[3].ly = DEFAULT_TOUCHSENSOR2_WIDTH;
	touchSensorRight[3].lz = DEFAULT_TOUCHSENSOR2_HEIGHT;
	touchSensorRight[3].x = DEFAULT_TOUCHSENSOR2_X;
	touchSensorRight[3].y = DEFAULT_BODY_Y - (DEFAULT_TOUCHSENSOR2_Y - DEFAULT_BODY_Y);
	touchSensorRight[3].z = DEFAULT_TOUCHSENSOR2_Z;
	touchSensorRight[3].axisx = 1;
	touchSensorRight[3].axisy = 0;
	touchSensorRight[3].axisz = 0;
	touchSensorRight[3].red = 1.;
	touchSensorRight[3].green = 1.;
	touchSensorRight[3].blue = 0.;


	gyroSensor.m = DEFAULT_GYROSENSOR_MASS;
	gyroSensor.lx = DEFAULT_GYROSENSOR_LENGTH;
	gyroSensor.ly = DEFAULT_GYROSENSOR_WIDTH;
	gyroSensor.lz = DEFAULT_GYROSENSOR_HEIGHT;
	gyroSensor.x = DEFAULT_GYROSENSOR_X;
	gyroSensor.y = DEFAULT_GYROSENSOR_Y;
	gyroSensor.z = DEFAULT_GYROSENSOR_Z;
	gyroSensor.red = 1.;
	gyroSensor.green = 1.;
	gyroSensor.blue = 0.;


	colourSensor.m = DEFAULT_COLOURSENSOR_MASS;
	colourSensor.lx = DEFAULT_COLOURSENSOR_LENGTH;
	colourSensor.ly = DEFAULT_COLOURSENSOR_WIDTH;
	colourSensor.lz = DEFAULT_COLOURSENSOR_HEIGHT;
	colourSensor.x = DEFAULT_COLOURSENSOR_X;
	colourSensor.y = DEFAULT_COLOURSENSOR_Y;
	colourSensor.z = DEFAULT_COLOURSENSOR_Z;
	colourSensor.red = 1.;
	colourSensor.green = 1.;
	colourSensor.blue = 0.;

	mmotor.m = DEFAULT_MMOTOR_MASS;
	mmotor.lx = DEFAULT_MMOTOR_LENGTH;
	mmotor.ly = DEFAULT_MMOTOR_WIDTH;
	mmotor.lz = DEFAULT_MMOTOR_HEIGHT;
	mmotor.x = DEFAULT_MMOTOR_X;
	mmotor.y = DEFAULT_MMOTOR_Y;
	mmotor.z = DEFAULT_MMOTOR_Z;
	mmotor.red = 1.;
	mmotor.green = 1.;
	mmotor.blue = 0.;

	
}

/**
*@brief 直方体作成
* @param body ボディオブジェクト
*/
void EV3SimulatorObj::setBox(MyLink *body)
{
	dMass mass;
	body->body  = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass,body->m , body->lx, body->ly, body->lz);
	dBodySetMass(body->body,&mass);
	body->geom = dCreateBox(space,body->lx, body->ly, body->lz);
	dGeomSetBody(body->geom, body->body);
	dBodySetPosition(body->body, body->x, body->y, body->z);
}

/**
*@brief 円柱作成
* @param body ボディオブジェクト
*/
void EV3SimulatorObj::setCylinder(MyLink *body)
{
	dMass mass;
	body->body  = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass,body->m , 2, body->lx,  body->lz);
	dBodySetMass(body->body,&mass);
	body->geom = dCreateCylinder(space,body->lx, body->lz);
	dGeomSetBody(body->geom, body->body);
	dBodySetPosition(body->body, body->x, body->y, body->z);
}

/**
*@brief 球作成
* @param body ボディオブジェクト
*/
void EV3SimulatorObj::setSphere(MyLink *body)
{
	dMass mass;
	body->body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetSphereTotal(&mass, body->m, body->lz);
	dBodySetMass(body->body, &mass);
	body->geom = dCreateSphere(space, body->lz);
	dGeomSetBody(body->geom, body->body);
	dBodySetPosition(body->body, body->x, body->y, body->z);
}

/**
*@brief ヒンジジョイント作成
* @param body1 ボディ1
* @param body2 ボディ2
*/
void EV3SimulatorObj::setHinge(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateHinge(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	dJointSetHingeAnchor(body1->joint, body1->jx, body1->jy, body1->jz);
	dJointSetHingeAxis(body1->joint, body1->axisx, body1->axisy,body1->axisz);
}

/**
*@brief スライダージョイント作成
* @param body1 ボディ1
* @param body2 ボディ2
*/
void EV3SimulatorObj::setSlider(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateSlider(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	
	dJointSetSliderAxis(body1->joint, body1->axisx, body1->axisy,body1->axisz);
}

/**
*@brief 固定ジョイント作成
* @param body1 ボディ1
* @param body2 ボディ2
*/
void EV3SimulatorObj::setFixed(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateFixed(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	dJointSetFixed(body1->joint);
}


/**
*@brief ボールジョイント作成
* @param body1 ボディ1
* @param body2 ボディ2
*/
void EV3SimulatorObj::setBall(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateBall(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	dJointSetBallAnchor(body1->joint, body1->jx, body1->jy, body1->jz);
	//dJointSetBallAnchor2(body1->joint, body1->jx, body1->jy, body1->jz);
}

/**
*@brief 全ボディ、接続する全ジョイント生成
*/
void EV3SimulatorObj::makeRobot()
{
	mu.lock();
	setBox(&EV3Block);
	setCylinder(&wheelLeft);
	setCylinder(&wheelRight);
	setSphere(&wheelBall);
	setBox(&touchSensorLeft[0]);
	setBox(&touchSensorLeft[1]);
	setBox(&touchSensorLeft[2]);
	setBox(&touchSensorRight[0]);
	setBox(&touchSensorRight[1]);
	setBox(&touchSensorRight[2]);
	setBox(&touchSensorLeft[3]);
	setBox(&touchSensorRight[3]);
	setBox(&gyroSensor);
	setBox(&colourSensor);
	setBox(&mmotor);

	dMatrix3 R;
	dRFromAxisAndAngle(R, 1, 0, 0, M_PI/2);
	dGeomSetRotation(wheelLeft.geom, R);
	
	dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
	dGeomSetRotation(wheelRight.geom, R);

	dRFromAxisAndAngle(R, 0, 1, 0, -DEFAULT_STICK_RADIUS);
	dGeomSetRotation(touchSensorLeft[1].geom, R);
	dRFromAxisAndAngle(R, 0, 1, 0, -DEFAULT_STICK_RADIUS);
	dGeomSetRotation(touchSensorRight[1].geom, R);


	EV3Block.joint = dJointCreateFixed(world, 0);
	dJointAttach(EV3Block.joint, EV3Block.body, 0);
	dJointSetFixed(EV3Block.joint);

	setHinge(&wheelLeft, &EV3Block);
	setHinge(&wheelRight, &EV3Block);
	setBall(&wheelBall, &EV3Block);
	setFixed(&touchSensorLeft[0], &EV3Block);
	setFixed(&touchSensorLeft[1], &touchSensorLeft[0]);
	setFixed(&touchSensorLeft[2], &touchSensorLeft[1]);
	setFixed(&touchSensorRight[0], &EV3Block);
	setFixed(&touchSensorRight[1], &touchSensorRight[0]);
	setFixed(&touchSensorRight[2], &EV3Block);
	



	setSlider(&touchSensorLeft[3], &touchSensorLeft[2]);
	setSlider(&touchSensorRight[3], &touchSensorRight[2]);
	setFixed(&gyroSensor, &EV3Block);
	setFixed(&colourSensor, &EV3Block);
	

	double KP = DEFAULT_SPRINFG_VALUE;
	double KD = 0;
	double erp = st*KP / (st*KP + KD);
	double cfm = 1 / (st*KP + KD);

	dJointSetSliderParam(touchSensorLeft[3].joint, dParamLoStop, -0);
	dJointSetSliderParam(touchSensorLeft[3].joint, dParamHiStop, 0);
	dJointSetSliderParam(touchSensorLeft[3].joint, dParamStopERP, erp);
	dJointSetSliderParam(touchSensorLeft[3].joint, dParamStopCFM, cfm);

	dJointSetSliderParam(touchSensorRight[3].joint, dParamLoStop, -0);
	dJointSetSliderParam(touchSensorRight[3].joint, dParamHiStop, 0);
	dJointSetSliderParam(touchSensorRight[3].joint, dParamStopERP, erp);
	dJointSetSliderParam(touchSensorRight[3].joint, dParamStopCFM, cfm);

	setFixed(&mmotor, &EV3Block);
	
	pause = true;

	mu.unlock();

	
}

/**
*@brief 接触コールバック
* @param o1 ジオメトリ1
* @param o2 ジオメトリ2
*/
void EV3SimulatorObj::m_nearCallback(dGeomID o1, dGeomID o2)
{
	//return;
	
  dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
  if (b1 && b2 && dAreConnected(b1,b2)) return;
  if ((o1 != ground) && (o2 != ground)) return;
 
   static const int N = 20;
	dContact contact[N];
	int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

	

	if (n > 0) {
		for (int i=0; i<n; i++) {
			contact[i].surface.mode = dContactApprox1|dContactSoftERP|dContactSoftCFM|dContactSlip1|dContactSlip2;



			contact[i].surface.mu   = 0.5;
			contact[i].surface.slip1 = 0.001;
			contact[i].surface.slip2 = 0.001;
			contact[i].surface.soft_erp = 0.3;
			contact[i].surface.soft_cfm = 1e-4;
			dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
			dJointAttach(c,b1,b2);
		}
	}
}

/**
*@brief 接触コールバック
* @param data データ
* @param o1 ジオメトリ1
* @param o2 ジオメトリ2
*/
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
	
	if(obj)
	{
		obj->m_nearCallback(o1, o2);
	}
  
		
}

/**
*@brief ヒンジジョイント制御
* @param body ボディオブジェクト
* @param theta ヒンジジョイントの位置
*/
void EV3SimulatorObj::controlHinge(MyLink *body, dReal theta)
{
	dReal kp = 100;
	dReal tmp = dJointGetHingeAngle(body->joint);
	dReal diff = theta - tmp;
	dReal u = kp * diff;

	dJointSetHingeParam(body->joint,dParamVel, u);
	dJointSetHingeParam(body->joint,dParamFMax,20.);
}

/**
*@brief スライダージョイント制御
* @param body ボディオブジェクト
* @param length スライダージョイントの位置
*/
void EV3SimulatorObj::controlSlider(MyLink *body, dReal length)
{
	dReal kp = 10;
	dReal tmp = dJointGetSliderPosition(body->joint);
	dReal diff = length - tmp;
	dReal u = kp * diff;

	dJointSetSliderParam (body->joint,dParamVel, u);
	dJointSetSliderParam (body->joint,dParamFMax,20.);
}

/**
*@brief 全ジョイント制御
*/
void EV3SimulatorObj::control()
{
	/*

	dReal r2 = dJointGetHingeAngle (link2.joint);


	controlHinge(&link1, -rb->theta[0]+rb->offset[0]);//

	*/
}

/**
*@brief 更新
*/
void EV3SimulatorObj::update()
{
	if(pause)
	{
		mu.lock();
		control(); 
		dSpaceCollide(space,0,&nearCallback);
		dWorldStep(world, st);
		dJointGroupEmpty(contactgroup);
		mu.unlock();

	}
}

/**
*@brief 全ボディ、接続する全ジョイント消去
*/
void EV3SimulatorObj::destroyRobot()
{
	mu.lock();
	pause = false;
	//dJointDestroy(EV3Block.joint);
	dJointDestroy(wheelLeft.joint);
	dJointDestroy(wheelRight.joint);
	dJointDestroy(wheelBall.joint);


	dBodyDestroy(EV3Block.body);
	dBodyDestroy(wheelLeft.body);
	dBodyDestroy(wheelRight.body);
	dBodyDestroy(wheelBall.body);


	mu.unlock();
}