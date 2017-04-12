/*!
* @file  EV3SimulatorObj.cpp
* @brief シミュレーション関連のクラス
*
*/

#include <iostream>
#include "EV3SimulatorObj.h"
#include <fstream>
#include <coil/stringutil.h>

EV3SimulatorObj *obj = NULL;

EV3Obj::EV3Obj() :
target_vx(0),
target_va(0),
current_px(0),
current_py(0),
current_pa(0),
current_ultrasonicSensorData(0),
current_colourSensorData(0),
target_mangle(0),
right_touch(false),
left_touch(false),
current_ultrasonicSensor_x(0),
current_ultrasonicSensor_y(0),
current_ultrasonicSensor_z(0),
current_colourSensor_x(0),
current_colourSensor_y(0),
current_colourSensor_z(0),
right_touch_value(0),
left_touch_value(0),
velocity_mmotor(1.6)
{

}

void EV3Obj::setTargetVelocity(double vx, double va)
{
	target_vx = vx;
	target_va = va;
}

void EV3Obj::setCurrentPosition(double px, double py, double pa)
{
	current_px = px;
	current_py = py;
	current_pa = pa;
}

void EV3Obj::setUltrasonicSensorPos(double x, double y, double z)
{
	current_ultrasonicSensor_x = x;
	current_ultrasonicSensor_y = y;
	current_ultrasonicSensor_z = z;
}
void EV3Obj::setColourSensorPos(double x, double y, double z)
{
	current_colourSensor_x = x;
	current_colourSensor_y = y;
	current_colourSensor_z = z;
}


bool EV3Obj::getRightTouch(double limit)
{
	if (right_touch_value > limit)
	{
		right_touch = true;
	}
	else
	{
		right_touch = false;
	}
	return right_touch;
}
bool EV3Obj::getLeftTouch(double limit)
{
	if (left_touch_value > limit)
	{
		left_touch = true;
	}
	else
	{
		left_touch = false;
	}
	return left_touch;
}




double EV3Obj::calcUltrasonicSensorDistance(double x, double y, double z)
{
	double tmp = sqrt(pow(current_ultrasonicSensor_x - x,2) + pow(current_ultrasonicSensor_y - y,2) + pow(current_ultrasonicSensor_z - z,2));
	if (current_ultrasonicSensorData > tmp)
	{
		current_ultrasonicSensorData = tmp;
	}
	//std::cout << x << "\t" << y << "\t" << z << std::endl;
	//std::cout << current_ultrasonicSensorData << std::endl;
	return current_ultrasonicSensorData;
}
double EV3Obj::calcColourSensorDistance(double x, double y, double z)
{
	double tmp = sqrt(pow(current_colourSensor_x - x, 2) + pow(current_colourSensor_y - y, 2) + pow(current_colourSensor_z - z, 2));
	if (current_colourSensorData > tmp)
	{
		current_colourSensorData = tmp;
	}
	std::cout << current_colourSensorData << std::endl;
	return current_colourSensorData;
}


void EV3Obj::resetUltrasonicSensorDistance()
{
	current_ultrasonicSensorData = 100;
}
void EV3Obj::resetColourSensorDistance()
{
	current_colourSensorData = 100;
}
/**
*@brief シミュレーションの操作をするためのクラスのコンストラクタ
*/
EV3SimulatorObj::EV3SimulatorObj()
{
	st = 0.01;
	gravity = 9.8;
	pause = false;
	plane_exist = false;
	

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
	//makePlane(100,100,10);

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
*@param offset_z 高さを調整
*/
void EV3SimulatorObj::makeParam(double offset_z)
{
	EV3Block.m = DEFAULT_BODY_MASS;
	EV3Block.lx = DEFAULT_BODY_LENGTH;
	EV3Block.ly = DEFAULT_BODY_WIDTH;
	EV3Block.lz = DEFAULT_BODY_HEIGHT;
	EV3Block.x = DEFAULT_BODY_X;
	EV3Block.y = DEFAULT_BODY_Y;
	EV3Block.z = DEFAULT_BODY_Z + offset_z;
	EV3Block.red = 0.9;
	EV3Block.green = 0.9;
	EV3Block.blue = 0.9;

	wheelLeft.m = DEFAULT_WHEEL_MASS;
	wheelLeft.lx = DEFAULT_WHEEL_RADIUS;
	wheelLeft.lz = DEFAULT_WHEEL_WIDTH;
	wheelLeft.x = DEFAULT_WHEEL_X;
	wheelLeft.y = DEFAULT_WHEEL_Y;
	wheelLeft.z = DEFAULT_WHEEL_Z + offset_z;
	wheelLeft.jx = wheelLeft.x;
	wheelLeft.jy = wheelLeft.y;
	wheelLeft.jz = wheelLeft.z;
	wheelLeft.axisx = 0;
	wheelLeft.axisy = -1;
	wheelLeft.axisz = 0;
	wheelLeft.red = 0.1;
	wheelLeft.green = 0.1;
	wheelLeft.blue = 0.1;

	wheelRight.m = DEFAULT_WHEEL_MASS;
	wheelRight.lx = DEFAULT_WHEEL_RADIUS;
	wheelRight.lz = DEFAULT_WHEEL_WIDTH;
	wheelRight.x = DEFAULT_WHEEL_X;
	wheelRight.y = DEFAULT_BODY_Y - (DEFAULT_WHEEL_Y - DEFAULT_BODY_Y);
	wheelRight.z = DEFAULT_WHEEL_Z + offset_z;
	wheelRight.jx = wheelRight.x;
	wheelRight.jy = wheelRight.y;
	wheelRight.jz = wheelRight.z;
	wheelRight.axisx = 0;
	wheelRight.axisy = -1;
	wheelRight.axisz = 0;
	wheelRight.red = 0.1;
	wheelRight.green = 0.1;
	wheelRight.blue = 0.1;

	wheelBall.m = DEFAULT_BALL_MASS;
	wheelBall.lz = DEFAULT_BALL_RADIUS;
	wheelBall.x = DEFAULT_BALL_X;
	wheelBall.y = DEFAULT_BALL_Y;
	wheelBall.z = DEFAULT_BALL_Z + offset_z;
	wheelBall.jx = wheelBall.x;
	wheelBall.jy = wheelBall.y;
	wheelBall.jz = wheelBall.z;
	wheelBall.red = 1.;
	wheelBall.green = 1.;
	wheelBall.blue = 1.;

	const double touchsensor_stick_length = 0.7;
	
	const double touchsensor_stick_distance = (DEFAULT_TOUCHSENSOR_X - (DEFAULT_BODY_X + DEFAULT_BODY_LENGTH / 2.0));

	touchSensorLeft[0].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorLeft[0].lx = touchsensor_stick_distance * touchsensor_stick_length;
	touchSensorLeft[0].ly = DEFAULT_STICK_WIDTH;
	touchSensorLeft[0].lz = DEFAULT_STICK_WIDTH;
	touchSensorLeft[0].x = DEFAULT_BODY_X + DEFAULT_BODY_LENGTH / 2.0 + touchSensorLeft[0].lx / 2.0;
	touchSensorLeft[0].y = DEFAULT_BODY_Y + DEFAULT_BODY_WIDTH / 2.0 + DEFAULT_STICK_WIDTH/2.0;
	touchSensorLeft[0].z = DEFAULT_BODY_Z - touchsensor_stick_distance*(1.0 - touchsensor_stick_length)*sin(DEFAULT_STICK_RADIUS) + offset_z;
	touchSensorLeft[0].red = 0.7;
	touchSensorLeft[0].green = 0.7;
	touchSensorLeft[0].blue = 0.7;

	touchSensorLeft[1].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorLeft[1].lx = touchsensor_stick_distance*(1.0 - touchsensor_stick_length)/cos(DEFAULT_STICK_RADIUS);
	touchSensorLeft[1].ly = DEFAULT_STICK_WIDTH;
	touchSensorLeft[1].lz = DEFAULT_STICK_WIDTH;
	touchSensorLeft[1].x = touchSensorLeft[0].x + touchSensorLeft[0].lx / 2.0 + touchsensor_stick_distance*(1.0 - touchsensor_stick_length) / 2.0;
	touchSensorLeft[1].y = touchSensorLeft[0].y + DEFAULT_STICK_WIDTH;
	touchSensorLeft[1].z = touchSensorLeft[0].z + (DEFAULT_TOUCHSENSOR_Z - touchSensorLeft[0].z + offset_z) / 2.0;
	touchSensorLeft[1].red = 0.7;
	touchSensorLeft[1].green = 0.7;
	touchSensorLeft[1].blue = 0.7;

	touchSensorLeft[2].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorLeft[2].lx = DEFAULT_TOUCHSENSOR_LENGTH;
	touchSensorLeft[2].ly = DEFAULT_TOUCHSENSOR_WIDTH;
	touchSensorLeft[2].lz = DEFAULT_TOUCHSENSOR_HEIGHT;
	touchSensorLeft[2].x = DEFAULT_TOUCHSENSOR_X;
	touchSensorLeft[2].y = DEFAULT_TOUCHSENSOR_Y;
	touchSensorLeft[2].z = DEFAULT_TOUCHSENSOR_Z + offset_z;
	touchSensorLeft[2].red = 0.7;
	touchSensorLeft[2].green = 0.7;
	touchSensorLeft[2].blue = 0.7;

	touchSensorLeft[3].m = DEFAULT_TOUCHSENSOR2_MASS;
	touchSensorLeft[3].lx = DEFAULT_TOUCHSENSOR2_LENGTH;
	touchSensorLeft[3].ly = DEFAULT_TOUCHSENSOR2_WIDTH;
	touchSensorLeft[3].lz = DEFAULT_TOUCHSENSOR2_HEIGHT;
	touchSensorLeft[3].x = DEFAULT_TOUCHSENSOR2_X;
	touchSensorLeft[3].y = DEFAULT_TOUCHSENSOR2_Y;
	touchSensorLeft[3].z = DEFAULT_TOUCHSENSOR2_Z + offset_z;
	touchSensorLeft[3].axisx = 1;
	touchSensorLeft[3].axisy = 0;
	touchSensorLeft[3].axisz = 0;
	touchSensorLeft[3].red = 0.6;
	touchSensorLeft[3].green = 0.6;
	touchSensorLeft[3].blue = 0.6;

	touchSensorRight[0].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorRight[0].lx = touchsensor_stick_distance * touchsensor_stick_length;
	touchSensorRight[0].ly = DEFAULT_STICK_WIDTH;
	touchSensorRight[0].lz = DEFAULT_STICK_WIDTH;
	touchSensorRight[0].x = DEFAULT_BODY_X + DEFAULT_BODY_LENGTH / 2.0 + touchSensorRight[0].lx / 2.0;
	touchSensorRight[0].y = DEFAULT_BODY_Y - DEFAULT_BODY_WIDTH / 2.0 - DEFAULT_STICK_WIDTH / 2.0;
	touchSensorRight[0].z = DEFAULT_BODY_Z - touchsensor_stick_distance*(1.0 - touchsensor_stick_length)*sin(DEFAULT_STICK_RADIUS) + offset_z;
	touchSensorRight[0].red = 0.7;
	touchSensorRight[0].green = 0.7;
	touchSensorRight[0].blue = 0.7;

	touchSensorRight[1].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorRight[1].lx = touchsensor_stick_distance*(1.0 - touchsensor_stick_length) / cos(DEFAULT_STICK_RADIUS);
	touchSensorRight[1].ly = DEFAULT_STICK_WIDTH;
	touchSensorRight[1].lz = DEFAULT_STICK_WIDTH;
	touchSensorRight[1].x = touchSensorRight[0].x + touchSensorRight[0].lx / 2.0 + touchsensor_stick_distance*(1.0 - touchsensor_stick_length) / 2.0;
	touchSensorRight[1].y = touchSensorRight[0].y - DEFAULT_STICK_WIDTH;
	touchSensorRight[1].z = touchSensorRight[0].z + (DEFAULT_TOUCHSENSOR_Z - touchSensorLeft[0].z + offset_z) / 2.0;
	touchSensorRight[1].red = 0.7;
	touchSensorRight[1].green = 0.7;
	touchSensorRight[1].blue = 0.7;

	touchSensorRight[2].m = DEFAULT_TOUCHSENSOR_MASS;
	touchSensorRight[2].lx = DEFAULT_TOUCHSENSOR_LENGTH;
	touchSensorRight[2].ly = DEFAULT_TOUCHSENSOR_WIDTH;
	touchSensorRight[2].lz = DEFAULT_TOUCHSENSOR_HEIGHT;
	touchSensorRight[2].x = DEFAULT_TOUCHSENSOR_X;
	touchSensorRight[2].y = DEFAULT_BODY_Y - (DEFAULT_TOUCHSENSOR_Y - DEFAULT_BODY_Y);
	touchSensorRight[2].z = DEFAULT_TOUCHSENSOR_Z + offset_z;
	touchSensorRight[2].red = 0.7;
	touchSensorRight[2].green = 0.7;
	touchSensorRight[2].blue = 0.7;

	

	touchSensorRight[3].m = DEFAULT_TOUCHSENSOR2_MASS;
	touchSensorRight[3].lx = DEFAULT_TOUCHSENSOR2_LENGTH;
	touchSensorRight[3].ly = DEFAULT_TOUCHSENSOR2_WIDTH;
	touchSensorRight[3].lz = DEFAULT_TOUCHSENSOR2_HEIGHT;
	touchSensorRight[3].x = DEFAULT_TOUCHSENSOR2_X;
	touchSensorRight[3].y = DEFAULT_BODY_Y - (DEFAULT_TOUCHSENSOR2_Y - DEFAULT_BODY_Y);
	touchSensorRight[3].z = DEFAULT_TOUCHSENSOR2_Z + offset_z;
	touchSensorRight[3].axisx = 1;
	touchSensorRight[3].axisy = 0;
	touchSensorRight[3].axisz = 0;
	touchSensorRight[3].red = 0.6;
	touchSensorRight[3].green = 0.6;
	touchSensorRight[3].blue = 0.6;

	double gyro_stick_length = ((DEFAULT_BODY_X - DEFAULT_BODY_LENGTH / 2.0) - DEFAULT_GYROSENSOR_X);
	gyroSensor[0].m = DEFAULT_GYROSENSOR_MASS;
	gyroSensor[0].lx = gyro_stick_length / cos(DEFAULT_STICK_RADIUS);
	gyroSensor[0].ly = DEFAULT_STICK_WIDTH;
	gyroSensor[0].lz = DEFAULT_STICK_WIDTH;
	gyroSensor[0].x = DEFAULT_BODY_X - DEFAULT_BODY_LENGTH / 2.0 - gyroSensor[0].lx * cos(DEFAULT_STICK_RADIUS) / 2.0;
	gyroSensor[0].y = DEFAULT_GYROSENSOR_Y;
	gyroSensor[0].z = DEFAULT_BODY_Z + gyroSensor[0].lx * sin(DEFAULT_STICK_RADIUS) / 2.0 + offset_z;
	gyroSensor[0].red = 0.7;
	gyroSensor[0].green = 0.7;
	gyroSensor[0].blue = 0.7;

	gyroSensor[1].m = DEFAULT_GYROSENSOR_MASS;
	gyroSensor[1].lx = DEFAULT_STICK_WIDTH;
	gyroSensor[1].ly = DEFAULT_STICK_WIDTH;
	gyroSensor[1].lz = (DEFAULT_GYROSENSOR_Z - DEFAULT_GYROSENSOR_HEIGHT / 2.0) - (gyroSensor[0].z - offset_z) - gyroSensor[0].lx * sin(DEFAULT_STICK_RADIUS) / 2.0;
	gyroSensor[1].x = DEFAULT_GYROSENSOR_X;
	gyroSensor[1].y = DEFAULT_GYROSENSOR_Y;
	gyroSensor[1].z = DEFAULT_GYROSENSOR_Z - gyroSensor[1].lz / 2.0 - DEFAULT_GYROSENSOR_HEIGHT / 2.0 + offset_z;
	gyroSensor[1].red = 0.7;
	gyroSensor[1].green = 0.7;
	gyroSensor[1].blue = 0.7;

	gyroSensor[2].m = DEFAULT_GYROSENSOR_MASS;
	gyroSensor[2].lx = DEFAULT_GYROSENSOR_LENGTH;
	gyroSensor[2].ly = DEFAULT_GYROSENSOR_WIDTH;
	gyroSensor[2].lz = DEFAULT_GYROSENSOR_HEIGHT;
	gyroSensor[2].x = DEFAULT_GYROSENSOR_X;
	gyroSensor[2].y = DEFAULT_GYROSENSOR_Y;
	gyroSensor[2].z = DEFAULT_GYROSENSOR_Z + offset_z;
	gyroSensor[2].red = 0.6;
	gyroSensor[2].green = 0.6;
	gyroSensor[2].blue = 0.6;

	



	colourSensor.m = DEFAULT_COLOURSENSOR_MASS;
	colourSensor.lx = DEFAULT_COLOURSENSOR_LENGTH;
	colourSensor.ly = DEFAULT_COLOURSENSOR_WIDTH;
	colourSensor.lz = DEFAULT_COLOURSENSOR_HEIGHT;
	colourSensor.x = DEFAULT_COLOURSENSOR_X;
	colourSensor.y = DEFAULT_COLOURSENSOR_Y;
	colourSensor.z = DEFAULT_COLOURSENSOR_Z + offset_z;
	colourSensor.red = 0.6;
	colourSensor.green = 0.6;
	colourSensor.blue = 0.6;

	mmotor.m = DEFAULT_MMOTOR_MASS;
	mmotor.lx = DEFAULT_MMOTOR_LENGTH;
	mmotor.ly = DEFAULT_MMOTOR_WIDTH;
	mmotor.lz = DEFAULT_MMOTOR_HEIGHT;
	mmotor.x = DEFAULT_MMOTOR_X;
	mmotor.y = DEFAULT_MMOTOR_Y;
	mmotor.z = DEFAULT_MMOTOR_Z + offset_z;
	mmotor.red = 0.6;
	mmotor.green = 0.6;
	mmotor.blue = 0.6;

	ultrasonicSensor[0].m = DEFAULT_ULTRASONICSENSOR_MASS;
	ultrasonicSensor[0].lx = DEFAULT_STICK_WIDTH;
	ultrasonicSensor[0].ly = DEFAULT_STICK_WIDTH;
	ultrasonicSensor[0].lz = DEFAULT_ULTRASONICSENSOR_Z - (DEFAULT_MMOTOR_Z + DEFAULT_MMOTOR_HEIGHT/2.0);
	ultrasonicSensor[0].x = DEFAULT_MMOTOR_X;
	ultrasonicSensor[0].y = DEFAULT_MMOTOR_Y;
	ultrasonicSensor[0].z = DEFAULT_MMOTOR_Z + DEFAULT_MMOTOR_HEIGHT / 2.0 + ultrasonicSensor[0].lz / 2.0 + offset_z;
	ultrasonicSensor[0].jx = DEFAULT_MMOTOR_X;
	ultrasonicSensor[0].jy = DEFAULT_MMOTOR_Y;
	ultrasonicSensor[0].jz = DEFAULT_MMOTOR_Z;
	ultrasonicSensor[0].axisx = 0;
	ultrasonicSensor[0].axisy = 0;
	ultrasonicSensor[0].axisz = -1;
	ultrasonicSensor[0].red = 0.7;
	ultrasonicSensor[0].green = 0.7;
	ultrasonicSensor[0].blue = 0.7;


	ultrasonicSensor[1].m = DEFAULT_ULTRASONICSENSOR_MASS;
	ultrasonicSensor[1].lx = (DEFAULT_ULTRASONICSENSOR_X - DEFAULT_ULTRASONICSENSOR_LENGTH / 2.0) - DEFAULT_MMOTOR_X;
	ultrasonicSensor[1].ly = DEFAULT_STICK_WIDTH;
	ultrasonicSensor[1].lz = DEFAULT_STICK_WIDTH;
	ultrasonicSensor[1].x = ultrasonicSensor[0].x + ultrasonicSensor[1].lx/2.0;
	ultrasonicSensor[1].y = ultrasonicSensor[0].y;
	ultrasonicSensor[1].z = ultrasonicSensor[0].z + ultrasonicSensor[0].lz / 2.0;
	ultrasonicSensor[1].red = 0.7;
	ultrasonicSensor[1].green = 0.7;
	ultrasonicSensor[1].blue = 0.7;

	ultrasonicSensor[2].m = DEFAULT_ULTRASONICSENSOR_MASS;
	ultrasonicSensor[2].lx = DEFAULT_ULTRASONICSENSOR_LENGTH;
	ultrasonicSensor[2].ly = DEFAULT_ULTRASONICSENSOR_WIDTH;
	ultrasonicSensor[2].lz = DEFAULT_ULTRASONICSENSOR_HEIGHT;
	ultrasonicSensor[2].x = DEFAULT_ULTRASONICSENSOR_X;
	ultrasonicSensor[2].y = DEFAULT_ULTRASONICSENSOR_Y;
	ultrasonicSensor[2].z = DEFAULT_ULTRASONICSENSOR_Z + offset_z;
	ultrasonicSensor[2].red = 0.4;
	ultrasonicSensor[2].green = 0.4;
	ultrasonicSensor[2].blue = 0.4;

	
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
	dMatrix3 R;
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
	setBox(&gyroSensor[0]);
	dRFromAxisAndAngle(R, 0, 1, 0, DEFAULT_STICK_RADIUS);
	dGeomSetRotation(gyroSensor[0].geom, R);
	setBox(&gyroSensor[1]);
	setBox(&gyroSensor[2]);
	setBox(&colourSensor);
	setBox(&mmotor);
	setBox(&ultrasonicSensor[0]);
	setBox(&ultrasonicSensor[1]);
	setBox(&ultrasonicSensor[2]);

	
	dRFromAxisAndAngle(R, 1, 0, 0, M_PI/2);
	dGeomSetRotation(wheelLeft.geom, R);
	
	dRFromAxisAndAngle(R, 1, 0, 0, M_PI / 2);
	dGeomSetRotation(wheelRight.geom, R);

	dRFromAxisAndAngle(R, 0, 1, 0, -DEFAULT_STICK_RADIUS);
	dGeomSetRotation(touchSensorLeft[1].geom, R);
	dRFromAxisAndAngle(R, 0, 1, 0, -DEFAULT_STICK_RADIUS);
	dGeomSetRotation(touchSensorRight[1].geom, R);


	//EV3Block.joint = dJointCreateFixed(world, 0);
	//dJointAttach(EV3Block.joint, EV3Block.body, 0);
	//dJointSetFixed(EV3Block.joint);

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
	setFixed(&gyroSensor[0], &EV3Block);
	setFixed(&gyroSensor[1], &gyroSensor[0]);
	setFixed(&gyroSensor[2], &gyroSensor[1]);
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
	setHinge(&ultrasonicSensor[0], &mmotor);
	setFixed(&ultrasonicSensor[1], &ultrasonicSensor[0]);
	setFixed(&ultrasonicSensor[2], &ultrasonicSensor[1]);

	ultrasonicSensor_ray = dCreateRay(space, 2);
	colourSensor_ray = dCreateRay(space, 2);
	
	

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
  //if ((o1 != ground) && (o2 != ground)) return;
  //if ((o1 == plane.geom) || (o2 == plane.geom)) return;
 
   static const int N = 20;
	dContact contact[N];
	int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

	
	if ((o1 == ultrasonicSensor_ray && o2 == ground) || (o2 == ultrasonicSensor_ray && o1 == ground)){

		ev3.calcUltrasonicSensorDistance(contact[0].geom.pos[0], contact[0].geom.pos[1], contact[0].geom.pos[2]);
		return;
	}
	else if (plane_exist && ((o1 == ultrasonicSensor_ray && o2 == plane.geom) || (o2 == ultrasonicSensor_ray && o1 == plane.geom)))
	{
		ev3.calcUltrasonicSensorDistance(contact[0].geom.pos[0], contact[0].geom.pos[1], contact[0].geom.pos[2]);
		return;
	}
	else if (o1 == ultrasonicSensor_ray || o2 == ultrasonicSensor_ray){
		return;
	}


	if ((o1 == colourSensor_ray && o2 == ground) || (o2 == colourSensor_ray && o1 == ground)){

		ev3.calcColourSensorDistance(contact[0].geom.pos[0], contact[0].geom.pos[1], contact[0].geom.pos[2]);
		return;
	}
	else if (plane_exist && ((o1 == colourSensor_ray && o2 == plane.geom) || (o2 == colourSensor_ray && o1 == plane.geom)))
	{
		ev3.calcColourSensorDistance(contact[0].geom.pos[0], contact[0].geom.pos[1], contact[0].geom.pos[2]);
		return;
	}
	else if (o1 == colourSensor_ray || o2 == colourSensor_ray){
		return;
	}

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
*@brief ヒンジジョイント制御
* @param body ボディオブジェクト
* @param theta ヒンジジョイントの位置
* @param vel ヒンジジョイントの速度
*/
void EV3SimulatorObj::controlHinge(MyLink *body, dReal theta, dReal vel)
{
	dReal kp = 100;
	dReal tmp = dJointGetHingeAngle(body->joint);
	dReal diff = theta - tmp;
	dReal u = kp * diff;
	if (abs(vel) < abs(u))
	{
		u = vel;
	}

	dJointSetHingeParam(body->joint, dParamVel, u);
	dJointSetHingeParam(body->joint, dParamFMax, 20.);
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
	double vx = ev3.target_vx;
	double va = ev3.target_va;
	double wheel_distance = (wheelLeft.y - wheelRight.y)/2.0;
	double wheel_radius = wheelLeft.lx;

	double right_motor_speed = (vx + va*wheel_distance) / wheel_radius;
	double left_motor_speed = (vx - va*wheel_distance) / wheel_radius;

	
	dJointSetHingeParam(wheelLeft.joint, dParamVel, left_motor_speed);
	dJointSetHingeParam(wheelLeft.joint, dParamFMax, 20.);

	dJointSetHingeParam(wheelRight.joint, dParamVel, right_motor_speed);
	dJointSetHingeParam(wheelRight.joint, dParamFMax, 20.);
	//ev3.velocity_mmotor = 0.1;
	controlHinge(&ultrasonicSensor[0], ev3.target_mangle, ev3.velocity_mmotor);

	ev3.left_touch_value = dJointGetSliderPosition(touchSensorLeft[3].joint);
	ev3.right_touch_value = dJointGetSliderPosition(touchSensorRight[3].joint);

	double left_wheel_speed = dJointGetHingeParam(wheelLeft.joint, dParamVel) * wheel_radius;
	double right_wheel_speed = dJointGetHingeParam(wheelRight.joint, dParamVel) * wheel_radius;
	double o = (right_wheel_speed - left_wheel_speed) / (2.0*wheel_distance);
	double v = (right_wheel_speed + left_wheel_speed) / 2.0;

	double current_vx = v * cos(ev3.current_pa);
	double current_vy = v * sin(ev3.current_pa);
	double current_va = o;

	ev3.current_px += current_vx*st;
	ev3.current_py += current_vy*st;
	ev3.current_pa += current_va*st;
	//std::cout << lp << "\t" << rp << std::endl;
	/*

	dReal r2 = dJointGetHingeAngle (link2.joint);


	controlHinge(&link1, -rb->theta[0]+rb->offset[0]);//

	*/
}

/**
*@brief 超音波距離センサを設定
*/
void EV3SimulatorObj::setUltrasonicSensorRay()
{
	dVector3 pos0, pos1;
	dBodyGetRelPointPos(ultrasonicSensor[2].body, 0, 0, -ultrasonicSensor[2].lz / 2.0, pos0);
	dBodyGetRelPointPos(ultrasonicSensor[2].body, 0, 0, -DEFAULT_MAX_ULTRASONICSENSOR_DISTANCE - ultrasonicSensor[2].lz/2.0, pos1);
	ev3.setUltrasonicSensorPos(pos0[0], pos0[1], pos0[2]);
	//std::cout << "pos0\t" << pos0[0] << "\t" << pos0[1] << "\t" << pos0[2] << std::endl;
	//std::cout << "pos1\t" << pos1[0] << "\t" << pos1[1] << "\t" << pos1[2] << std::endl;
	dGeomRaySet(ultrasonicSensor_ray, pos0[0], pos0[1], pos0[2], pos1[0] - pos0[0], pos1[1] - pos0[1], pos1[2] - pos0[2]);
	ev3.resetUltrasonicSensorDistance();
}
/**
*@brief カラーセンサを設定
*/
void EV3SimulatorObj::setColourSensorRay()
{
	dVector3 pos0, pos1;
	dBodyGetRelPointPos(colourSensor.body, 0, 0, -colourSensor.lz/2.0, pos0);
	dBodyGetRelPointPos(colourSensor.body, 0, 0, -DEFAULT_MAX_ULTRASONICSENSOR_DISTANCE - colourSensor.lz / 2.0, pos1);
	ev3.setColourSensorPos(pos0[0], pos0[1], pos0[2]);
	dGeomRaySet(colourSensor_ray, pos0[0], pos0[1], pos0[2], pos1[0] - pos0[0], pos1[1] - pos0[1], pos1[2] - pos0[2]);
	ev3.resetColourSensorDistance();
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

		setUltrasonicSensorRay();
		setColourSensorRay();

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

	dJointDestroy(wheelLeft.joint);
	dJointDestroy(wheelRight.joint);
	dJointDestroy(wheelBall.joint);

	dJointDestroy(touchSensorLeft[0].joint);
	dJointDestroy(touchSensorLeft[1].joint);
	dJointDestroy(touchSensorLeft[2].joint);
	dJointDestroy(touchSensorLeft[3].joint);

	dJointDestroy(touchSensorRight[0].joint);
	dJointDestroy(touchSensorRight[1].joint);
	dJointDestroy(touchSensorRight[2].joint);
	dJointDestroy(touchSensorRight[3].joint);

	dJointDestroy(gyroSensor[0].joint);
	dJointDestroy(gyroSensor[1].joint);
	dJointDestroy(gyroSensor[2].joint);

	dJointDestroy(colourSensor.joint);

	dJointDestroy(mmotor.joint);


	dJointDestroy(ultrasonicSensor[0].joint);
	dJointDestroy(ultrasonicSensor[1].joint);
	dJointDestroy(ultrasonicSensor[2].joint);

	
	dBodyDestroy(EV3Block.body);
	dBodyDestroy(wheelLeft.body);
	dBodyDestroy(wheelRight.body);
	dBodyDestroy(wheelBall.body);


	dBodyDestroy(touchSensorLeft[0].body);
	dBodyDestroy(touchSensorLeft[1].body);
	dBodyDestroy(touchSensorLeft[2].body);
	dBodyDestroy(touchSensorLeft[3].body);

	dBodyDestroy(touchSensorRight[0].body);
	dBodyDestroy(touchSensorRight[1].body);
	dBodyDestroy(touchSensorRight[2].body);
	dBodyDestroy(touchSensorRight[3].body);

	dBodyDestroy(gyroSensor[0].body);
	dBodyDestroy(gyroSensor[1].body);
	dBodyDestroy(gyroSensor[2].body);

	dBodyDestroy(colourSensor.body);

	dBodyDestroy(mmotor.body);
	

	

	dBodyDestroy(ultrasonicSensor[0].body);
	dBodyDestroy(ultrasonicSensor[1].body);
	dBodyDestroy(ultrasonicSensor[2].body);


	dGeomDestroy(EV3Block.geom);
	dGeomDestroy(wheelLeft.geom);
	dGeomDestroy(wheelRight.geom);
	dGeomDestroy(wheelBall.geom);


	dGeomDestroy(touchSensorLeft[0].geom);
	dGeomDestroy(touchSensorLeft[1].geom);
	dGeomDestroy(touchSensorLeft[2].geom);
	dGeomDestroy(touchSensorLeft[3].geom);

	dGeomDestroy(touchSensorRight[0].geom);
	dGeomDestroy(touchSensorRight[1].geom);
	dGeomDestroy(touchSensorRight[2].geom);
	dGeomDestroy(touchSensorRight[3].geom);

	dGeomDestroy(gyroSensor[0].geom);
	dGeomDestroy(gyroSensor[1].geom);
	dGeomDestroy(gyroSensor[2].geom);

	dGeomDestroy(colourSensor.geom);

	dGeomDestroy(mmotor.geom);




	dGeomDestroy(ultrasonicSensor[0].geom);
	dGeomDestroy(ultrasonicSensor[1].geom);
	dGeomDestroy(ultrasonicSensor[2].geom);
	
	
	
	dGeomDestroy(ultrasonicSensor_ray);
	dGeomDestroy(colourSensor_ray);
	

	if (plane_exist)
	{
		dJointDestroy(plane.joint);
		dBodyDestroy(plane.body);
		dGeomDestroy(plane.geom);
		plane_exist = false;
	}
	

	for (std::vector<MyLink>::iterator itr = blocks.begin(); itr != blocks.end(); ++itr) {
		dJointDestroy(itr->joint);
		dBodyDestroy(itr->body);
		dGeomDestroy(itr->geom);
	}
	blocks.clear();

	mu.unlock();
}

void EV3SimulatorObj::makePlane(double x, double y, double lx, double ly, double lz)
{
	mu.lock();
	plane.m = DEFAULT_BLOCK_MASS;
	plane.lx = lx;
	plane.ly = ly;
	plane.lz = lz;
	plane.x = x;
	plane.y = y;
	plane.z = lz / 2.0;
	plane.red = 0.;
	plane.green = 1.;
	plane.blue = 0.;

	setBox(&plane);

	plane.joint = dJointCreateFixed(world, 0);
	dJointAttach(plane.joint, plane.body, 0);
	dJointSetFixed(plane.joint);

	plane_exist = true;
	mu.unlock();
}

/**
*@brief 障害物生成
*/
void EV3SimulatorObj::makeBlock(double x, double y, double z, double lx, double ly, double lz, double r)
{
	mu.lock();
	MyLink block;
	block.m = DEFAULT_BLOCK_MASS;
	block.lx = lx;
	block.ly = ly;
	block.lz = lz;
	block.x = x;
	block.y = y;
	block.z = z;
	block.red = 0.;
	block.green = 1.;
	block.blue = 0.;

	setBox(&block);

	dMatrix3 R;
	dRFromAxisAndAngle(R, 0, 0, 1, r);
	dGeomSetRotation(block.geom, R);

	block.joint = dJointCreateFixed(world, 0);
	dJointAttach(block.joint, block.body, 0);
	dJointSetFixed(block.joint);

	blocks.push_back(block);
	mu.unlock();
}


/**
*@brief ファイルから障害物は位置読み込み
*@return 読み込み成功(true)、失敗(false)
*/
bool EV3SimulatorObj::loadBlocksData(std::string fname)
{
	
#ifdef WIN32
	coil::replaceString(fname, "/", "\\");
#else
	coil::replaceString(fname, "\\", "/");
#endif
	std::ifstream ifs(fname);
	if (ifs.fail())
	{
		return false;
	}

	std::string str;
	int count = 0;
	while (getline(ifs, str))
	{
		if (count > 0)
		{
			std::vector<std::string> vlist = coil::split(str,",");
			std::vector<double> rlist;
			for (std::vector<std::string>::iterator itr = vlist.begin(); itr != vlist.end(); ++itr) {
				std::string v = (*itr);
				coil::eraseBothEndsBlank(v);
				double val = 0;
				bool ret = coil::stringTo<double>(val, v.c_str());
				rlist.push_back(val);
			}
			if (rlist.size() >= 7)
			{
				makeBlock(rlist[0], rlist[1], rlist[2], rlist[3], rlist[4], rlist[5], rlist[6]);
			}
			
		}
		count += 1;
	}

	return true;
}

/**
*@brief 刻み幅設定
* @param s サンプリング時間
*/
void EV3SimulatorObj::setSamplingTime(double s)
{
	st = s;
}