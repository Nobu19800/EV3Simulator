/*!
* @file  SimulatorObj.h
* @brief シミュレーション関連のクラス
*
*/

#ifndef EV3SIMULATOROBJ_H
#define EV3SIMULATOROBJ_H

#define dDOUBLE

#include <coil/Mutex.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <vector>


#define _USE_MATH_DEFINES
#include <math.h>


#define DEFAULT_STICK_WIDTH 0.005
#define DEFAULT_STICK_RADIUS (M_PI/4.0)


#define DEFAULT_BODY_LENGTH 0.1
#define DEFAULT_BODY_WIDTH 0.05
#define DEFAULT_BODY_HEIGHT 0.05


#define DEFAULT_BODY_X 0
#define DEFAULT_BODY_Y 0
#define DEFAULT_BODY_Z (0.1)


#define DEFAULT_BODY_MASS 0.5

#define DEFAULT_WHEEL_RADIUS 0.02
#define DEFAULT_WHEEL_WIDTH 0.02

#define DEFAULT_WHEEL_X (DEFAULT_BODY_X + 0.02)
#define DEFAULT_WHEEL_Y (DEFAULT_BODY_Y + 0.045)
#define DEFAULT_WHEEL_Z (DEFAULT_BODY_Z - 0.025)

#define DEFAULT_WHEEL_MASS 0.01

#define DEFAULT_BALL_RADIUS 0.01

#define DEFAULT_BALL_X (DEFAULT_BODY_X - 0.05)
#define DEFAULT_BALL_Y DEFAULT_BODY_Y
#define DEFAULT_BALL_Z (DEFAULT_BODY_Z - 0.035)

#define DEFAULT_BALL_MASS 0.01

#define DEFAULT_TOUCHSENSOR_LENGTH 0.035
#define DEFAULT_TOUCHSENSOR_WIDTH 0.02
#define DEFAULT_TOUCHSENSOR_HEIGHT 0.02

#define DEFAULT_TOUCHSENSOR_X (DEFAULT_BODY_X + 0.15)
#define DEFAULT_TOUCHSENSOR_Y (DEFAULT_BODY_Y + DEFAULT_BODY_WIDTH/2.0 + DEFAULT_TOUCHSENSOR_WIDTH/2.0 + DEFAULT_STICK_WIDTH*2)
#define DEFAULT_TOUCHSENSOR_Z (DEFAULT_BODY_Z)

#define DEFAULT_TOUCHSENSOR_MASS 0.01

#define DEFAULT_TOUCHSENSOR2_MOVE 0.004
#define DEFAULT_TOUCHSENSOR2_LENGTH (DEFAULT_TOUCHSENSOR2_MOVE*2)
#define DEFAULT_TOUCHSENSOR2_WIDTH (DEFAULT_TOUCHSENSOR_WIDTH*0.8)
#define DEFAULT_TOUCHSENSOR2_HEIGHT (DEFAULT_TOUCHSENSOR_HEIGHT*0.8)
#define DEFAULT_TOUCHSENSOR2_X (DEFAULT_TOUCHSENSOR_X + DEFAULT_TOUCHSENSOR_LENGTH/2.0)
#define DEFAULT_TOUCHSENSOR2_Y DEFAULT_TOUCHSENSOR_Y
#define DEFAULT_TOUCHSENSOR2_Z DEFAULT_TOUCHSENSOR_Z
#define DEFAULT_TOUCHSENSOR2_MASS 0.01

#define DEFAULT_SPRINFG_VALUE 10

#define DEFAULT_GYROSENSOR_LENGTH 0.02
#define DEFAULT_GYROSENSOR_WIDTH 0.04
#define DEFAULT_GYROSENSOR_HEIGHT 0.02

#define DEFAULT_GYROSENSOR_X (DEFAULT_BODY_X - 0.06)
#define DEFAULT_GYROSENSOR_Y DEFAULT_BODY_Y
#define DEFAULT_GYROSENSOR_Z (DEFAULT_BODY_Z + 0.06)

#define DEFAULT_GYROSENSOR_MASS 0.01


#define DEFAULT_COLOURSENSOR_LENGTH 0.01
#define DEFAULT_COLOURSENSOR_WIDTH 0.01
#define DEFAULT_COLOURSENSOR_HEIGHT 0.03


#define DEFAULT_COLOURSENSOR_X (DEFAULT_BODY_X + 0.07)
#define DEFAULT_COLOURSENSOR_Y (DEFAULT_BODY_Y + 0.02)
#define DEFAULT_COLOURSENSOR_Z (DEFAULT_BODY_Z - 0.01)

#define DEFAULT_COLOURSENSOR_MASS 0.01


#define DEFAULT_MMOTOR_LENGTH 0.01
#define DEFAULT_MMOTOR_WIDTH 0.03
#define DEFAULT_MMOTOR_HEIGHT 0.01

#define DEFAULT_MMOTOR_X (DEFAULT_BODY_X + 0.06)
#define DEFAULT_MMOTOR_Y DEFAULT_BODY_Y
#define DEFAULT_MMOTOR_Z DEFAULT_BODY_Z

#define DEFAULT_MMOTOR_MASS 0.01


#define DEFAULT_ULTRASONICSENSOR_LENGTH 0.01
#define DEFAULT_ULTRASONICSENSOR_WIDTH 0.03
#define DEFAULT_ULTRASONICSENSOR_HEIGHT 0.01


#define DEFAULT_ULTRASONICSENSOR_X (DEFAULT_MMOTOR_X + 0.05)
#define DEFAULT_ULTRASONICSENSOR_Y DEFAULT_MMOTOR_Y
#define DEFAULT_ULTRASONICSENSOR_Z (DEFAULT_MMOTOR_Z + 0.06)

#define DEFAULT_ULTRASONICSENSOR_MASS 0.01

#define DEFAULT_MAX_ULTRASONICSENSOR_DISTANCE 100.0


#define DEFAULT_BLOCK_MASS 100


#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif





/**
* @struct MyLink
*@brief ボディオブジェクト
*接続するジョイントも含む
*/
typedef struct {
  dBodyID  body;
  dGeomID  geom;
  dJointID joint;
  int  dir;
  float  red, green, blue;
  dReal    m,r,x,y,z,lx,ly,lz, the, dthe, ddthe, axisx, axisy, axisz, Tthe, Tdthe, Tddthe, tau, jx,jy,jz;
} MyLink;

/**
* @class EV3_Obj
*@brief EV3の各種データ格納クラス
*/
class EV3Obj
{
public:
	EV3Obj();
	void setTargetVelocity(double vx, double va);
	void setCurrentPosition(double px, double py, double pa);
	void setUltrasonicSensorPos(double x, double y, double z);
	void setColourSensorPos(double x, double y, double z);
	double calcUltrasonicSensorDistance(double x, double y, double z);
	double calcColourSensorDistance(double x, double y, double z);
	void resetUltrasonicSensorDistance();
	void resetColourSensorDistance();
	bool getRightTouch(double limit);
	bool getLeftTouch(double limit);
	double target_vx, target_va;
	double current_px, current_py, current_pa;
	double gyro_data;
	double target_mangle, velocity_mmotor;
	double current_ultrasonicSensorData, current_colourSensorData;
	double right_touch_value, left_touch_value;
	bool right_touch, left_touch;
	double current_ultrasonicSensor_x, current_ultrasonicSensor_y, current_ultrasonicSensor_z;
	double current_colourSensor_x, current_colourSensor_y, current_colourSensor_z;
};
/**
* @class EV3SimulatorObj
*@brief シミュレーションの操作をするためのクラス
*/
class EV3SimulatorObj
{
public:
	/**
	*@brief コンストラクタ
	*/
	EV3SimulatorObj();
	/**
	*@brief デストラクタ
	*/
	~EV3SimulatorObj();
	coil::Mutex mu;
	MyLink EV3Block, wheelLeft, wheelRight, wheelBall, touchSensorLeft[4], touchSensorRight[4], gyroSensor[3], colourSensor, mmotor, ultrasonicSensor[3];
	dGeomID ultrasonicSensor_ray, colourSensor_ray;
	std::vector<MyLink> blocks;
	bool plane_exist;
	MyLink plane;
	double st;
	double gravity;
	bool pause;
	dWorldID      world;       
	dSpaceID      space;       
	dGeomID       ground;       
	dJointGroupID contactgroup;
	EV3Obj ev3;

	
	/**
	*@brief 超音波距離センサを設定
	*/
	void setUltrasonicSensorRay();
	/**
	*@brief カラーセンサを設定
	*/
	void setColourSensorRay();
	/**
	*@brief 各パラメータの初期化を行う
	*@param offset_z 高さを調整
	*/
	void makeParam(double offset_z=0);
	/**
	*@brief 直方体作成
	* @param body ボディオブジェクト
	*/
	void setBox(MyLink *body);
	/**
	*@brief 円柱作成
	* @param body ボディオブジェクト
	*/
	void setCylinder(MyLink *body);
	/**
	*@brief 球作成
	* @param body ボディオブジェクト
	*/
	void setSphere(MyLink *body);
	/**
	*@brief ヒンジジョイント作成
	* @param body1 ボディ1
	* @param body2 ボディ2
	*/
	void setHinge(MyLink *body1, MyLink *body2);
	/**
	*@brief スライダージョイント作成
	* @param body1 ボディ1
	* @param body2 ボディ2
	*/
	void setSlider(MyLink *body1, MyLink *body2);
	/**
	*@brief 固定ジョイント作成
	* @param body1 ボディ1
	* @param body2 ボディ2
	*/
	void setFixed(MyLink *body1, MyLink *body2);
	/**
	*@brief ボールジョイント作成
	* @param body1 ボディ1
	* @param body2 ボディ2
	*/
	void setBall(MyLink *body1, MyLink *body2);
	/**
	*@brief 全ボディ、接続する全ジョイント生成
	*/
	void makeRobot();
	/**
	*@brief ヒンジジョイント制御
	* @param body ボディオブジェクト
	* @param theta ヒンジジョイントの位置
	*/
	void controlHinge(MyLink *body, dReal theta);
	/**
	*@brief ヒンジジョイント制御
	* @param body ボディオブジェクト
	* @param theta ヒンジジョイントの位置
	* @param vel ヒンジジョイントの速度
	*/
	void controlHinge(MyLink *body, dReal theta, dReal vel);
	/**
	*@brief スライダージョイント制御
	* @param body ボディオブジェクト
	* @param length スライダージョイントの位置
	*/
	void controlSlider(MyLink *body, dReal length);
	/**
	*@brief 全ジョイント制御
	*/
	void control();
	/**
	*@brief 更新
	*/
	void update();
	/**
	*@brief 全ボディ、接続する全ジョイント消去
	*/
	void destroyRobot();

	/**
	*@brief 接触コールバック
	* @param o1 ジオメトリ1
	* @param o2 ジオメトリ2
	*/
	void m_nearCallback(dGeomID o1, dGeomID o2);
	/**
	*@brief 刻み幅設定
	* @param s サンプリング時間
	*/
	void setSamplingTime(double s);


	/**
	*@brief 地面生成
	*/
	void makePlane(double x, double y, double lx, double ly, double lz);

	/**
	*@brief 障害物生成
	*/
	void makeBlock(double x, double y, double z, double lx, double ly, double lz, double r);

	/**
	*@brief ファイルから障害物は位置読み込み
	*@return 読み込み成功(true)、失敗(false)
	*/
	bool loadBlocksData(std::string fname);


};



#endif