#include <fstream>
#include "DrawThread_EV3.h"
#include "searchFile.h"
#include <coil/stringutil.h>

//std::ofstream ofs( "test.txt" );

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox     dsDrawBoxD
#define dsDrawLine    dsDrawLineD
#define dsDrawTriangle    dsDrawTriangleD
#endif

DrawThread_EV3 *obj = NULL;


/**
*@brief シミュレーションの描画をするスレッドのコンストラクタ
*/
DrawThread_EV3::DrawThread_EV3(EV3SimulatorObj *so, double dt)
{
	m_so = so;
	

	setDrawStuff();

	fps = 1.0 / dt;

	obj = this;
	RCP_flag = false;
}

/**
*@brief シミュレーションループ
* @param pause 未使用
*/
void simLoop(int pause)
{
#ifdef WIN32
	Sleep(1000.0 / obj->fps);
#else
	struct timespec ts;
	ts.tv_sec = 1;
	ts.tv_nsec = 0;
	nanosleep(&ts, NULL);
#endif
	if(obj)
	{
		obj->drawRobot();
		obj->resetCameraPosition();
	}
}



/**
*@brief シミュレーション初期化
*/
void start()
{
  //float xyz[3] = {  .0f,  1.0f, 3.0f};  
  float xyz[3] = {  -0.5f,  -0.0f, 0.25f}; 
  float hpr[3] = {-180.0f, -10.0f, 0.0f};  
  //float hpr[3] = {0.0f, -90.0f, 90.0f}; 
  //float xyz[3] = {  5.0f,  -5.0f, 3.0f};
  //float hpr[3] = {180.0f, -10.0f, 0.0f};
  dsSetViewpoint(xyz,hpr);                
  dsSetSphereQuality(3);
  dsSetCapsuleQuality(6);


  
  
}

/**
*@brief スレッド実行関数
* @return
*/
int DrawThread_EV3::svc()
{
	int argc = 0;
	char *argv[] = {""};

	dsSimulationLoop(argc,argv,800,480,&fn);

	return 0;
}


/**
*@brief DrawStuff初期化
*/
void DrawThread_EV3::setDrawStuff()
{
	fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  
  static std::string drawstuff = search_file("drawstuff/textures", "PATH", ";");
  
  coil::replaceString(drawstuff, "\\", "/");
  
  
  if (drawstuff == "")
  {
	  fn.path_to_textures = "drawstuff/textures";
  }
  else
  {
	  fn.path_to_textures = drawstuff.c_str();
	  
  }

}

/**
*@brief 直方体描画
* @param body ボディオブジェクト
*/
void DrawThread_EV3::drawBox(MyLink *body)
{
	const double sides[3] = {body->lx, body->ly, body->lz};
	dsSetColorAlpha(body->red,body->green,body->blue,1.0);
	dsDrawBoxD(dBodyGetPosition(body->body),
						dBodyGetRotation(body->body),sides);
}

/**
*@brief 円柱描画
* @param body ボディオブジェクト
*/
void DrawThread_EV3::drawCylinder(MyLink *body)
{
	dsSetColorAlpha(body->red,body->green,body->blue,1.0);
	dsDrawCylinderD(dBodyGetPosition(body->body),
						dBodyGetRotation(body->body),body->lz,body->lx);
}


/**
*@brief 球描画
* @param body ボディオブジェクト
*/
void DrawThread_EV3::drawSphere(MyLink *body)
{
	dsSetColorAlpha(body->red, body->green, body->blue, 1.0);
	dsDrawSphereD(dBodyGetPosition(body->body),
		dBodyGetRotation(body->body), body->lz);
}

/**
*@brief 全ボディ描画
*/
void DrawThread_EV3::drawRobot()
{
	if(m_so->pause)
	{
		m_so->mu.lock();

		drawBox(&m_so->EV3Block);

	
		drawCylinder(&m_so->wheelLeft);
		drawCylinder(&m_so->wheelRight);
		drawSphere(&m_so->wheelBall);

		drawBox(&m_so->touchSensorLeft[0]);
		drawBox(&m_so->touchSensorLeft[1]);
		drawBox(&m_so->touchSensorRight[0]);
		drawBox(&m_so->touchSensorRight[1]);
		drawBox(&m_so->touchSensorLeft[2]);
		drawBox(&m_so->touchSensorRight[2]);

		drawBox(&m_so->touchSensorLeft[3]);
		drawBox(&m_so->touchSensorRight[3]);

		drawBox(&m_so->gyroSensor[0]);
		drawBox(&m_so->gyroSensor[1]);
		drawBox(&m_so->gyroSensor[2]);
		drawBox(&m_so->colourSensor);

		drawBox(&m_so->mmotor);
		drawBox(&m_so->ultrasonicSensor[0]);
		drawBox(&m_so->ultrasonicSensor[1]);
		drawBox(&m_so->ultrasonicSensor[2]);

		/*const double pos0[3] = { m_so->ev3.current_ultrasonicSensor_x, m_so->ev3.current_ultrasonicSensor_y, m_so->ev3.current_ultrasonicSensor_z };
		const double pos1[3] = { m_so->ev3.current_ultrasonicSensor_x, m_so->ev3.current_ultrasonicSensor_y, m_so->ev3.current_ultrasonicSensor_z - m_so->ev3.current_ultrasonicSensorData };
		dsDrawLine(pos0, pos1);*/

		if (m_so->plane_exist)
		{
			drawBox(&m_so->plane);
		}

		for (std::vector<MyLink>::iterator itr = m_so->blocks.begin(); itr != m_so->blocks.end(); ++itr) {
			drawBox(&(*itr));
		}

		m_so->mu.unlock();
	}
}



/**
*@brief カメラ位置再設定
*/
void DrawThread_EV3::resetCameraPosition()
{
	m_so->mu.lock();
	if (RCP_flag)
	{
		const dReal *pos = dBodyGetPosition(m_so->EV3Block.body);
		float xyz[3] = { -0.3f + pos[0], 0.3f + pos[1], 0.2f + pos[2] };
		float hpr[3] = { -40.0f, -30.0f, 0.0f };
		dsSetViewpoint(xyz, hpr);
		dsSetSphereQuality(3);
		dsSetCapsuleQuality(6);


		RCP_flag = false;

	}
	m_so->mu.unlock();
}


/**
*@brief カメラ位置再設定フラグを立てる
*/
void DrawThread_EV3::setRCPFlag()
{
	m_so->mu.lock();
	RCP_flag = true;
	m_so->mu.unlock();
}