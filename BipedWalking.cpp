/*****************************************************************************************/
//                           二足歩行ロボットシミュレーション	
/*****************************************************************************************/
//■2013/01/31 プロジェクト作成、すっきりさせるために一から書き直し
//■2013/02/10 ようやく物になってきた、コントローラを実装中
//■2013/05/31 SHR版に改良する。ちゃんと歩くようになったが、
//
/*****************************************************************************************/


//=======================================================================================//
//                                 インクルードヘッダ
//=======================================================================================//
#include <ode/ode.h>				// OpenDynaamicEngine
#include <drawstuff/drawstuff.h>	// 描画用ライブラリ
#include <cstdlib>					// C++一般ユーティリティ
#include <cstdio>					// C++入出力関数
#include <cmath>					// C++数学関数

//=======================================================================================//
//                            ODE変数と自作関数・クラスの宣言
//=======================================================================================//
// 短精度と倍精度に対応するためのおまじない
#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

// 物理エンジン用の各種構造体を作成（書き換え不可）
dWorldID world;					// 動力学計算用ワールド
dSpaceID space;					// 衝突検出用スペース
dGeomID  ground;				// 地面
dJointGroupID contactgroup;		// コンタクトグループ
dsFunctions fn;					// drawstuffで使う構造体

// ファイルポインタ
FILE *fp;



// 自作インクルードヘッダ
#include "InverseKinematics.h"
#include "Functions.h"			// 自作関数
#include "Robot.h"				// ロボットの設定
#include "Robot_ZMP.h"			// ロボットの設定
#include "createRobot.h"		// ロボットの作成



//=======================================================================================//
//                                   インスタンスの作成
//=======================================================================================//
#define ROBOT_NUM 1
Robot_ZMP robo[ROBOT_NUM];		// 二足歩行ロボットのインスタンス
Robot_ZMP pre[ROBOT_NUM];		// 二足歩行ロボットのインスタンス（一ステップ前状態保存用）

//Robot_ZMP robo2;		// 二足歩行ロボットのインスタンス
//Robot_ZMP pre2;		// 二足歩行ロボットのインスタンス（一ステップ前状態保存用）


const double STEPTIME = 0.01;	// シミュレーションステップ
int STEPS=0;					// シミュレーション時間

//=======================================================================================//
//                                       衝突検出
//=======================================================================================//
void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 7;     // 接触点数
  dContact contact[N];

  int isGround = ((ground == o1) || (ground == o2));

  // 2つのボディがジョイントで結合されていたら衝突検出しない
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;

  //if( b1 == robo.LegL[4].body || b2 == robo.LegL[4].body ){ return; }
  //if( b1 == robo.LegL[5].body || b2 == robo.LegL[5].body ){ return; }
  //if( b1 == robo.LegR[4].body || b2 == robo.LegR[4].body ){ return; }
  //if( b1 == robo.LegR[5].body || b2 == robo.LegR[5].body ){ return; }

  int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (isGround)  {
    for (int i = 0; i < n; i++) {
      contact[i].surface.mode   = dContactBounce | dContactSoftERP |
                                  dContactSoftCFM;
      contact[i].surface.soft_erp   = 0.10;   // 接触点のERP
      contact[i].surface.soft_cfm   = 0.001;//0.001; // 接触点のCFM
	  contact[i].surface.mu     = dInfinity; // 摩擦係数:無限大
      dJointID c = dJointCreateContact(world,
                                       contactgroup,&contact[i]);
      dJointAttach (c,dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

void LogPrint(){
	/*
	// コマンドプロンプト出力
	printf("------------------------------------\n");
	printf("STEPS:%d\n",STEPS);
	printf("stat:%d\n",robo.stat);
	printf("L_or_R=%d\n",robo.getL_or_R());
	printf("hosuu:%d\n",robo.hosuu);
	printf("GlVelX=%lf\n",robo.getGlVelX());
	printf("GlPosX=%lf\n",robo.getGlPosX());
	printf("GlPosY=%lf\n",robo.getGlPosY());
	printf("GlPosZ=%lf\n",robo.getGlPosZ());
	printf("sitaZ=%lf\n",robo.getSitaZ());
	printf("pre_sitaZ=%lf\n",robo.getpre_SitaZ());
	printf("refPos[0]=%lf\n",robo.refPos[0]);
	printf("refPos[1]=%lf\n",robo.refPos[1]);
	printf("\n");
	*/
}

void LogfPrint(){
	/*
	// コマンドプロンプト出力
	static int count = 0;
	if(count == 0){
		fprintf(fp,"STEPS,GlVelX,refVelX\n");
	}
	fprintf(fp,"%d,%lf,%lf\n",STEPS,robo.getGlVelX(),robo.refVel);
	count++;
	*/
}

//=======================================================================================//
//                                 シミュレーションループ
//=======================================================================================//
void simLoop(int pause)
{

	if (!pause) {


		for(int i=0;i<ROBOT_NUM;i++)robo[i].controler(pre[i], STEPTIME, STEPS);
		for(int i=0;i<ROBOT_NUM;i++)pre[i] = robo[i];

		//robo2.controler(pre2, STEPTIME, STEPS);
		//pre2 = robo2;

		LogfPrint();
		//LogPrint();
		//printf("STEPS=%d\n",STEPS);

		STEPS++;
		dSpaceCollide(space,0,&nearCallback);
		dWorldStep(world,STEPTIME);
		dJointGroupEmpty(contactgroup);
	}

	for(int i=0;i<ROBOT_NUM;i++)robo[i].drawPendulum();
	for(int i=0;i<ROBOT_NUM;i++)robo[i].drawRobot();
	for(int i=0;i<ROBOT_NUM;i++)robo[i].drawRefPos();

	for(int i=0;i<ROBOT_NUM;i++)fprintf(fp,"%lf\n",robo[i].Leg_point);
	//robo2.drawPendulum();
	//robo2.drawRobot();
	//robo2.drawRefPos();
}


//=======================================================================================//
//                                   リスタート関数
//=======================================================================================//
void restart()
{
	STEPS = 0;
	for(int i=0;i<ROBOT_NUM;i++)robo[i].init();
	for(int i=0;i<ROBOT_NUM;i++)robo[i].init_PID();
	//robo[0].init_PID();
	//robo2.init();
	//robo2.init_PID();

	for(int i=0;i<ROBOT_NUM;i++)robo[i].destroyRobot();				// ロボットの破壊
	//robo2.destroyRobot();				// ロボットの破壊

	dJointGroupDestroy(contactgroup);		// ジョイントグループの破壊
	contactgroup = dJointGroupCreate(0);	// ジョイントグループの生成

	for(int i=0;i<ROBOT_NUM;i++)robo[i].createRobot(0,i,80.0E-2);				// ロボットの生成
	//robo2.createRobot(0,2,80.0E-2);				// ロボットの生成
}

//=======================================================================================//
//                                     キー操作
//=======================================================================================//
void command(int cmd)
{
	switch (cmd) {
		//case 'd': robo.setL_or_R(1);					break;
		case 'r': restart();							break;
		//case 'a': robo.setxpu();						break;
		//case 's': robo.setxmi();						break;
		default : printf("key missed \n");				break;
	}
}

//=======================================================================================//
//                                     スタート関数
//=======================================================================================//
void start()
{
	//init = robo;
	static float xyz[3] = { -1.5, 0.0, 0.5};
	static float hpr[3] = { 0.0, 0.0, 0.0};
	dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
	dsSetSphereQuality(3);                 // 球の品質設定
}

//=======================================================================================//
//                                    描画関数の設定
//=======================================================================================//
void setDrawStuff()
{
	fn.version = DS_VERSION;    // ドロースタッフのバージョン
	fn.start   = &start;        // 前処理 start関数のポインタ
	fn.step    = &simLoop;      // simLoop関数のポインタ
	fn.command = &command;      // キー入力関数へのポインタ
	fn.path_to_textures = "../../drawstuff/textures"; // テクスチャ
}

//=======================================================================================//
//                                      メイン関数
//=======================================================================================//
int main (int argc, char *argv[])
{
	fp = fopen("data.csv","w");
	dInitODE();
	setDrawStuff();
	robo[0].init_PID();
	world        = dWorldCreate();
	space        = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);

	dWorldSetGravity(world, 0,0, -9.8);
	dWorldSetERP(world, 0.2);//0.9);          // ERPの設定
	dWorldSetCFM(world, 10E-3);//1e-5);         // CFMの設定
	ground = dCreatePlane(space, 0, 0, 1, 0);
	for(int i=0;i<ROBOT_NUM;i++)robo[i].createRobot(0,i,100.0E-2);
	//robo2.createRobot(0,2,80.0E-2);

	dsSimulationLoop (argc, argv, 640, 480, &fn);
	dWorldDestroy (world);
	dCloseODE();

	return 0;
}
