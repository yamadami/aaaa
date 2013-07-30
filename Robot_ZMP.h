/*****************************************************************************************/
//                          ロボット派生PIDコントローラ搭載クラス	
/*****************************************************************************************/
//							2013/07/19 高さ0.20上がっている状態
//
/*****************************************************************************************/



class Robot_ZMP : public Robot
{
private:


public:

	double refVel;
	double refHight;
	double poszErrInteg;

	//******************************追加************************************************//
	double Leg_point[1000][3];			//Leg_point[][0]は左右確認,[][1]はx座標[][2]はy座標 calcZMP,simLoop内で使用
	#define I 100						//歩行周期　歩行回数
	#define Ts 50						//一歩中の分割数　
	#define O   1						//一歩中の分割数の表示したい割合　　Ts50、O5ならTsの5の倍数分だけ表示
	#define zc 0.05				//拘束平面の高さ 多分任意に決定する
	double Pxfix[I] ;					//修正した着地位置(各歩の着地位置)
	double Pyfix[I] ;					//修正した着地位置
	double Point_x,Point_y,Point_z;		//左足の値
	double Point_xr,Point_yr,Point_zr;	//右足の値
	double sohenx[I][Ts], soheny[I][Ts];//歩行素片の値 [M][N]M歩目のN(1~50)番目
	double kari_y[I][Ts];
	double endx[I], endy[I];			//
	int count ;
	
	//**********************************************************************************//

	//**************************追加**********************************************//
	//void move_Foot(double l, double m, double n, int L_or_R);
	void simloop(Robot_ZMP pre);
	void move_Foot(double l,double m,double n, int L_or_R);
	void simloop();
	void calcZMP(); 
	//*****************************************************************************//


	Robot_ZMP():Robot()
	{
		init_PID();
	}

	void init_PID()
	{
		//refVel = 0.3;
		//refHight=0.4;
		//poszErrInteg=0;
		//count = 0;
		//**************************//
		//	ここで最初に計算する	//
		calcZMP();
		//**************************//
		//	for(int e=1;e<200;e++){
		//printf("Leg_point=%lf\n",Leg_point[e][0]);
	//}
	}

	
	void controler(Robot_ZMP pre, double steptime, int steps);

	bool changeFootX(Robot_ZMP pre);
	bool changeFootY(Robot_ZMP pre);

	void setMotor(int steps);
	void Ashisaki();

};


/*****************************************************************************************/
// コントローラー(メインループ)
/*****************************************************************************************/
void Robot_ZMP::controler(Robot_ZMP pre, double steptime, int steps){
	// step1 ロボットの状態取得
	simloop(pre);
	// step2 モータの角度を取得
	// 略
	// step3 重心情報の取得

	/*
	for(int i=0;i<I;i++){
		for(int T=0;T<Ts;T++){
			printf("%lf\n",kari_y[i][T]);
		}
		printf("%d\n",i);
	}
	*/


	//**********************************************//
	//			ここに計算はいらない				//
	//calcGlPos();				// 重心計算
	//calcGlVel(pre,steptime);	// 重心速度計算
	//calcGlAcc(pre, steptime);	// 重心加速度計算
	//**********************************************//


	// 着地目標位置(x,y,z)と足先が一定値(10e-2)以上離れていた場合は
	// 定数倍(<1)を使って目標値を刻んでいく．
	//Ashisaki();	//使わない方が安定するぽい

	// step5 伸長力の計算
	//double f = shinchouryoku(pre);
	//printf("f=%lf\n",f);

	// step6 遊脚・支持脚の関節角度を求める
	//calcRefJoint(pre, refPos);

	// step7 モータ・スライダに力を設定する
	setMotor(steps);
}

/*****************************************************************************************/
// 逆運動学＋モータの角度セット.(山田)
/*****************************************************************************************/

void Robot_ZMP::move_Foot(double l,double m,double n, int L_or_R){
	

	//printf("aa");
	//************逆運動学、足の関節角度の決定***//
	InverseKinematics(l, m, n, L_or_R);
	//*******************************************//


	//****************今ここはいらない********************//
	/*
	if( L_or_R == 0 ){
		epwm_set( 0, 0,theta[1]-3.14/36);//左腰（左足ヨー角
		epwm_set( 0, 1,theta[2]);//左太もも裏（左足ロール
		epwm_set( 0, 2,theta[3]);//左太もも上部
		epwm_set( 0, 3,theta[4]);//左太もも下部
		epwm_set( 0, 4,theta[5]);//左ひざ
		epwm_set( 0, 5,theta[6]);//左足首裏（足首ロール
	}
	if( L_or_R == 1 ){
		epwm_set( 1, 0,theta[1]+3.14/36);//右腰（右足ヨー角
		epwm_set( 1, 1,theta[2]);//右太もも裏（右足ロール
		epwm_set( 1, 2,theta[3]);//右太もも上部
		epwm_set( 1, 3,theta[4]);//右太もも下部
		epwm_set( 1, 4,theta[5]);//右ひざ
		epwm_set( 1, 5,theta[6]);//右足首裏（足首ロール
	}
	*/
	//*****************************************************//

}

/*****************************************************************************************/
// ロボットの状態変数の更新する.(山田)
/*****************************************************************************************/
// ロボットの状態変数の更新
void Robot_ZMP::simloop(Robot_ZMP pre){
	static int step_time =0;		//足を上げる際の刻み回数
	static int wait = 3;			//wait * 6 msec::: 0.2sec(33)
	static int step=0;				//時間*1msec
	static int count = 0;
	static int count_step = 0;			//歩数カウント
	double height_u=-0.40;//0.16;	//u遊脚の持ち上げる高さ
	static double flag_Leg =0;		//切換えフラグ
	double stride_z=0.04,d=0.007;	// stride_z高さの刻み幅 d距離の刻み幅
	//int count= 0;	
	//Leg_point[step][0] == 0なら右支持状態、1なら左支持状態へ

	step ++;
	step_time ++;

	//printf("stat=%d\n",stat);
	//printf("Leg_point=%lf\n",Leg_point[step][0]);
	
	//flag_Leg = Leg_point[step][0];
	//printf("%lf\n",soheny[count_step][1]);
	//printf("%d",count_step);
	//Point_x = Leg_point[step][1];
	//Point_xr = Leg_point[step][1];
	//Point_x = 0.0;
	//Point_xr = 0.0;
	

	
	//for(int i=0;i<I;i++){
	//	for(int T=0;T<Ts;T++){
	//		int i = 2;		
	//		printf("%lf\n",kari_y[count_step][step_time]);
	//	}
	//}
	

	switch (stat) {

	//**********************初期状態維持50step分***********************************//
	//**********************最初以外ここには入らない*******************************//

	case 0: 
		//Point_x = 0.0;
		Point_y = 0.04;
		Point_z = 0.20;
		//Point_xr = 0.0;
		Point_yr = -0.04;
		Point_zr = 0.00;

		//move_Foot(Point_x,Point_y,Point_z,0);
		//move_Foot(Point_xr,Point_yr,Point_zr,1);
		step_time = 0;

		count++ ;
		if(count > 50){
			printf("case0=ok\n");
			stat = 1;
			count = 0;
		}
		break;

	

	//****************************右足支持中 *****************************//
	case 1:
		//
		if(step_time<25){
			Point_z= Point_z + stride_z ; // ????????????????
			//左足が(歩行素片の高さ－遊脚の持ち上げる高さ)よりも大なら---------
			if(Point_z > zc-height_u){
				Point_z = zc-height_u;
			}
		}
		else{
			Point_z =Point_z - stride_z;
			if(Point_z < zc){
				Point_z = zc;
			}
		}
		Point_y =  soheny[count_step][step_time]+ 0.04;
		Point_yr = soheny[count_step][step_time]-0.04;
		Point_zr = zc;
		//Point_x = 0.0;
		//Point_xr = 0.0;
		
		//****************仮の値*********************//
		/*
		Point_y = -0.04;
		Point_yr = 0.04;
		Point_x = 0.0;
		Point_xr = 0.0;
		Point_z = zc ;
		Point_zr = zc ;
		count++ ;
		*/
		//*******************************************//
		move_Foot(Point_x,Point_y,Point_z,0);
		move_Foot(Point_xr,Point_yr,Point_zr,1);


		

		if(Leg_point[step][0] == 0){
			step_time = 0;
			count_step++;
			stat = 2;
			printf("case1=ok\n");
		}
		break;

	//****************************右足支持から左足支持に切り替え中 *****************************//
	case 2: 
		Point_yr = soheny[count_step][step_time]-0.04;

		Point_y = soheny[count_step][step_time]+0.04;

		//****************仮の値*********************//
		
		//Point_y = -0.04;
		//Point_yr = 0.04;
		//Point_x = 0.0;
		//Point_xr = 0.0;
		Point_z = zc ;
		Point_zr = zc ;
		//count++ ;
		
		//*******************************************//
		move_Foot(Point_x,Point_y,Point_z,0);
		move_Foot(Point_xr,Point_yr,Point_zr,1);


		if(Leg_point[step][0] == 0){
			stat = 3;
			printf("case2=ok\n");
		}
		break;

	//****************************左足支持中 *************************************************//
	case 3:
		//



		if(step_time<25){
			Point_zr= Point_zr+stride_z ; // ????????????????
			//左足が(歩行素片の高さ－遊脚の持ち上げる高さ)よりも大なら---------
			if(Point_zr> zc-height_u){
				Point_zr = zc-height_u;
			}
		}
		else{
			Point_zr =Point_zr-stride_z;
			if(Point_zr < zc){
				Point_zr = zc;
			}
		}

		Point_y = soheny[count_step][step_time]+0.04;
		Point_z = zc;
		Point_yr = soheny[count_step][step_time]-0.04;

		//****************仮の値*********************//
		/*
		Point_y = -0.04;
		Point_yr = 0.04;
		Point_x = 0.0;
		Point_xr = 0.0;
		Point_z = zc ;
		Point_zr = zc ;
		count++ ;
		*/

		//Point_x = 0.0;
		//Point_xr = 0.0;
		//*******************************************//
		//move_Foot(Point_x,Point_y,Point_z,0);
		move_Foot(Point_xr,Point_yr,Point_zr,1);


		if(Leg_point[step][0] == 1){
			step_time = 0;
			count_step++;
			stat = 4;
			printf("case3=ok\n");
		}
		break;	

//********************左足支持から右足支持に切り替え中******************************//
	case 4: 
		
		//*******************************************//
		//****************仮の値*********************//
		//Point_y = -0.04;
		//Point_yr = 0.04;
		//Point_x = 0.0;
		//Point_xr = 0.0;
		Point_z = zc ;
		Point_zr = zc ;
		//count++ ;
		
		//*******************************************//

		Point_y = soheny[count_step][step_time]+0.04;
		Point_yr =soheny[count_step][step_time];
		//printf("%d\n",Leg_point[step][0]);

		move_Foot(Point_x,Point_y,Point_z,0);
		move_Foot(Point_xr,Point_yr,Point_zr,1);
			if(Leg_point[step][0] == 1){
				printf("case4=ok\n");
				stat = 1;	
			}
		break;
		


	}
}
	


///*************************************************************************************/
//	関数作り(山田)6.27
///*************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// ここから////////////////////////////////////////////
//double Ts =50;	//プログラム用歩行周期
double Tsup = 0.01*Ts*2; //歩行周期

//double sohenx[I][Ts], soheny[I][Ts], sohenz[I][Ts];
//double sohenvx[I][Ts], sohenvy[I][Ts];

//zcをどうするのか？？

void Robot_ZMP::calcZMP(){
	int i, j, fugou;
	double Tc, C, S, D;				//定数
	double a = 10.0, b = 1.0;		//評価関数の重み  a位置　b速度
	double Px[2], Py[2];			//着地位置
	double sx[I], sy[I];			//前進方向,左右方向の歩幅
	double xbar, ybar;				//歩行素片の位置
	double vxbar, vybar;			//歩行素片の終端速度
	double xi[2], yi[2];			//歩行開始瞬間の重心位置
	double xidot[2], yidot[2];		//歩行開始瞬間の速度
	double xd, xddot, yd, yddot;	//目標状態の位置と速度
	
	//定数定義
	Tc = sqrt(zc/9.8);
	C  = cosh(Tsup/Tc);
	S  = sinh(Tsup/Tc);
	D  = a*(C-1)*(C-1) + b*(S/Tc)*(S/Tc);

	for (i = 0 ; i < I ; i++){

		if(i==0 || i==I-2){sx[i] = 0.0;}
		else{sx[i] = 0.2;}

		sy[i] = 0.04;
	}
	
	Px[0]=0.0;
	Py[0]=0.0;

	//歩行開始瞬間の重心位置
	xi[0] = 0.0;
	xidot[0] = 0; 

	yi[0] = 0.0/*0.1*/;
	yidot[0] = 0;

	//初期足位置補正
	Pxfix[0] =   0.0;
	Pyfix[0] =   0.0;
	endx[0]=0.0;
	endy[0]=0.0;
	/*******************************************************/
	/*ループ開始*/
	for (i = 0 ; i < I ; i++){
		//符号決定
		fugou = (int)pow(-1.0,i+2);
		
		//計画した着地位置
		if(i>0){
			Px[1] =  Px[0] + sx[i];
			Py[1] =  Py[0] - fugou * sy[i];
		}else{
			Px[1]=Px[0];
			Py[1]=Py[0];
		}
		//一歩前の着地位置　保存用
		Px[0] =  Px[1];
		Py[0] =  Py[1];

		//歩行素片のパラメータ
		xbar = sx[i] / 2;
		ybar = fugou * sy[i] / 2;

		//歩行素片の終端速度
		vxbar = xbar * (C+1.00) /(Tc*S);
		vybar = ybar * (C-1.00) /(Tc*S);

		//(4.54式
		if(i>0){
			xi[1] = (C*xi[0]) + (Tc*S*xidot[0]) + (1-C)*Pxfix[i-1];
			xidot[1] = (S*xi[0]/Tc) + (C*xidot[0]) + (-S/Tc)*Pxfix[i-1];

			yi[1] = (C*yi[0]) + (Tc*S*yidot[0]) + (1-C)*Pyfix[i-1];
			yidot[1] = (S*yi[0]/Tc) + (C*yidot[0]) + (-S/Tc)*Pyfix[i-1];
		}else{
			xi[1]=xi[0];xidot[1]=xidot[0];
			yi[1]=yi[0];yidot[1]=yidot[0];
		}
		xi[0] = xi[1];
		xidot[0] = xidot[1];

		yi[0] = yi[1];
		yidot[0] = yidot[1];

		//目標状態の位置と速度
		xd = Px[1] + xbar;
		xddot = vxbar;

		yd = Py[1] + ybar;
		yddot = vybar;
		
		//修正着地位置

		Pxfix[i] = - (a*(C-1)*(xd -(C*xi[1]) -(Tc*S*xidot[1])) / D)
					- (b*S*(xddot - (S*xi[1]/Tc) -(C*xidot[1])) / (Tc*D));

		Pyfix[i] = - (a*(C-1)*(yd -(C*yi[1]) -(Tc*S*yidot[1])) / D)
					- (b*S*(yddot - (S*yi[1]/Tc) -(C*yidot[1])) / (Tc*D));
		//printf("%lf\n",Pxfix[i]);
		if(i>0){
			endx[i]=Pxfix[i];
			endy[i]=Pyfix[i];
		}

		//printf("%lf\n",Pxfix[i]);
		//歩行素片の軌道を計算
		double t;

		for( j = 0 ; j < Ts ; j++ ){
			t=Tsup/Ts*j;
			sohenx[i][j] = (xi[1]-Pxfix[i]) * cosh(t/Tc) + Tc * xidot[1] * sinh(t/Tc) + Pxfix[i];
			soheny[i][j] = (yi[1]-Pyfix[i]) * cosh(t/Tc) + Tc * yidot[1] * sinh(t/Tc) + Pyfix[i];//これ反転させるべき？
			kari_y[i][j] = soheny[i][j];
			//printf("%lf\n",sohenx[i][j]);
		}

	}
	int e=0;
	Leg_point[0][0]=0;
	for(i=0;i<I;i++){
		for(j=0;j<Ts;j+=1){
			if((i%2)==0){
				Leg_point[e][0]=0;
			}
			else{
				Leg_point[e][0]=1;
			}
			//Leg_point[e][1]=(endx[i]-sohenx[i][j]);
			//Leg_point[e][2]=(endy[i]-soheny[i][j]);
			Leg_point[e][1]=(sohenx[i][j]);
			Leg_point[e][2]=(soheny[i][j]);		

			e++;
			//printf("%lf\n",Leg_point[e][0]);
	//		printf("%lf\n",Leg_point[e][1]);
			
		}
	}
	
	//Leg_point[e][0]=2;
	Leg_point[e][1]=0;
	//Leg_point[e][2]=0;

	for(i=0;i<5000;i++){
	printf("%lf\n",Leg_point[i][1]);
	}
}

//****************************多分使わない*********************************************//
// 遊脚支持脚切り替え判定ｘ
bool Robot_ZMP::changeFootX(Robot_ZMP pre){
	double k = 0.12 * getGlVelX() + 0.10 * (refVel-getGlVelX()) + 0.02;
	//printf("k=%lf\n",k);

	//  　　  重心位置X   > 条件値　&& 　　　　　　外に倒れている
	if( fabs(getGlPosX()) > fabs(k) && ( getSitaZ() - getpre_SitaZ() ) > 0 ){	//&& ( getSitaZ() - pre.getSitaZ() )
		return true;
	}else{
		return false;
	}
}

//**************************************************************************************///

//****************************多分使わない**********************************************//
// 遊脚支持脚切り替え判定ｙ
bool Robot_ZMP::changeFootY(Robot_ZMP pre){
	double k = 0.05 ;//0.05;

	//        重心位置Y　 > 条件値  &&          外に倒れている　　　 &&  重心が支持脚と逆に進んでいる
	if( fabs(getGlPosY()) > fabs(k) && (getSitaZ()-getpre_SitaZ()>0) && (getL_or_R()*getGlVelY()>0.0) ){	//&& (getSitaZ()-pre.getSitaZ()>0)
		return true;
	}else{
		return false;
	}
}
//***************************************************************************************//


//*******************************これは使う？使わない？**********************************//
/*****************************************************************************************/
// モータ・スライダに力を設定する
/*****************************************************************************************/
void Robot_ZMP::setMotor(int steps){
	double f_Yuukyaku=0;
	double ref_Yuukyaku_length=0;
	count = steps;
	if(steps < 50){
		
		// 初期姿勢を取らせる
		stat=0;
		init_refjoint();
		// 計算結果から力を設定する
		for(int i=0;i<6;i++){
			setHingeParams(LegLJoint[i], refLegLJointAngle[i]);
			setHingeParams(LegRJoint[i], refLegRJointAngle[i]);
		}
	

		/*
		// 一歩目が前進になるように、0.1秒間力を加える（背中を押す）。要改良。
		if(40 < steps && steps < 50){
			//dBodySetForce(Body[0].body,2,0,0);
			// 左脚フリー
			dJointSetHingeParam(LegLJoint[4], dParamFMax, 0);
			//dJointSetHingeParam(LegLJoint[5], dParamFMax, 0);
			// 右足フリー
			dJointSetHingeParam(LegRJoint[4], dParamFMax, 0);
			//dJointSetHingeParam(LegRJoint[5], dParamFMax, 0);
		}	
		*/
	}else{
		
		// 計算結果から力を設定する
		for(int i=0;i<6;i++){
			setHingeParams(LegLJoint[i], l_theta[i]);
			setHingeParams(LegRJoint[i], r_theta[i]);
		}
		
		// 基本形として支持脚の足首はフリー
		//if(getChuukuu() == 0){
		//	if( getL_or_R() == -1 ){
				// 左脚フリー
				//printf("左脚フリー\n");
		//		dJointSetHingeParam(LegLJoint[4], dParamFMax, 0);
		//		dJointSetHingeParam(LegLJoint[5], dParamFMax, 0);
		//	}else{
				// 右足フリー
				//printf("右脚フリー\n");
		//		dJointSetHingeParam(LegRJoint[4], dParamFMax, 0);
		//		dJointSetHingeParam(LegRJoint[5], dParamFMax, 0);
		//	}
		}
		

}

//******************************************************************************************//

//*****************************多分使わないはず*********************************************//
void Robot_ZMP::Ashisaki(){
	// 着地目標位置(x,y,z)と足先が一定値(10e-2)以上離れていた場合は
	// 定数倍(<1)を使って目標値を刻んでいく．
	
	if(getL_or_R() == -1){
		ashiSaki[0] = getPartsPosX(LegR[7].body);
		ashiSaki[1] = getPartsPosY(LegR[7].body);
		//ashiSaki[2] = 0;//getPartsPosZ(LegR[7].body);
		ashiSaki[2] = getPartsPosZ(LegR[7].body);

		vec_minus(workDistance, refPos, ashiSaki);
		workDistance[3] = length(workDistance);
		//for(int i=0;i<4;i++){printf("workDistance[%d]=%lf\n",i,workDistance[i]);};
		//for(int i=0;i<4;i++){printf("ashiSaki[%d]=%lf\n",i,ashiSaki[i]);};
		if(workDistance[3]>10e-2){
			printf("workDistance[3]=%lf>10e-2\n",workDistance[3]);
			a_vec( work_a_distance  , 5e-3, work_distance );	//２番目の引数は刻み倍率
			vec_add( work_chaku_moku,  work_a_distance, ashiSaki );
			vec_copy( refPos, work_chaku_moku );
			//for(int i=0;i<4;i++) printf("work_chaku_moku=%lf\n",work_chaku_moku[i]);
		}

	}else{
		ashiSaki[0] = getPartsPosX(LegL[7].body);
		ashiSaki[1] = getPartsPosY(LegL[7].body);
		//ashiSaki[2] = 0;//getPartsPosZ(LegL[7].body);
		ashiSaki[2] = getPartsPosZ(LegL[7].body);

		vec_minus(workDistance, refPos, ashiSaki);
		workDistance[3] = length(workDistance);
		//for(int i=0;i<4;i++){printf("workDistance[i]=%lf\n",workDistance[i]);};
		if(workDistance[3]>10e-2){
			printf("workDistance[3]=%lf>10e-2\n",workDistance[3]);
			a_vec( work_a_distance  , 5e-3, work_distance );	//２番目の引数は刻み倍率
			vec_add( work_chaku_moku,  work_a_distance, ashiSaki );
			vec_copy( refPos, work_chaku_moku );
			//for(int i=0;i<4;i++) printf("work_chaku_moku=%lf\n",work_chaku_moku[i]);
		}
	}
}
//*****************************************************************************************//