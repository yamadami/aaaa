/*****************************************************************************************/
//                          ロボット派生PIDコントローラ搭載クラス	
/*****************************************************************************************/
//
//
/*****************************************************************************************/
class Robot_PID : public Robot
{
private:

public:
	//Robot_PID pre;
	double refVel;
	double refHight;



	double poszErrInteg;

	Robot_PID():Robot()
	{
		init_PID();
	}

	void init_PID()
	{
		refVel = 0.3;
		refHight=0.4;
		poszErrInteg=0;
	}

	void statusUpdate(Robot_PID pre);
	void controler(Robot_PID pre, double steptime, int steps);

	void statusUpdate();
	bool changeFootX(Robot_PID pre);
	bool changeFootY(Robot_PID pre);

	double refXCal();
	double refYCal();

	double shinchouryoku(Robot_PID pre);

	void setMotor(int steps, double f);

	void Ashisaki();
};


/*****************************************************************************************/
// コントローラー
/*****************************************************************************************/
void Robot_PID::controler(Robot_PID pre, double steptime, int steps){
	// step1 ロボットの状態取得
	statusUpdate(pre);
	// step2 モータの角度を取得
	// 略

	// step3 重心情報の取得
	calcGlPos();				// 重心計算
	calcGlVel(pre,steptime);	// 重心速度計算
	calcGlAcc(pre, steptime);	// 重心加速度計算

	// step4 速度から着地目標を計算

	//Sita計算
	for(int i=0;i<3;i++) pre_Sita[i] = Sita[i];	//前回状態保存 (statusUpdateに利用 preを用いると同じ値になってしまうので)
	calcSita();

	// 脚切替え直後はprev_sitaを現sitaで上書きする（これがないと不安定）
	if(pre.getL_or_R() != getL_or_R()){
		//setSitaX(pre_Sita[0]);
		//setSitaY(pre_Sita[1]);
		setSitaZ(pre_Sita[2]);
	}

	// 着地目標の計算
	if(getL_or_R()==-1){
		refPos[0] = refXCal()+getPartsPosX(LegL[7].body);
		refPos[1] = refYCal()+getPartsPosY(LegL[7].body);
		refPos[2] = 0.0;
	}else{
		refPos[0] = refXCal()+getPartsPosX(LegR[7].body);
		refPos[1] = refYCal()+getPartsPosY(LegR[7].body);
		refPos[2] = 0.0;
	}

	if( (stat == 3) || (stat == 8) ){
		// 半着地状態のときは，目標位置を変更しないようにする（ロボットが飛び跳ねる原因になるため）
		refPos[0] = pre.refPos[0];
		refPos[1] = pre.refPos[1];
	}

	if(pre.getL_or_R() != getL_or_R()){
		// 切り替え直後も変えない方がいいかも？ Glが大きく変わるから
		refPos[0] = pre.refPos[0];
		refPos[1] = pre.refPos[1];
	}

	if ( (stat == 2) || (stat == 3) || (stat == 7) || (stat == 8) ){
		refPos[2] = 0.0;
	}else{
		refPos[2] = 0.12;
	}

	// 着地目標位置(x,y,z)と足先が一定値(10e-2)以上離れていた場合は
	// 定数倍(<1)を使って目標値を刻んでいく．
	//Ashisaki();	//使わない方が安定するぽい

	// step5 伸長力の計算
	double f = shinchouryoku(pre);
	//printf("f=%lf\n",f);

	// step6 遊脚・支持脚の関節角度を求める
	calcRefJoint(pre, refPos);

	// step7 モータ・スライダに力を設定する
	setMotor(steps, f);
}

/*****************************************************************************************/
// ロボットの状態変数の更新する． 
/*****************************************************************************************/
// ロボットの状態変数の更新
void Robot_PID::statusUpdate(Robot_PID pre){
	int numOnSensorR = SensorRCheck();
	int numOnSensorL = SensorLCheck();
	//printf("SenR:%d\n",numOnSensorR);
	//printf("SenL:%d\n",numOnSensorL);

	int kirikae_jouken_flag=0;
	static int interval=0;

	// 状態の設定
	switch(stat){
	case 1: // 左足支持中 --------------------------------------------------------------/
		interval++;
		if( interval > 5 ){
			// 切り替え条件①
			if ( changeFootX(pre) ){
				kirikae_jouken_flag = 1;
				printf("ｘ切り替え\n");
			}
			// 切り替え条件②
			if ( changeFootY(pre) ){	
				kirikae_jouken_flag = 2;
				printf("ｙ切り替え\n");
			}

			// 上の切替え条件が成立した場合の処理
			if( kirikae_jouken_flag != 0 ){
				// 状態を進める
				stat = 2;
				interval =0; 
			}
		}
		break;

	case 2: // 右足着地動作中 ---------------------------------------------------
		interval++;
		if ( numOnSensorR >=1 ){
			// 状態を進める
			stat = 3;
			interval =0;
		}
		break;

	case 3: // 右足　半着地中 ---------------------------------------------------
		interval++;
		if ( numOnSensorR >= 4 ){
			// 状態を進める
			stat = 4;
			interval =0; 
		}
		break;

	case 4: // 両足支持中 ---------------------------------------------------
		interval++;
		if (interval > 1){
			// 状態を進める
			stat = 5;
			interval =0; 
		}
		break;

	case 5: // 左足持上げ中 ---------------------------------------------------
		interval++;
		if (interval > 5){
			// 状態を進める
			stat = 6;
			interval =0;
			// 歩数をカウントする
			hosuu++;
		}
		break;

	case 6: // 右足支持中 ---------------------------------------------------
		interval++;
		if( interval > 5 ){
			// 切り替え条件①
			if ( changeFootX(pre) ){
				kirikae_jouken_flag = 1;
				printf("ｘ切り替え\n");
			}
			// 切り替え条件②
			if ( changeFootY(pre) ){
				kirikae_jouken_flag = 2;
				printf("ｙ切り替え\n");
			}

			if(kirikae_jouken_flag != 0){
				// 状態を進める
				stat = 7;
				interval =0; 
			}
		}
		break;

	case 7: // 左足着地動作中 ---------------------------------------------------
		interval++;
		if ( numOnSensorL >=1 ){
			// 状態を進める
			stat = 8;
			interval =0; 
		}
		break;

	case 8: // 左足　半着地中 ---------------------------------------------------
		interval++;
		if ( numOnSensorL >= 4 ){
			// 状態を進める
			stat = 9;
			interval =0; 
		}
		break;

	case 9: // 両足支持中 ---------------------------------------------------
		interval++;
		if (interval > 1 ){
			// 状態を進める
			stat = 10;
			interval =0; 
		}
		break;

	case 10: // 右足持上げ中 ---------------------------------------------------
		interval++;
		if (interval >  5 ){
			// 状態を進める
			stat = 1;
			interval =0; 
			// 歩数をカウントする
			hosuu++;
		}
		break;

	}

	// 支持脚の設定
	if((stat < 4) || (stat > 8)){
		setL_or_R(-1);
	}else{
		setL_or_R(1);
	}
}


// 遊脚支持脚切り替え判定ｘ
bool Robot_PID::changeFootX(Robot_PID pre){
	double k = 0.12 * getGlVelX() + 0.10 * (refVel-getGlVelX()) + 0.02;
	//printf("k=%lf\n",k);

	//  　　  重心位置X   > 条件値　&& 　　　　　　外に倒れている
	if( fabs(getGlPosX()) > fabs(k) && ( getSitaZ() - getpre_SitaZ() ) > 0 ){	//&& ( getSitaZ() - pre.getSitaZ() )
		return true;
	}else{
		return false;
	}
}

// 遊脚支持脚切り替え判定ｙ
bool Robot_PID::changeFootY(Robot_PID pre){
	double k = 0.05 ;//0.05;

	//        重心位置Y　 > 条件値  &&          外に倒れている　　　 &&  重心が支持脚と逆に進んでいる
	if( fabs(getGlPosY()) > fabs(k) && (getSitaZ()-getpre_SitaZ()>0) && (getL_or_R()*getGlVelY()>0.0) ){	//&& (getSitaZ()-pre.getSitaZ()>0)
		return true;
	}else{
		return false;
	}
}




/*****************************************************************************************/
// 速度から着地目標を計算 
/*****************************************************************************************/
// 着地位置計算
double Robot_PID::refXCal(){
	// 着地目標ｘ位置の計算
	double refX=0;
	//if( getChuukuu() ){
		// 支持脚センサが４つ以上オンのある時
		//refX = 1*getGlPosX() + 0.2*getGlVelX() - 0.2*(refVel-getGlVelX());
		//refX = 1*getGlPosX() + 0.02*getGlVelX() - 0.05*(refVel-getGlVelX());
		refX = 1.0*getGlPosX() + 0.1*getGlVelX() - 0.1*(refVel-getGlVelX());
	//}else{
		// 支持脚センサが４つ以上オンでない時
		//refX = 1*getGlPosX() + 0.02*getGlVelX() - 0.05*(refVel-getGlVelX());
	//}
	// 上限リミッター
	if(refX > 30.0e-2)	{
		refX = 30.0e-2;
		//printf("refX=%lf,jogen\n",refX);
	}
	// 下限リミッター
	if(refX < - 20.0e-2)	{
		refX =  - 20.0e-2;
		//printf("refX=%lf,kagen\n",refX);
	}
	return refX;
}
double Robot_PID::refYCal(){
	// 着地目標ｙ位置の計算
	//double refY = 1*getGlPosY() + 0.026*getGlVelY() + 0.029*getL_or_R();
	double refY = 1*getGlPosY() + 0.026*getGlVelY() + 0.09*getL_or_R();
	// 着地目標ｙが中心より逆側になった場合０にする
	if(refY*getL_or_R() < 0.0){
		refY = 0.0;
	}
	return refY;
}


/*****************************************************************************************/
// 伸長力を計算
/*****************************************************************************************/
// 伸長力計算用メソッド
double get_gain_x_plus( double sita_x ){
	if (sita_x < 0) {
		//printf("ERROR!! sita_x < 0 \n");
		return 0.;
	}else if (sita_x < M_PI * (3./8.) ) {		
		return 1.;
	}else if ( sita_x < (M_PI / 2.) ){
		return 1. + (0. - 1.) / (M_PI / 2. - M_PI * (3./8.) ) * (sita_x - M_PI *(3. / 8.) );
	}else if ( sita_x < (M_PI) ){
		return 0.;
	}else{   
		//printf("ERROR!! sita_x < Pi \n");
		return 0.;
	}
}
double get_gain_x_minus(double sita_x){
	if( sita_x < 0 ){        
	   // printf("ERROR!! sita_x < 0\n");
		return 0.;
	}else if( sita_x < (M_PI / 2.) ){
		return 0.;
	}else if(  sita_x < ((5. / 8.) * M_PI) ) {
		return 0. + (1. - 0.) / ((5. / 8.) * M_PI - M_PI / 2.) * (sita_x - M_PI / 2.);
	}else if(  (sita_x < (M_PI)) ) {
		return 1.;
	}else {
	   // printf("ERROR!! sita_x > Pi\n");
		return 0.;
	}
}
double Robot_PID::shinchouryoku(Robot_PID pre){
	// 速度誤差の算出
	double vxErr = refVel - getGlVelX();
	double prevxErr = getGlVelX() -pre.getGlVelX();

	// 重力に対する力
	double f_Mg = getMass() * 9.8 * cos(getSitaZ());
	//printf("mass=%lf\n",getMass());
	//printf("f_Mg=%lf\n",f_Mg);

	// x角度分のプラス補正
	double f_x_plus  = get_gain_x_plus(getSitaX()) * ( 30.0*vxErr +5.0*(vxErr-prevxErr));
	if(f_x_plus <-10){ f_x_plus  = -10; }
	//printf("get_gain_x_plus(agent.getSitaX())=%lf\n",get_gain_x_plus(agent.getSitaX()));
	//printf("f_x_plus=%lf\n",f_x_plus);

	// x角度分のマイナス補正
	double f_x_minus = get_gain_x_minus(getSitaX()) * (-30.0*vxErr -5.0*(vxErr-prevxErr));
	if(f_x_minus<-10){ f_x_minus = -10; }
	//printf("f_x_minus=%lf\n",f_x_minus);

	// 高さ誤差からの補正
	double poszErr = refHight - getGlPosZ();
	//printf("getGlPosZ()=%lf\n",getGlPosZ());
	poszErrInteg = (fabs(poszErr)<0.05) ? (poszErrInteg + poszErr) : (poszErrInteg + sign(poszErr)*0.05) ;
	double f_z_pos = 8.0*poszErr - 20.0*getGlVelZ()+ 0.02*poszErrInteg;
	//printf("f_z_pos=%lf\n",f_z_pos);

	// 伸長力
	double f = ( 1.0*f_Mg + f_x_plus + f_x_minus + f_z_pos );
	if(f<0){ f=0.0; }
	//printf("f=%lf\n",f);

	return f;

}

/*****************************************************************************************/
// モータ・スライダに力を設定する
/*****************************************************************************************/
void Robot_PID::setMotor(int steps, double f){
	double f_Yuukyaku=0;
	double ref_Yuukyaku_length=0;
	/*
	double LL_l_3 = dJointGetSliderPosition(LegLJoint[3]);		// * 1E+2;
	double LL_l_3_vel = dJointGetSliderPositionRate(LegLJoint[3]);	// * 1E+2;
	double LR_l_3 = dJointGetSliderPosition(LegRJoint[3]);		// * 1E+2;
	double LR_l_3_vel = dJointGetSliderPositionRate(LegRJoint[3]);	// * 1E+2;
	*/

	if(steps < 50){
		
		// 初期姿勢を取らせる
		stat=4;
		init_refjoint();
		// 計算結果から力を設定する
		for(int i=0;i<6;i++){
			setHingeParams(LegLJoint[i], refLegLJointAngle[i]);
			setHingeParams(LegRJoint[i], refLegRJointAngle[i]);
		}
		/*
		for(int i=0;i<6;i++){
			// 初期姿勢のモーター目標角度を設定
			// 左脚
			setHingeParams(LegLJoint[i], 0.0);
			// 右脚
			setHingeParams(LegRJoint[i], 0.0);
		}
		*/
	
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

	}else{
		
		// 計算結果から力を設定する
		for(int i=0;i<6;i++){
			setHingeParams(LegLJoint[i], refLegLJointAngle[i]);
			setHingeParams(LegRJoint[i], refLegRJointAngle[i]);
		}

		// 基本形として支持脚の足首はフリー
		if(getChuukuu() == 0){
			if( getL_or_R() == -1 ){
				// 左脚フリー
				//printf("左脚フリー\n");
				dJointSetHingeParam(LegLJoint[4], dParamFMax, 0);
				dJointSetHingeParam(LegLJoint[5], dParamFMax, 0);
			}else{
				// 右足フリー
				//printf("右脚フリー\n");
				dJointSetHingeParam(LegRJoint[4], dParamFMax, 0);
				dJointSetHingeParam(LegRJoint[5], dParamFMax, 0);
			}
		}
/*
		// 遊脚の長さ目標を決定する
		if( (stat == 2) || (stat == 3) ){
			// 2:右脚着地動作中or3:右脚半着地中
			ref_Yuukyaku_length =  40E-2; //30E-2; //50E-2;

		}else if( (stat == 7) || (stat == 8) ){
			// 7:左脚着地動作中or8:左脚半着地中
			ref_Yuukyaku_length =  40E-2; //30E-2; //50E-2;

		}else if( (stat == 5)  || (stat == 10) ){
			// 5:左脚持ち上げ中or10:右脚持ち上げ中
			ref_Yuukyaku_length = -20e-2;

		}else if( (stat == 4)  || (stat == 9) ){
			// 4,9:両足支持中
			ref_Yuukyaku_length = 15e-2;

		}else if( (stat == 1)  || (stat == 6) ){
			// 1:左脚支持中or6:右脚支持中
			ref_Yuukyaku_length = -5e-2;//-5e-2; // -10e-2; //-30E-2;

		}else{
			// あり得ない数字なのでエラー表示
			printf("Motor_set.h  ERROR(stat=%d is an impossible number.) ",stat);
			getchar();
			exit(0);
		}
*/
		
		// 計算結果をスライダー目標長さに設定 -------------------------------------------------
		/*
		if(getL_or_R() == -1){
			// 左脚
			dJointAddSliderForce(LegLJoint[3], f);			// 支持脚は伸長力で求めたfを使う
			// 右脚
			f_Yuukyaku = getSliderForce(ref_Yuukyaku_length,LR_l_3,LR_l_3_vel);
			dJointAddSliderForce(LegRJoint[3], f_Yuukyaku);		// 遊脚は上で決めた結果を使う

		}else{
			// 左脚
			f_Yuukyaku = getSliderForce(ref_Yuukyaku_length,LL_l_3,LL_l_3_vel);
			dJointAddSliderForce(LegLJoint[3], f_Yuukyaku);		// 遊脚は上で決めた結果を使う
			// 右脚
			dJointAddSliderForce(LegRJoint[3], f);			// 支持脚は伸長力で求めたfを使う

		}
		*/
	}
}

void Robot_PID::Ashisaki(){
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
