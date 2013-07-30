/*****************************************************************************************/
//                                     ロボットクラス	
/*****************************************************************************************/
// ■2013/6/4 pos=dBodyGetPosition(dbodyID);をgetPartsPosX(dBodyID);で統一
// ■描画関係の関数内以外はこちらで統一するようにすること
//
/*****************************************************************************************/

class Robot{
private:
	//---------------------------------------------------------------------------------//
	// ロボットのプライベートメンバ変数の宣言
	//---------------------------------------------------------------------------------//
	dVector3 absGl;	// ロボットの重心位置（絶対座標系での重心位置）
	dVector3 GlPos;	// ロボットの重心位置（支持脚接地位置が原点）
	dVector3 GlVel;	// ロボットの重心速度
	dVector3 GlAcc;	// ロボットの重心加速度
	dReal mass;		// ロボットの全質量（重心計算時に取得）
	int L_or_R;		// ロボットの支持脚（左:-1, 右:1）
public:
	int stat;
	int hosuu;

	//着地目標位置手動操作用
	double x;
	void setxmi(){ x = x - 0.01; }
	void setxpu(){ x = x + 0.01; }

	dVector3 refPos;
	dVector3 Sita;
	dVector3 pre_Sita;

	dReal refLegLJointAngle[6];
	dReal refLegRJointAngle[6];
	dReal buf_LLr[6];
	dReal buf_LRr[6];

	dVector3 workDistance;
	float ashiSaki[4];

	Robot(){
		init();		///変数初期化
	}

	void init()
	{
		stat = 0;
		hosuu = 0;
		x=0.0;
		for(int i=0;i<3;i++){
			//refPos[i]=0.0;
			//Sita[i]=0.0;
			//pre_Sita[i]=0.0;
			//ashiSaki[i]=0.0;
		}
		for(int i=0;i<4;i++) workDistance[i]=0.0;
		
		for(int i=0;i<6;i++){
			buf_LLr[i]=0.0;
			buf_LRr[i]=0.0;
		}
		init_Gl();
		init_refjoint();
	}
	//ジョイント角度の初期設定
	void init_refjoint(){

		refLegLJointAngle[0] =  deg2rad(0);
		refLegLJointAngle[1] =  deg2rad(0);
		refLegLJointAngle[2] =  deg2rad(-30);
		refLegLJointAngle[3] =  deg2rad(60);
		refLegLJointAngle[4] =  deg2rad(-30);
		refLegLJointAngle[5] =  deg2rad(0);

		refLegRJointAngle[0] =  deg2rad(0);
		refLegRJointAngle[1] =  deg2rad(0);
		refLegRJointAngle[2] =  deg2rad(-30);
		refLegRJointAngle[3] =  deg2rad(60);
		refLegRJointAngle[4] =  deg2rad(-30);
		refLegRJointAngle[5] =  deg2rad(0);
	}
	//重心関係変数宣言
	dReal Gl[4];
	float zero_vec[4];

	const dReal *w_R_12;
	dReal w_R_44[4][4];
	dReal inverse_w_R_44[4][4];

	float work_distance[4];
	float work_chaku_moku[4];
	float work_a_distance[4];

	//重心関係変数初期化
	void init_Gl(){
		for(int i=0;i<4;i++){
			Gl[i] = 0.0;
			zero_vec[i] = 0.0;
			work_distance[i] = 0.0;
			work_chaku_moku[i] = 0.0;
			work_a_distance[i] = 0.0;
		}
		for (int i=0;i<4;i++){
			for (int j=0;j<4;j++){
				w_R_44[i][j] = 0.0;
				inverse_w_R_44[i][j] = 0.0;
			}
		}
	}

	//---------------------------------------------------------------------------------//
	// ロボットのパブリックメンバ変数の宣言
	//---------------------------------------------------------------------------------//
	MyBoxLink Body[2];		// 本体（腰・胴）
	MyBoxLink LegL[8];		// 左脚（腰、腱、膝、足）
	MyBoxLink LegR[8];		// 右脚（腰、腱、膝、足）
	MyBoxLink SensorL[6];		// 左足裏センサ（左前、右前、左中、右中、左後、右後）
	MyBoxLink SensorR[6];		// 右足裏センサ（右前、左前、右中、左中、右後、左後）
	MyBoxLink SpringL[6];		// 左足裏センサのバネ部分
	MyBoxLink SpringR[6];		// 右足裏センサのバネ部分

	dJointID BodyJoint[1];		// Body[0]とBody[1]のジョイント
	dJointID LegLJoint[8];		// 左脚のジョイント(3番Slider、6・7番はFixed)
	dJointID LegRJoint[8];		// 右脚のジョイント(3番Slider、6・7番はFixed)
	dJointID joint;				// 空中用
	dJointID SenLJoint[6];		// 左足裏センサジョイント
	dJointID SenRJoint[6];		// 右足裏センサジョイント
	dJointID SrgLJoint[6];		// 左足裏センサバネジョイント
	dJointID SrgRJoint[6];		// 右足裏センサバネジョイント


	//---------------------------------------------------------------------------------//
	// ゲッター・セッター
	//---------------------------------------------------------------------------------//
	// 支持脚のセッターとゲッター（左:-1, 右:1）
	void setL_or_R(int l_or_r){ L_or_R = l_or_r; }
	int getL_or_R(){ return L_or_R; }

	//sitaのセッター
	void setSitaX(dReal sitaX){ Sita[0] = sitaX; }
	void setSitaY(dReal sitaY){ Sita[1] = sitaY; }
	void setSitaZ(dReal sitaZ){ Sita[2] = sitaZ; }

	/*
	支持脚を原点とした重心のゲッター
	calcGlPosでGlPosとmassを計算
	calcGlVelでGlVelを計算
	calcGlAccでGlAccを計算
	*/
	double getMass(){ return mass;}	// ロボットの全質量のゲッター
	double getGlPosX(){ return GlPos[0]; }	// 重心位置ｘのゲッター
	double getGlPosY(){ return GlPos[1]; }	// 重心位置ｙのゲッター	
	double getGlPosZ(){ return GlPos[2]; }	// 重心位置ｚのゲッター
	double getSitaX(){ return Sita[0]; }	//sitaの値変えたいとき用
	double getSitaY(){ return Sita[1]; }
	double getSitaZ(){ return Sita[2]; }
	double getpre_SitaX(){ return pre_Sita[0]; }	//sita前回の値保存用
	double getpre_SitaY(){ return pre_Sita[1]; }
	double getpre_SitaZ(){ return pre_Sita[2]; }
	double getGlVelX(){ return GlVel[0]; }	// 重心速度ｘのゲッター
	double getGlVelY(){ return GlVel[1]; }	// 重心速度ｙのゲッター	
	double getGlVelZ(){ return GlVel[2]; }	// 重心速度ｚのゲッター
	double getGlAccX(){ return GlAcc[0]; }	// 重心加速度ｘのゲッター
	double getGlAccY(){ return GlAcc[1]; }	// 重心加速度ｙのゲッター	
	double getGlAccZ(){ return GlAcc[2]; }	// 重心加速度ｚのゲッター

	bool getChuukuu(){
		if(getL_or_R() == -1){
			if( SensorLCheck() ==4 ){
				// 左脚支持で左脚センサがすべてオン -> 中空フラグオン
				return false;//true;	//0がon
			}else{
				return true;//false;	//1がoff
			}
		}else{
			if( SensorRCheck() ==4 ){
				// 右脚支持で右脚センサがすべてオン -> 中空フラグオン
				return false;//true;	
			}else{
				return true;//false;
			}
		}
		printf("error:getChuukuu\n");
		getchar();
		exit(0);
	}

	
	// ロボットの生成メソッド
	void createRobot(dReal x0,dReal y0, dReal z0);

	// ロボットの描画メソッド
	void drawRefPos();
	void drawRobot();
	void drawPendulum();

	// ロボットの破壊メソッド
	void destroyRobot();


	// sita計算
	void calcSita(){
		Sita[0] = atan2(sqrt(pow(GlPos[1],2) + pow(GlPos[2],2)),GlPos[0]);
		Sita[1] = atan2(sqrt(pow(GlPos[2],2) + pow(GlPos[0],2)),GlPos[1]);
		Sita[2] = atan2(sqrt(pow(GlPos[0],2) + pow(GlPos[1],2)),GlPos[2]);
	}

	//---------------------------------------------------------------------------------//
	// 重心位置・速度・加速度の計算メソッド
	//---------------------------------------------------------------------------------//
	// 絶対座標系の重心位置の設定
	void setAbsGl(dReal glx, dReal gly, dReal glz){
		absGl[0] = glx;
		absGl[1] = gly;
		absGl[2] = glz;
	}
	// パーツを追加して重心計算（absGlGetで利用）
	void calcAbsGlAddParts(dBodyID body, const dReal addMass, const dReal sumMass){
		absGl[0] = absGl[0]*(sumMass/(addMass+sumMass)) + getPartsPosX(body)*(addMass/(addMass+sumMass));
		absGl[1] = absGl[1]*(sumMass/(addMass+sumMass)) + getPartsPosY(body)*(addMass/(addMass+sumMass));
		absGl[2] = absGl[2]*(sumMass/(addMass+sumMass)) + getPartsPosZ(body)*(addMass/(addMass+sumMass));
	}

	// 絶対座標系重心計算（ODEのAPI利用、絶対座標系）
	void calcAbsGl(){
		//const dReal *pos;		// パーツ位置情報一時保管用
		dReal sumMass=0;		// 足し込んだパーツの質量合計

		// 胴体の座標設定（一つ目なので、ロボット重心＝パーツの重心）
		//pos = dBodyGetPosition(Body[0].body);			// パーツ位置を取得
		setAbsGl(getPartsPosX(Body[0].body),getPartsPosY(Body[0].body),getPartsPosZ(Body[0].body));				// 重心位置を保存
		sumMass = Body[0].mass;						// パーツ質量を設定
		
		// 胴体の重心足し込み（二つ目以降は現在の重心の足し込む形で再計算していく）
		//pos = dBodyGetPosition(Body[1].body);			// パーツ位置を取得
		calcAbsGlAddParts(Body[1].body, Body[1].mass, sumMass);	// パーツを追加して重心計算
		sumMass += Body[1].mass;					// パーツ質量を足し込む

		// 脚のパーツ重心足し込み
		for(int i=0;i<8;i++){
			// 左脚座標足し込み
			//pos = dBodyGetPosition(LegL[i].body);		// パーツ位置を取得
			calcAbsGlAddParts(LegL[i].body, LegL[i].mass, sumMass);	// パーツを追加して重心計算
			sumMass += LegL[i].mass;				// パーツ質量を足し込む
			// 右脚座標足し込み
			//pos = dBodyGetPosition(LegR[i].body);		// パーツ位置を取得
			calcAbsGlAddParts(LegR[i].body, LegR[i].mass, sumMass);	// パーツを追加して重心計算
			sumMass += LegR[i].mass;				// パーツ質量を足し込む
			
		}

		// ロボットの全質量
		mass = sumMass;
	}

	// 相対座標系重心計算（ODEのAPI利用、支持脚の足裏接地位置を原点とした場合）
	void calcGlPos(){
		calcAbsGl();			// 絶対座標系での重心位置の取得
		//const dReal *pos;		// パーツ位置情報一時保管用
		if(L_or_R == -1){
			//pos = dBodyGetPosition(LegL[7].body);	// 原点座標位置の取得
			GlPos[0] = absGl[0] - getPartsPosX(LegL[7].body);
			GlPos[1] = absGl[1] - getPartsPosY(LegL[7].body);
			GlPos[2] = absGl[2] - getPartsPosZ(LegL[7].body);
		}else{
			//pos = dBodyGetPosition(LegR[7].body);	// 原点座標位置の取得
			//getPartsPos(LegR[7].body, pos);
			GlPos[0] = absGl[0] - getPartsPosX(LegR[7].body);
			GlPos[1] = absGl[1] - getPartsPosY(LegR[7].body);
			GlPos[2] = absGl[2] - getPartsPosZ(LegR[7].body);
		}


	}

	// 重心速度の計算
	void calcGlVel(Robot pre0, double steptime){
		GlVel[0] = (absGl[0]-pre0.absGl[0])/steptime;
		GlVel[1] = (absGl[1]-pre0.absGl[1])/steptime;
		GlVel[2] = (absGl[2]-pre0.absGl[2])/steptime;

		/*
		GlVel[0] = (getGlPosX()-pre0.getGlPosX())/steptime;
		GlVel[1] = (getGlPosY()-pre0.getGlPosY())/steptime;
		GlVel[2] = (getGlPosZ()-pre0.getGlPosZ())/steptime;
		*/
	}

	// 重心加速度の計算
	void calcGlAcc(Robot pre0, double steptime){
		GlAcc[0] = (getGlVelX()-pre0.getGlVelX())/steptime;
		GlAcc[1] = (getGlVelY()-pre0.getGlVelY())/steptime;
		GlAcc[2] = (getGlVelZ()-pre0.getGlVelZ())/steptime;
	}


	//---------------------------------------------------------------------------------//
	// センサ状態取得
	//---------------------------------------------------------------------------------//
	// 左足センサがオンしている個数の取得
	int SensorLCheck(){
		int onNumSensor=0;
		/*
		for(int i=0;i<6;i++){
			if(dJointGetSliderPosition(SrgLJoint[i]) < -0.1E-2){			
				onNumSensor++;	
			}
		}
		*/
		//const dReal *pos;
		for(int i=0;i<4;i++){
			//pos = dBodyGetPosition(SensorL[i].body);
			//printf("sensorL=%lf\n",pos[2]);
			if(getPartsPosZ(SensorL[i].body) < 0.02){			//0.02
				onNumSensor++;	
			}
		}
		return onNumSensor;	
	}
	// 右足センサがオンしている個数の取得
	int SensorRCheck(){
		int onNumSensor=0;
		/*
		for(int i=0;i<6;i++){
			if(dJointGetSliderPosition(SrgLJoint[i]) < -0.1E-2){			
				onNumSensor++;	
			}
		}
		*/
		//const dReal *pos;
		for(int i=0;i<4;i++){
			//pos = dBodyGetPosition(SensorR[i].body);
			//printf("sensorR=%lf\n",pos[2]);
			if(getPartsPosZ(SensorR[i].body) < 0.02){			
				onNumSensor++;	
			}
		}
		return onNumSensor;	
	}

	//---------------------------------------------------------------------------------//
	// 脚関節角度計算
	//---------------------------------------------------------------------------------//
	// 足ジョイントの目標角度・目標長さ
	void initRefJointAngle(){
		for(int i=0;i<6;i++){
			refLegLJointAngle[i] = 0.0;
			refLegRJointAngle[i] = 0.0;
		}
	}

	// 遊脚脚付け根が原点とした現在遊脚足先位置の取得	（現在は遊脚のjoint[0]の座標を原点としている。）
	void getSwingLegStr2EndPos(dReal r[4], const dReal ref[4]){
		//const dReal *pos;	// パーツ位置取得用
		dVector3 pos;
		if(getL_or_R()==-1){
			//pos = dBodyGetPosition(LegR[1].body);
			dJointGetHingeAnchor(LegRJoint[0],pos);
		}else{
			//pos = dBodyGetPosition(LegL[1].body);
			dJointGetHingeAnchor(LegLJoint[0],pos);
		}
		//for( int i=0;i<3;i++) printf("%lf\n",pos[i]);
		vec_minus(r, ref, pos);

	}

	// 遊脚付け根(LegX[0])の姿勢の取得
	void getSwingLegSrtRel(dReal r[][4]){
		const dReal *rel;
		if(getL_or_R()==-1){
			rel = dBodyGetRotation(LegR[1].body);		//ここLegR[0]→LegR[1]に変えた。正しいかはわからない 
		}else{
			rel = dBodyGetRotation(LegL[1].body);
		}

		for(int i=0;i<4;i++){
			for(int j=0;j<4;j++){
				r[i][j] = rel[i*4+j];
			}
		}
	}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
	//↓追加関数
	
	// 支持脚付け根(LegX[1])の姿勢の取得
	void getsupportLegSrtRel(dReal r[][4]){
		const dReal *rel;
		if(getL_or_R()==-1){
			rel = dBodyGetRotation(LegL[0].body);
		}else{
			rel = dBodyGetRotation(LegR[0].body);
		}

		for(int i=0;i<4;i++){
			for(int j=0;j<4;j++){
				r[i][j] = rel[i*4+j];
			}
		}
	}

	void getsupportLegStr2EndPos(dReal r[4]){
		dVector3 pos;
		dVector3 pos1;
		if(getL_or_R()==-1){
			dJointGetHingeAnchor(LegLJoint[0],pos);
			dJointGetHingeAnchor(LegLJoint[5],pos1);
		}else{
			dJointGetHingeAnchor(LegRJoint[0],pos);
			dJointGetHingeAnchor(LegRJoint[5],pos1);
		}

		//for( int i=0;i<3;i++) printf("%lf\n",pos[i]);
		vec_minus(r, pos, pos1);
		r[2] += 8.0E-2;	//足裏までの長さ
	}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


	////////////////////////////////////////////////////////////////////////////////////////////
	// ジョイントの目標角度の計算
	void calcRefJoint(Robot pre, dVector3 refSwingLegPos){

		// 遊脚目標足先(LegX[7])位置の計算(遊脚付け根(LegX[0])が原点)
		dReal swingLegSrt2RefPos[4];
		getSwingLegStr2EndPos(swingLegSrt2RefPos, refSwingLegPos);
		
		// 遊脚付け根(LegX[0])の姿勢の取得
		dReal swingLegSrtRel[4][4];
		getSwingLegSrtRel(swingLegSrtRel);

		// 遊脚付け根(LegX[0])の姿勢の逆行列の取得
		dReal inv_swingLegSrtRel[4][4];
		inverse(inv_swingLegSrtRel, swingLegSrtRel);

		// 上で計算した行列とベクトルを掛け合わせる
		dReal sigma[4];
		ftrvadd( sigma, swingLegSrt2RefPos, inv_swingLegSrtRel);

		//for(int i=0;i<3;i++) printf("sigma[%d]=%lf\n",i,sigma[i]);

	//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
	//以下相馬先生のコピペとりあえずSHR版

		int flag;
		//脚の長さ(joint間の長さ)	//左右同じなので左足のみ宣言
		float Lxl_2 = 0.0;
		float Lxl_3 = 25.0E-2;
		float Lxl_4 = 25.0E-2;
		float Lxl_5 = 0.0;
		float Lxl_6 = (7.0 + 1.0) * 0.01;

		//角度計算に使う↓
		double l_sha = sqrtf( (fabs(sigma[2]) - Lxl_6 ) * (fabs(sigma[2]) - Lxl_6 ) + sigma[1] * sigma[1] );

		double l_2to4 = sqrtf( sigma[0] * sigma[0] + ( l_sha - Lxl_2 - Lxl_5 ) * ( l_sha - Lxl_2 - Lxl_5 ));
		
		double l_t1 = ( Lxl_3 * Lxl_3 -  Lxl_4 * Lxl_4 + l_2to4 * l_2to4  ) / (2 * l_2to4 );

		double alpha_0 = atan2f( sigma[0]  ,  ( l_sha - Lxl_2 - Lxl_5 ));

		double alpha_1 = acosf(( Lxl_3*Lxl_3 + l_2to4*l_2to4 - Lxl_4*Lxl_4 )
								/  ( 2 * Lxl_3 * l_2to4 ) );
		double alpha_2 = acosf( ( (Lxl_3)*(Lxl_3) + (Lxl_4)*(Lxl_4) - (l_2to4)*(l_2to4) )
								/  ( 2 * (Lxl_3) * (Lxl_4) ));

		//printf("l_sha=%lf\n",l_sha);
		//printf("l_2to4=%lf\n",l_2to4);
		//printf("alpha_0=%lf\n",alpha_0 * 180.0 / M_PI);
		//printf("alpha_1=%lf\n",alpha_1 * 180.0 / M_PI);
		//printf("alpha_2=%lf\n",alpha_2 * 180.0 / M_PI);


		// =========================================================================
		//  (step8) 遊脚の間接角度を求める．
		// =========================================================================

		// 左足支持の場合
		if( getL_or_R() == -1){
			
			// 右脚（遊脚）の計算
			refLegRJointAngle[1] = atan2f(sigma[1], -sigma[2]);
			
			flag = 1;
			if (refLegRJointAngle[1]  >  M_PI ) {
				refLegRJointAngle[1] = refLegRJointAngle[1] - M_PI;
			}else if(refLegRJointAngle[1] < ( -5 * M_PI)){
				refLegRJointAngle[1] = refLegRJointAngle[1] + M_PI;
			}else{
				flag = 0;
			}

			if (flag == 0){

				// 膝が伸びきるかチェック（目標位置に脚が伸ばせるかどうか？）
				if( l_2to4 < (Lxl_3 + Lxl_4) ){

					if( ! ( (l_2to4 + Lxl_3) <= Lxl_4 ) ){
						refLegRJointAngle[2] = - ( alpha_0 + alpha_1 ); // 前の負符号はモーターの回転方向からくる．
						refLegRJointAngle[3] = M_PI - alpha_2;
					}else{
						refLegRJointAngle[2] = getHingeAngle(LegRJoint[2]);
						refLegRJointAngle[3] = getHingeAngle(LegRJoint[3]);
					}
				}else{	//伸びきらない場合
					refLegRJointAngle[2] = - alpha_0;
					refLegRJointAngle[3] = 0;
				}
			}else{
				printf("stop -- Yuukyaku_shijikyaku.h");
				getchar();
			}

			if( (stat == 9) ){	//両足支持になったときの処理
				if( (stat == 9) && (pre.stat == 8) ){
					buf_LRr[1] = getHingeAngle(LegRJoint[1]);
					buf_LRr[2] = getHingeAngle(LegRJoint[2]);
					buf_LRr[3] = getHingeAngle(LegRJoint[3]);
				}
				refLegRJointAngle[1] = buf_LRr[1];
				refLegRJointAngle[2] = buf_LRr[2];
				refLegRJointAngle[3] = buf_LRr[3];
			}

			//	遊脚の脚上げ処理がダブっているので，こちらをコメント化する．←よくわからん一応コメントアウトしないでおこう
			//ここコメントアウトすると歩かなくなる。不安定
			
			if( (stat == 10) ){	//遊脚持ち上げ中
				refLegRJointAngle[1] = buf_LRr[1];
				refLegRJointAngle[2] = buf_LRr[2] - deg2rad(40);
				refLegRJointAngle[3] = buf_LRr[3] + deg2rad(60);
			}
			

			refLegRJointAngle[0] = 0.0;	//方向転換用ジョイント　現在不使用

			buf_LLr[1] = getHingeAngle(LegLJoint[1]);
			buf_LLr[5] = getHingeAngle(LegLJoint[5]);
			buf_LRr[1] = getHingeAngle(LegRJoint[1]);

			refLegRJointAngle[4] = - (refLegRJointAngle[2] + refLegRJointAngle[3]);	//地面に対して垂直になるようにするためかな？
			//refLegRJointAngle[5] =  (buf_LLr[5] + buf_LLr[1]) - refLegRJointAngle[1] ;
			//refLegRJointAngle[5] =  (buf_LLr[5] + buf_LLr[1]) - buf_LRr[1] ;	//どっちが正しいのか？
			refLegRJointAngle[5] = - refLegRJointAngle[1];	//←が正しいような気がするが・・・


		// 右足支持の場合の左足の角度計算
		}else{
			// 左脚（遊脚）の計算
			refLegLJointAngle[1] = atan2f(sigma[1], -sigma[2]);

			flag = 1;
			if (refLegLJointAngle[1]  >  M_PI ) {
				refLegLJointAngle[1] = refLegLJointAngle[1] - M_PI;
			}else if(refLegLJointAngle[1] < ( -5 * M_PI)){
				refLegLJointAngle[1] = refLegLJointAngle[1] + M_PI;
			}else{
				flag = 0;
			}

			if (flag == 0){

				// 膝が伸びきるかチェック（目標位置に脚が伸ばせるかどうか？）
				if( l_2to4 < (Lxl_3 + Lxl_4) ){

					if( ! ( (l_2to4 + Lxl_3) <= Lxl_4 ) ){
						refLegLJointAngle[2] = - ( alpha_0 + alpha_1 ); // 前の負符号はモーターの回転方向からくる．
						refLegLJointAngle[3] = M_PI - alpha_2;
					}else{
						refLegLJointAngle[2] = getHingeAngle(LegLJoint[2]);
						refLegLJointAngle[3] = getHingeAngle(LegLJoint[3]);
					}
				}else{
					refLegLJointAngle[2] = - alpha_0;
					refLegLJointAngle[3] = 0;
				}
			}else{
				printf("stop -- Yuukyaku_shijikyaku.h");
				getchar();
			}

			if( (stat == 4) ){	//両足支持になったときの処理
				if( (stat == 4) && (pre.stat == 3) ){
					buf_LLr[1] = getHingeAngle(LegLJoint[1]);
					buf_LLr[2] = getHingeAngle(LegLJoint[2]);
					buf_LLr[3] = getHingeAngle(LegLJoint[3]);
				}
				refLegLJointAngle[1] = buf_LLr[1];
				refLegLJointAngle[2] = buf_LLr[2];
				refLegLJointAngle[3] = buf_LLr[3];
			}
			//	遊脚の脚上げ処理がダブっているので，こちらをコメント化する．
			
			if( (stat == 5) ){	//遊脚持ち上げ中
				refLegLJointAngle[1] = buf_LLr[1];
				refLegLJointAngle[2] = buf_LLr[2] - deg2rad(40);
				refLegLJointAngle[3] = buf_LLr[3] + deg2rad(60);
			}
			

			refLegLJointAngle[0] = 0.0;

			buf_LRr[1] = getHingeAngle(LegRJoint[1]);
			buf_LRr[5] = getHingeAngle(LegRJoint[5]);

			refLegLJointAngle[4] = - (refLegLJointAngle[2] + refLegLJointAngle[3]);
			//refLegLJointAngle[5] =  (buf_LRr[5] + buf_LRr[1]) - refLegLJointAngle[1] ;	//こっちが正しいかも
			refLegLJointAngle[5] = - refLegLJointAngle[1];

		} //右足支持の場合のelse文の閉じ括弧


		// =========================================================================
		//  (step9) 支持脚の関節角度を求める．
		// ========================================================================
		
		//以下の支持脚角度計算は相馬先生のコピペSHR版

		double ref_z = 55.0E-2;//35.0E-2;		//目標高さ
		double ref_Lx_1[3];
		double Lx_1[3];		
		double ref_Rx_1[3];
		double Rx_1[3];

		// 支持脚目標高さ(LegX[0])位置の計算(足裏が原点)
		dReal supportLegSrt2RefPos[4];
		getsupportLegStr2EndPos(supportLegSrt2RefPos);

		//for(int i=0;i<3;i++) printf("supportLegSrt2RefPos[%d]=%lf\n",i,supportLegSrt2RefPos[i]);
		
		// 支持脚付け根(LegX[0])の姿勢の取得
		dReal supportLegSrtRel[4][4];
		getsupportLegSrtRel(supportLegSrtRel);

		// 支持脚付け根(LegX[0])の姿勢の逆行列の取得
		dReal inv_supportLegSrtRel[4][4];
		inverse(inv_supportLegSrtRel, supportLegSrtRel);

		// 上で計算した行列とベクトルを掛け合わせる
		dReal sigma1[4];
		ftrvadd( sigma1, supportLegSrt2RefPos, inv_supportLegSrtRel);

		//for(int i=0;i<3;i++) printf("sigma1[%d]=%lf\n",i,sigma1[i]);

		//for(int i=0;i<3;i++) Lx_1[i] = sigma1[i];
		//for(int i=0;i<3;i++) Rx_1[i] = sigma1[i];

		for(int i=0;i<3;i++) Lx_1[i] = supportLegSrt2RefPos[i];
		for(int i=0;i<3;i++) Rx_1[i] = supportLegSrt2RefPos[i];


		float Rxl_2 = 0.0;
		float Rxl_3 = 25.0E-2;
		float Rxl_4 = 25.0E-2;
		float Rxl_5 = 0.0;
		float Rxl_6 = (7.0 + 1.0) * 0.01;

		// 左足支持の場合
		if( getL_or_R() ==-1 ){ 

			// Glの高さで膝の折角を計算しているが、腰の高さが一定となるように修正の要があるかもしれない．
			//ref_Lx_1[0] =fabs(Lx_1[0]) * (ref_z - Lxl_6) / ( GlPos[2] - Lxl_6 );
			//ref_Lx_1[1] =fabs(Lx_1[1]) * (ref_z - Lxl_6) / ( GlPos[2] - Lxl_6 );
			
			//20130118 上のコメントに従って修正．（原因は中腰後のスタート時にいきなり膝を伸ばそうとする現象が出たため）
			ref_Lx_1[0] =fabs(Lx_1[0]) * (ref_z - Lxl_6) / ( Lx_1[2] - Lxl_6 );
			ref_Lx_1[1] =fabs(Lx_1[1]) * (ref_z - Lxl_6) / ( Lx_1[2] - Lxl_6 );
			//printf("ref_Lx_1[0]=%lf\n",ref_Lx_1[0]);
			//printf("ref_Lx_1[1]=%lf\n",ref_Lx_1[1]);

			l_t1 = sqrtf( (ref_z - Lxl_6)*(ref_z - Lxl_6) + (ref_Lx_1[1]) * (ref_Lx_1[1]) );

			l_sha = sqrt( (ref_Lx_1[0])*(ref_Lx_1[0]) + (l_t1 - Lxl_2 - Lxl_5) * (l_t1 - Lxl_2 - Lxl_5));

			alpha_0 = - sign( Lx_1[0] ) * atanf( (ref_Lx_1[0]) / (l_t1 - Lxl_2 - Lxl_5) );
			//printf("alpha_0=%lf\n",alpha_0);

			// 膝が伸びきるかチェック
			if( l_sha <= (Lxl_3 + Lxl_4) ){

				alpha_1 = acosf(
						( (Lxl_3)*(Lxl_3) + (l_sha)*(l_sha) - (Lxl_4)*(Lxl_4) )
						/
						( 2 * (Lxl_3) * (l_sha) )
					);

				refLegLJointAngle[2] = - ( alpha_0 + alpha_1 );

				alpha_2 = acosf( 
						( (Lxl_3)*(Lxl_3) + (Lxl_4)*(Lxl_4) - (l_sha)*(l_sha) )
						/
						( 2 * (Lxl_3) * (Lxl_4) )
					);

				refLegLJointAngle[3] = M_PI - alpha_2;

			}else{
				refLegLJointAngle[2] = - alpha_0;
				refLegLJointAngle[3] = 0;
			}

			refLegLJointAngle[1] = - getHingeAngle(LegLJoint[5]);

			//  中空時は，そのままの値を指定する．
			if( getChuukuu() == 1 ){
				printf("支持脚そのままの値保持\n");
				for(int i=0;i<6;i++){
					refLegLJointAngle[i] = getHingeAngle(LegLJoint[i]);
				}
			}


		}else{   // 右足支持の場合の右足の角度計算

			// Glの高さで膝の折角を計算しているが、腰の高さが一定となるように修正の要があるかもしれない．

			//ref_Rx_1[0] =fabs(Rx_1[0]) * (ref_z - Rxl_6) / ( Gl[2] - Rxl_6 );
			//ref_Rx_1[1] =fabs(Rx_1[1]) * (ref_z - Rxl_6) / ( Gl[2] - Rxl_6 );

			//20130118 上のコメントに従って修正．（原因は中腰後のスタート時にいきなり膝を伸ばそうとする現象が出たため）
			ref_Rx_1[0] =fabs(Rx_1[0]) * (ref_z - Rxl_6) / ( Rx_1[2] - Rxl_6 );
			ref_Rx_1[1] =fabs(Rx_1[1]) * (ref_z - Rxl_6) / ( Rx_1[2] - Rxl_6 );

			l_t1 = sqrtf( (ref_z - Rxl_6)*(ref_z - Rxl_6) + (ref_Rx_1[1]) * (ref_Rx_1[1]));

			l_sha = sqrt( (ref_Rx_1[0])*(ref_Rx_1[0]) + (l_t1 - Rxl_2 - Rxl_5) * (l_t1 - Rxl_2 - Rxl_5));

			alpha_0 = - sign( Rx_1[0] ) * atanf( (ref_Rx_1[0]) / (l_t1 - Rxl_2 - Rxl_5) );

			// 膝が伸びきるかチェック
			if( l_sha <= (Rxl_3 + Rxl_4) ){

				alpha_1 = acosf(
						( (Rxl_3)*(Rxl_3) + (l_sha)*(l_sha) - (Rxl_4)*(Rxl_4) )
						/
						( 2 * (Rxl_3) * (l_sha) )
					);

				refLegRJointAngle[2] = - ( alpha_0 + alpha_1 );

				alpha_2 = acosf( 
						( (Rxl_3)*(Rxl_3) + (Rxl_4)*(Rxl_4) - (l_sha)*(l_sha) )
						/
						( 2 * (Rxl_3) * (Rxl_4) )
					);

				refLegRJointAngle[3] = M_PI - alpha_2;

			}else{
				refLegRJointAngle[2] = - alpha_0;
				refLegRJointAngle[3] = 0;
			}


			refLegRJointAngle[1] = - getHingeAngle(LegRJoint[5]);

			//  中空時は，そのままの値を指定する．
			if( getChuukuu() == 1 ){
				printf("支持脚そのままの値保持\n");
				for(int i=0;i<6;i++){
					refLegRJointAngle[i] = getHingeAngle(LegRJoint[i]);
				}
			}
		}

	}



};
