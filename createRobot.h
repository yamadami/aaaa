/*****************************************************************************************/
// ロボットの生成メソッド
/*****************************************************************************************/
void Robot::createRobot(dReal x0,dReal y0, dReal z0)
{
	//dReal x0=0.0, y0=0.0, z0=80.0E-2;	// ロボットの初期位置中心点

	// 胴体の生成 -----------------------------------------------------------------//
	// 胴体０（一番上の部分）
	Body[0].setMass(15.0E-3);						// 質量の設定
	Body[0].setPos(-2.0E-2, 0.0E-2, 11.0E-2);			// 位置の設定
	Body[0].setSize(16.0E-2, 25.6E-2, 10.0E-2);		// サイズの設定
	Body[0].setAngle(1, 0, 0, 0.0*M_PI/180.0);		// 傾きの設定
	createBox(Body[0], x0, y0, z0);				// パーツ生成

	// 胴体１
	Body[1].setMass(100.0E-2);						// 質量の設定
	Body[1].setPos(-2.0E-2, 0.0E-2, 1.0E-2);			// 位置の設定
	Body[1].setSize(13.0E-2, 25.6E-2, 10.0E-2);			// サイズの設定
	Body[1].setAngle(1, 0, 0, 0.0*M_PI/180.0);		// 傾きの設定
	createBox(Body[1], x0, y0, z0);				// パーツ生成

	// 胴体１を胴体０にジョイント
	BodyJoint[0] = dJointCreateHinge(world,0);				// ジョイント生成
	dJointAttach(BodyJoint[0], Body[1].body, Body[0].body);		// ジョイントするパーツ
	dJointSetHingeAnchor(BodyJoint[0], x0, y0, z0+2.5E-2 );		// アンカーの位置
	dJointSetHingeAxis(BodyJoint[0], 0, 1, 0);				// 軸向き+x
	dJointSetHingeParam(BodyJoint[0], dParamFMax, 700);			// トルク上限
	dJointSetHingeParam(BodyJoint[0], dParamLoStop, -90.0*M_PI/180.0);	// 角度下限
	dJointSetHingeParam(BodyJoint[0], dParamHiStop,  90.0*M_PI/180.0);	// 角度上限
	
	// 脚の生成 -------------------------------------------------------------------//
	// 質量[kg]
	//                  60g    60g    60g    65g    65g    5g    60g    20g
	dReal LegMass[8] = {60E-3, 60E-3, 60E-3, 65E-3, 65E-3, 5E-3, 60E-3, 20E-3 };




	/*dReal LegMass[8] = {
		387E-3,
		100E-3,
		(100 + 387 + 387 + 20 + 20) * 10e-3,
		(230 + 387) * 10e-3, 
		230E-3,
		(72 + 72 + 100 + 20 + 20) * 10e-3,
		100E-3,
		100E-3
	};
*/
	// 脚のサイズ設定
	dReal LegSize[8][3] =
	{
		{  9.0E-2, 4.5E-2, 3.0E-2 },	// LegX[0]のxyz長さ
		{ 15.1E-2, 7.7E-2, 9.4E-2 },	// LegX[1]のxyz長さ
		{ 11.7E-2, 3.2E-2, 9.3E-2 },	// LegX[2]のxyz長さ
		{ 5.3E-2, 11.1E-2, 28.8E-2 },	// LegX[3]のxyz長さ
		{ 5.3E-2, 11.1E-2, 28.8E-2 },	// LegX[4]のxyz長さ
		{ 11.5E-2, 7.6E-2, 7.1E-2 },	// LegX[5]のxyz長さ
		{ 14.8E-2, 5.0E-2, 8.8E-2 },	// LegX[6]のxyz長さ
		{ 25.0E-2, 14.0E-2, 1.0E-2 }	// LegX[7]のxyz長さ
	};
	// 脚の位置座標設定（左脚座標、右脚はy座標反転）
	dReal LegPos[8][3] =
	{
		{ - 2.25e-2, 9.0e-2,  - 1.5e-2 },	// LegX[0]のx,y,z座標
		{ - 3.05e-2, 9.0e-2,  - 9.7e-2 },	// LegX[1]のx,y,z座標
		{ - 3.25e-2, 9.55e-2, - 14.95e-2 },	// LegX[2]のx,y,z座標	
		{    0.0E-2, 9.55e-2, - 25.0E-2 },	// LegX[3]のx,y,z座標
		{    0.0E-2, 9.55e-2, - 50.0e-2 },	// LegX[4]のx,y,z座標
		{    -3.2E-2, 9.55e-2, - 61.55e-2 },	// LegX[5]のx,y,z座標
		{    -3.2E-2, 9.0e-2,  - 65.0e-2 },	// LegX[6]のx,y,z座標
		{    0.0E-2, 11.2e-2, - 69.9e-2 }	// LegX[7]のx,y,z座標
	};
	// ジョイントの座標設定（左脚座標、右脚はy座標反転）
	dReal LegAnchor[6][3] =
	{
		{ 0.0E-2, 9.0e-2, -12.5e-2},		// LegX[0]とLegX[1](ヒンジ)
		{ 0.0E-2, 9.0e-2, -12.5e-2},		// LegX[1]とLegX[2](ヒンジ)
		{ 0.0E-2, 9.0e-2, -12.5e-2},	// LegX[2]とLegX[3](ヒンジ)
		{ 0.0E-2, 9.0e-2, -37.5e-2},	// LegX[3]とLegX[4](ヒンジ)
		{ 0.0E-2, 9.0e-2, -62.5e-2},	// LegX[4]とLegX[5](ヒンジ)
		{ 0.0E-2, 9.0e-2, -62.5e-2}		// LegX[5]とLegX[6](ヒンジ)
	};
	// ヒンジジョイント軸方向
	dReal LegAxis[6][3] = 
	{	
		{  0,  0, -1 },
		{ -1,  0,  0 },
		{  0, -1,  0 },
		{  0, -1,  0 },
		{  0, -1,  0 },
		{ -1,  0,  0 }
	};

	// 脚生成
	for(int i=0;i<8;i++){
		// 左脚生成
		LegL[i].setMass(LegMass[i]);									// 質量の設定
		LegL[i].setSize(LegSize[i][0], LegSize[i][1], LegSize[i][2]);	// サイズの設定
		LegL[i].setPos(LegPos[i][0], LegPos[i][1], LegPos[i][2]);		// 位置の設定
		LegL[i].setAngle(1, 0, 0, 0*M_PI/180.0);						// 傾きの設定
		createBox(LegL[i], x0, y0, z0);									// パーツ生成
		// 右脚生成
		LegR[i].setMass(LegMass[i]);									// 質量の設定
		LegR[i].setSize(LegSize[i][0], LegSize[i][1], LegSize[i][2]);	// サイズの設定
		LegR[i].setPos(LegPos[i][0], -LegPos[i][1], LegPos[i][2]);		// 位置の設定
		LegR[i].setAngle(1, 0, 0, 0*M_PI/180.0);						// 傾きの設定
		createBox(LegR[i], x0, y0, z0);									// パーツ生成
	}

	// ジョイントの生成
	for(int i=0;i<6;i++){
			// 左脚ヒンジジョイントの設定
			LegLJoint[i] = dJointCreateHinge(world, 0);
			dJointAttach(LegLJoint[i], LegL[i].body,LegL[i+1].body);
			dJointSetHingeAnchor(LegLJoint[i], x0+LegAnchor[i][0], y0+LegAnchor[i][1], z0+LegAnchor[i][2]);
			dJointSetHingeAxis(LegLJoint[i], LegAxis[i][0], LegAxis[i][1], LegAxis[i][2]);
			//dJointSetHingeParam(LegLJoint[i], dParamBounce, 0);
			dJointSetHingeParam(LegLJoint[i], dParamLoStop, -1 * M_PI);
			dJointSetHingeParam(LegLJoint[i], dParamHiStop, 1 * M_PI);
			// 右脚ヒンジジョイントの設定
			LegRJoint[i] = dJointCreateHinge(world, 0);
			dJointAttach(LegRJoint[i], LegR[i].body,LegR[i+1].body);
			dJointSetHingeAnchor(LegRJoint[i], x0+LegAnchor[i][0], y0-LegAnchor[i][1], z0+LegAnchor[i][2]);
			dJointSetHingeAxis(LegRJoint[i], LegAxis[i][0], LegAxis[i][1], LegAxis[i][2]);
			//dJointSetHingeParam(LegRJoint[i], dParamBounce, 0);
			dJointSetHingeParam(LegRJoint[i], dParamLoStop, -1 * M_PI);
			dJointSetHingeParam(LegRJoint[i], dParamHiStop, 1 * M_PI);

	}

	LegLJoint[6] = dJointCreateFixed(world, 0);				// 固定関節
	dJointAttach(LegLJoint[6], LegL[6].body, LegL[7].body);	// 固定関節の取付け
	dJointSetFixed(LegLJoint[6]);							// 固定関節の設定

	LegLJoint[7] = dJointCreateFixed(world, 0);				// 固定関節
	dJointAttach(LegLJoint[7], Body[1].body, LegL[0].body);	// 固定関節の取付け
	dJointSetFixed(LegLJoint[7]);							// 固定関節の設定

	LegRJoint[6] = dJointCreateFixed(world, 0);				// 固定関節
	dJointAttach(LegRJoint[6], LegR[6].body, LegR[7].body);	// 固定関節の取付け
	dJointSetFixed(LegRJoint[6]);							// 固定関節の設定

	LegRJoint[7] = dJointCreateFixed(world, 0);				// 固定関節
	dJointAttach(LegRJoint[7], Body[1].body, LegR[0].body);	// 固定関節の取付け
	dJointSetFixed(LegRJoint[7]);							// 固定関節の設定

	//**************追加、空中で止めるためのもの***************************//
	joint = dJointCreateFixed(world, 0);				// 固定関節
	dJointAttach(joint, Body[0].body, NULL);	// 固定関節の取付け
	dJointSetFixed(joint);
	//*********************************************************************//


	// センサの生成 -----------------------------------------------------------------//
	// 各パーツのサイズ
	dReal SensorSize[4][3] = {
		{ 4.0E-2, 1.0E-2, 2.0E-2 },	// 左前センサーのサイズ
		{ 4.0E-2, 1.0E-2, 2.0E-2 }, // 右前センサーのサイズ
		{ 4.0E-2, 1.0E-2, 2.0E-2 }, // 左後センサーのサイズ
		{ 4.0E-2, 1.0E-2, 2.0E-2 }  // 右後センサーのサイズ
	};

	// 各パーツの初期位置
	dReal SensorPos[4][3] = {
		{ 10.5E-2 , 11.2e-2 +7.5e-2, - 69.9e-2 -0.5e-2 },
		{ 10.5E-2 , 11.2e-2 -7.5e-2, - 69.9e-2 -0.5e-2 },
		{ -10.5E-2 , 11.2e-2 -7.5e-2, - 69.9e-2 -0.5e-2 },
		{ -10.5E-2 , 11.2e-2 +7.5e-2, - 69.9e-2 -0.5e-2 }
	};

	// 各パーツの質量
	dReal SensorMass[4] = {1E-3, 1E-3, 1E-3, 1E-3};

	// センサ生成
	for(int i=0;i<4;i++){
		// 左足センサ生成
		SensorL[i].setMass(SensorMass[i]);							// 質量の設定
		SensorL[i].setSize(SensorSize[i][0], SensorSize[i][1], SensorSize[i][2]);	// サイズの設定
		SensorL[i].setPos(SensorPos[i][0], SensorPos[i][1], SensorPos[i][2]);		// 位置の設定
		SensorL[i].setAngle(1, 0, 0, 0*M_PI/180.0);					// 傾きの設定
		createBox(SensorL[i], x0, y0, z0);							// パーツ生成
		// 左足センサバネ生成
		/*
		SpringL[i].setMass(SensorMass[i]);							// 質量の設定
		SpringL[i].setSize(SensorSize[i][0], SensorSize[i][1], SensorSize[i][2]);	// サイズの設定
		SpringL[i].setPos(SensorPos[i][0], SensorPos[i][1], SensorPos[i][2]-0.5E-2);		// 位置の設定
		SpringL[i].setAngle(1, 0, 0, 0*M_PI/180.0);					// 傾きの設定
		createBox(SpringL[i], x0, y0, z0);							// パーツ生成
*/
		// 左足センサをLegX[7]に固定
		SenLJoint[i] = dJointCreateFixed(world, 0);					// 固定関節
		dJointAttach(SenLJoint[i], SensorL[i].body, LegL[7].body);	// 固定関節の取付け
		dJointSetFixed(SenLJoint[i]);								// 固定関節の設定
		/*
		// 左足センサバネを設定
		SrgLJoint[i] = dJointCreateSlider(world, 0);				// スライダーセンサ
		dJointAttach(SrgLJoint[i], LegL[7].body, SpringL[i].body);	
		dJointSetSliderAxis(SrgLJoint[i],0,0,1);
		dJointSetSliderParam(SrgLJoint[i],dParamLoStop, 0);	
		dJointSetSliderParam(SrgLJoint[i],dParamHiStop, 0);
		dJointSetSliderParam(SrgLJoint[i],dParamStopCFM,0.2);
		dJointSetSliderParam(SrgLJoint[i],dParamStopERP,0.2); 
*/

		// 右足センサ生成
		SensorR[i].setMass(SensorMass[i]);							// 質量の設定
		SensorR[i].setSize(SensorSize[i][0], SensorSize[i][1], SensorSize[i][2]);	// サイズの設定
		SensorR[i].setPos(SensorPos[i][0], -SensorPos[i][1], SensorPos[i][2]);		// 位置の設定
		SensorR[i].setAngle(1, 0, 0, 0*M_PI/180.0);					// 傾きの設定
		createBox(SensorR[i], x0, y0, z0);							// パーツ生成
		/*
		// 右足センサバネ生成
		SpringR[i].setMass(SensorMass[i]);							// 質量の設定
		SpringR[i].setSize(SensorSize[i][0], SensorSize[i][1], SensorSize[i][2]);	// サイズの設定
		SpringR[i].setPos(SensorPos[i][0], -SensorPos[i][1], SensorPos[i][2]-0.5E-2);		// 位置の設定
		SpringR[i].setAngle(1, 0, 0, 0*M_PI/180.0);					// 傾きの設定
		createBox(SpringR[i], x0, y0, z0);							// パーツ生成
*/
		// 右足センサをLegX[7]に固定
		SenRJoint[i] = dJointCreateFixed(world, 0);					// 固定関節
		dJointAttach(SenRJoint[i], SensorR[i].body, LegR[7].body);	// 固定関節の取付け
		dJointSetFixed(SenRJoint[i]);								// 固定関節の設定
/*
		// 右足センサバネを設定
		SrgRJoint[i] = dJointCreateSlider(world, 0);				// スライダーセンサ
		dJointAttach(SrgRJoint[i], LegR[7].body, SpringR[i].body);	
		dJointSetSliderAxis(SrgRJoint[i],0,0,1);
		dJointSetSliderParam(SrgRJoint[i],dParamLoStop, 0);	
		dJointSetSliderParam(SrgRJoint[i],dParamHiStop, 0);
		dJointSetSliderParam(SrgRJoint[i],dParamStopCFM,0.2);
		dJointSetSliderParam(SrgRJoint[i],dParamStopERP,0.2); 
		*/
	}

}



/*****************************************************************************************/
// ロボットの描画メソッド
/*****************************************************************************************/
void Robot::drawRobot()
{

	// パーツの描画
	drawBox(Body[0], 1.0, 1.0, 1.0, 0.8);
	drawBox(Body[1], 0.0, 0.0, 0.0, 0.8);

	for(int i=0;i<8;i++){
		drawBox(LegL[i], 0.0, 0.0, 0.0, 0.8);
		drawBox(LegR[i], 0.0, 0.0, 0.0, 0.8);
	}

	for(int i=0;i<4;i++){
		drawBox(SensorL[i], 0.0, 0.5, 1.0, 0.8);
		drawBox(SensorR[i], 0.0, 0.5, 1.0, 0.8);
		//drawBox(SpringL[i], 0.5, 0.5, 1.0, 0.8);
		//drawBox(SpringR[i], 0.5, 0.5, 1.0, 0.8);
	}

}

// 倒立振子の描画（重心位置とベースからの直線）
void Robot::drawPendulum()
{
	// 重心位置の描画
	const dReal *R;
	R = dBodyGetRotation(Body[0].body);
	dsSetColor(1.0, 0.0, 0.0); 
	dsDrawSphereD(absGl, R, (float)0.02);
	
	//body[1]の位置表示
	const dReal *body0;
	body0 = dBodyGetPosition(Body[1].body);
	dsDrawSphereD(body0, R, (float)0.02);

	// ベースから重心への直線描画
	const dReal *base;
	if(getL_or_R()==-1){
		base = dBodyGetPosition(LegL[7].body);
	}else{
		base = dBodyGetPosition(LegR[7].body);
	}
	dsDrawLineD(absGl,base);

}

// 着地目標位置の描画
void Robot::drawRefPos()
{
	// 着地目標位置の描画
	const dReal *R;
	R = dBodyGetRotation(Body[0].body);
	dsSetColor(1.0, 0.0, 0.0);

	dsDrawSphereD(refPos, R, (float)0.02);

}




/*****************************************************************************************/
// ロボットの破壊メソッド
/*****************************************************************************************/
void Robot::destroyRobot()
{
	// バーツの破壊
	for (int i = 0; i < 2; i++) {
		dBodyDestroy(Body[i].body);
		dGeomDestroy(Body[i].geom); 
		
	}
	for (int i = 0; i < 8; i++) {
		dBodyDestroy(LegL[i].body);
		dGeomDestroy(LegL[i].geom);
		dBodyDestroy(LegR[i].body);
		dGeomDestroy(LegR[i].geom);
		
	}
	for (int i = 0; i < 4; i++) {
		dBodyDestroy(SensorL[i].body);
		dGeomDestroy(SensorL[i].geom);
		dBodyDestroy(SensorR[i].body);
		dGeomDestroy(SensorR[i].geom);
		/*
		dBodyDestroy(SpringL[i].body);
		dGeomDestroy(SpringL[i].geom);
		dBodyDestroy(SpringR[i].body);
		dGeomDestroy(SpringR[i].geom);
		*/
	}

	// ジョイントの破壊
	dJointDestroy(BodyJoint[0]);
	for(int i = 0; i < 8; i++){
		dJointDestroy(LegLJoint[i]);
		dJointDestroy(LegRJoint[i]);
	}
	for(int i = 0; i < 4; i++){
		dJointDestroy(SenLJoint[i]);
		dJointDestroy(SenRJoint[i]);
		/*
		dJointDestroy(SrgLJoint[i]);
		dJointDestroy(SrgRJoint[i]);
		*/
	}
}
