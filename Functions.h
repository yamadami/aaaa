/*****************************************************************************************/
//                           　　　　　各種関数宣言	
/*****************************************************************************************/
//■2013/2/1 関数をこちらにまとめることにする
//
//
/*****************************************************************************************/


//------------------------------------------------------------------------------------------
// パーツの位置取得関数（ODEシミュレーション時とマイコン時で切り替えやすくするため）
//------------------------------------------------------------------------------------------

// パーツの絶対値でのX座標取得
double getPartsPosX(dBodyID body){
	const dReal *pos;
	pos = dBodyGetPosition(body);
	return pos[0];
}
// パーツの絶対値でのY座標取得
double getPartsPosY(dBodyID body){
	const dReal *pos;
	pos = dBodyGetPosition(body);
	return pos[1];
}
// パーツの絶対値でのZ座標取得
double getPartsPosZ(dBodyID body){
	const dReal *pos;
	pos = dBodyGetPosition(body);
	return pos[2];
}
//
double getHingeAngle(dJointID joint){
	return dJointGetHingeAngle(joint);
}




//------------------------------------------------------------------------------------------
// 関節モータへの命令設定
// - joint:操作するジョイント番号
// - refAngle:目標ジョイント角度
//------------------------------------------------------------------------------------------
void setHingeParams(dJointID joint, dReal refAngle)
{
	dReal kp=20;//40;	// 比例ゲイン
	dReal tmpAngle = getHingeAngle(joint);	// 現在ジョイント角度の取得
	dReal diff = refAngle -tmpAngle;			// 操作量の計算

	// ODEのトルク制御にはバグがあり、以下の二文を用いてトルク制御とする
	dJointSetHingeParam(joint, dParamVel, kp*diff);	// モータ速度設定
	dJointSetHingeParam(joint, dParamFMax, 100);	// モータ最大トルク設定
}



//------------------------------------------------------------------------------------------
// ボックス関係の自作クラス・関数 
//------------------------------------------------------------------------------------------
// ボックスオブジェクトクラス
class MyBoxLink{
	
public:
	// メンバ変数
	dBodyID body;				// ボディ(剛体)のID番号（動力学計算用）
	dGeomID geom;				// ジオメトリのID番号(衝突検出計算用）
	dReal mass;					// 質量[kg]
	dReal sizeX,sizeY,sizeZ;	// サイズ[m]
	dReal posX,posY,posZ;		// 初期位置[m]
	dReal axisX,axisY,axisZ;	// 初期傾き軸
	dReal angle;				// 初期傾き角度[rad]

	// セッター
	// 質量の設定
	void setMass(dReal m){
		mass = m;
	}
	// 位置の設定
	void setPos(dReal x, dReal y, dReal z){
		posX = x;	
		posY = y;		
		posZ = z;
	}
	// サイズの設定
	void setSize(dReal x, dReal y, dReal z){
		sizeX = x;
		sizeY = y;
		sizeZ = z;
	}
	// 傾きの設定
	// x:ｘ軸, y:ｙ軸, z:ｚ軸
	// a:傾き角度(rad)
	void setAngle(dReal x, dReal y, dReal z, dReal a){
		axisX = x;
		axisY = y;
		axisZ = z;
		angle = a;
	}

};


// ボックスの描画
void drawBox(MyBoxLink box, dReal red, dReal green, dReal blue, dReal alpha)
{
	dsSetColorAlpha((float)red, (float)green, (float)blue, (float)alpha); 
	dVector3 sides;
	dGeomBoxGetLengths(box.geom,sides);
	dsDrawBox(dBodyGetPosition(box.body),dBodyGetRotation(box.body),sides);
}


// ボックスの生成
void createBox(MyBoxLink &box, dReal x0, dReal y0, dReal z0)
{
	dMass mass;
	dMatrix3 R;

	box.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, box.mass, box.sizeX, box.sizeY, box.sizeZ);
	dBodySetMass(box.body, &mass);
	dBodySetPosition(box.body, x0+box.posX, y0+box.posY, z0+box.posZ);
	box.geom = dCreateBox(space, box.sizeX, box.sizeY, box.sizeZ);
	dGeomSetBody(box.geom, box.body);

	dRFromAxisAndAngle(R, box.axisX, box.axisY, box.axisZ, box.angle);
	dBodySetRotation(box.body,R);

}
//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------
// 重心取得関係の関数
//------------------------------------------------------------------------------------------

double sign(double x){
	if(x>0){
		return 1.0;
	}else if(x == 0){
		return 0.0;
	}else{
		return -1.0;
	}
}

//------------------------------------------------------------------------------------------
// モーター命令決定のための関数
//------------------------------------------------------------------------------------------
double getSliderForce(double ref_length, double cur_length, double cur_vel){
	double velCoeff = 10.0; //15; // 10; // 1;
	double lenCoeff = 100.0;

	return  lenCoeff * (ref_length - cur_length) - velCoeff * (cur_vel);
}



//------------------------------------------------------------------------------------------
// ベクトル・行列計算
//------------------------------------------------------------------------------------------
void inverse(dReal r[][4], const dReal a[][4]){							//	逆変換行列
	double	d=0;
	int i,j;
	for(i=0; i<3; i++)
		d=d+a[0][i] * a[1][ (i+1) % 3 ] * a[2][(i+2) % 3 ]	
			- a[0][i] * a[1][(i+2) % 3] * a[2][(i+1) % 3 ];
	if(d!=0){
		for(  i=0; i<3; i++){							//	拡大・回転
			for(j=0; j<3; j++){
				r[i][j]=( a[(j+1) %  3][ (i+1) %  3] * a[ (j+2) % 3 ][ (i+2) % 3 ]
						-a[ (j+1) %  3 ][ (i+2) % 3 ] * a[ (j+2) % 3 ][ (i+1) % 3 ] ) / d;
				r[3][j] = 0;
			}
			r[i][3] = 0;
		}
	}
}


void ftrvadd( dReal r[], const dReal a[], const dReal _ld_ext_a[][4] ){
	int i;
	double w[4] = {0.0};




	for(i=0;i<4;i++){
		w[i] = _ld_ext_a[i][0]*a[0]
			+ _ld_ext_a[i][1]*a[1] 
			+ _ld_ext_a[i][2]*a[2]
			+ _ld_ext_a[i][3]*a[3];
	}

	for(i=0;i<4;i++){
		r[i] = w[i];
	}

}


// ベクトル引き算　r = a-b
void vec_minus(dReal r[], const dReal a[], const dReal b[] ){
	int i;

	for(i=0;i<4;i++){
		r[i] = a[i] - b[i];
	}
}

// ベクトルコピー　r = a
void vec_copy(double r[], const dReal *a){
	int i;

	for(i=0;i<4;i++){
		r[i] = a[i];
	}
}

void vec_copy(double r[], const float *a){
	int i;

	for(i=0;i<4;i++){
		r[i] = a[i];
	}
}

// ベクトルの長さを取得
double length(double a[]){
	double sum=0;
	int i;

	for(i=0;i<3;i++){
		sum += a[i] * a[i];
	}
	
	return sqrtf(sum);
}



// ベクトルコピー　r = a
void mat_copy(float r[][4],float a[][4]){
	int i,j;

	for (i=0; i<4; i++){
		for(j=0;j<4;j++){
			r[i][j] = a[i][j];
		}
	}
}

float min(float a, float b){
	if (a>b) {return b;} else {return a;}
}



// ==========================================================================================================================================================================
//追加関数　田原　使ってないのもあるかも
float _ld_ext_a[4][4];

void ftrvadd( float a[], float b[], float r[] ){
	int i;
	float w[4] = {0.0};


	for(i=0;i<4;i++){
		w[i] = _ld_ext_a[i][0]*a[0] + _ld_ext_a[i][1]*a[1] 
			 + _ld_ext_a[i][2]*a[2] + _ld_ext_a[i][3]*a[3]
			 + b[i];
	}

	for(i=0;i<4;i++){
		r[i] = w[i];
	}

}

void ftrvadd( dReal a[], float b[], dReal r[] ){
	int i;
	float w[4] = {0.0};


	for(i=0;i<4;i++){
		w[i] = _ld_ext_a[i][0]*a[0] + _ld_ext_a[i][1]*a[1] 
			 + _ld_ext_a[i][2]*a[2] + _ld_ext_a[i][3]*a[3]
			 + b[i];
	}

	for(i=0;i<4;i++){
		r[i] = w[i];
	}

}

void ftrvadd( float a[], const dReal *b, float r[] ){
	int i;
	double w[4] = {0.0};


	for(i=0;i<4;i++){
		w[i] = _ld_ext_a[i][0]*a[0] + _ld_ext_a[i][1]*a[1] 
			 + _ld_ext_a[i][2]*a[2] + _ld_ext_a[i][3]*a[3]
			 + b[i];
	}

	for(i=0;i<4;i++){
		r[i] = w[i];
	}

}

void ftrvadd( float a[], const dReal *b, double r[] ){
	int i;
	double w[4] = {0.0};


	for(i=0;i<4;i++){
		w[i] = _ld_ext_a[i][0]*a[0] + _ld_ext_a[i][1]*a[1] 
			 + _ld_ext_a[i][2]*a[2] + _ld_ext_a[i][3]*a[3]
			 + b[i];
	}

	for(i=0;i<4;i++){
		r[i] = w[i];
	}

}

void ftrvadd( double a[], const dReal *b, double r[] ){
	int i;
	double w[4] = {0.0};


	for(i=0;i<4;i++){
		w[i] = _ld_ext_a[i][0]*a[0] + _ld_ext_a[i][1]*a[1] 
			 + _ld_ext_a[i][2]*a[2] + _ld_ext_a[i][3]*a[3]
			 + b[i];
	}

	for(i=0;i<4;i++){
		r[i] = w[i];
	}

}

void vec_copy(float r[], const float a[]){
	int i;

	for(i=0;i<4;i++){
		r[i] = a[i];
	}
}

void vec_copy(float r[], const dReal *a){
	int i;

	for(i=0;i<4;i++){
		r[i] = a[i];
	}
}


void vec_minus(float r[], const float a[], const float b[] ){
	int i;

	for(i=0;i<4;i++){
		r[i] = a[i] - b[i];
	}
}
void vec_minus(double r[], const double a[], const float b[] ){
	int i;

	for(i=0;i<4;i++){
		r[i] = a[i] - b[i];
	}
}


void ld_ext(const float a[][4]){
	int i,j;

	for(i=0;i<4;i++){
		for(j=0;j<4;j++){
			_ld_ext_a[i][j] = a[i][j];
		}
	}
}

void a_vec(float r[], const float a, const float b[] ){
	int i;

	for(i=0;i<4;i++){
		r[i] = a * b[i];
	}

}

void a_vec(double r[], const double a, const double b[] ){
	int i;

	for(i=0;i<4;i++){
		r[i] = a * b[i];
	}

}

void a_vec(double r[], const double a, const float b[] ){
	int i;

	for(i=0;i<4;i++){
		r[i] = a * b[i];
	}

}

void vec_add(float r[], const float a[], const float b[] ){
	int i;

	for(i=0;i<4;i++){
		r[i] = a[i] + b[i];
	}
}

void vec_add(float r[], const float a[], const dReal b[] ){
	int i;

	for(i=0;i<4;i++){
		r[i] = a[i] + b[i];
	}
}

void mat12_to_mat44(const dReal *mat12, dReal mat44[][4]){
	int i,j;

	for (i=0;i<3;i++){
		for(j=0;j<3;j++){
			mat44[i][j] = mat12[4*i + j];
		}
		mat44[i][3] = 0.0;
	}
	for (j=0;j<4;j++){
		mat44[3][j] = 0.0;
	}
}
void inverse(float r[][4], const float a[][4]){							//	逆変換行列
	float	d=0;
	int i,j;
	for(i=0; i<3; i++)
		d=d+a[0][i] * a[1][ (i+1) % 3 ] * a[2][(i+2) % 3 ]	
			- a[0][i] * a[1][(i+2) % 3] * a[2][(i+1) % 3 ];
	if(d!=0){
		for(  i=0; i<3; i++){							//	拡大・回転
			for(j=0; j<3; j++){
				r[i][j]=( a[(j+1) %  3][ (i+1) %  3] * a[ (j+2) % 3 ][ (i+2) % 3 ]
						-a[ (j+1) %  3 ][ (i+2) % 3 ] * a[ (j+2) % 3 ][ (i+1) % 3 ] ) / d;
				r[3][j] = 0;
			}
			r[i][3] = 0;
		}
	}
}

double deg2rad(double deg){
    return  M_PI / 180 * deg;
}

void inverse(dReal r[][4], const float a[][4]){							//	逆変換行列
	float	d=0;
	int i,j;
	for(i=0; i<3; i++)
		d=d+a[0][i] * a[1][ (i+1) % 3 ] * a[2][(i+2) % 3 ]	
			- a[0][i] * a[1][(i+2) % 3] * a[2][(i+1) % 3 ];
	if(d!=0){
		for(  i=0; i<3; i++){							//	拡大・回転
			for(j=0; j<3; j++){
				r[i][j]=( a[(j+1) %  3][ (i+1) %  3] * a[ (j+2) % 3 ][ (i+2) % 3 ]
						-a[ (j+1) %  3 ][ (i+2) % 3 ] * a[ (j+2) % 3 ][ (i+1) % 3 ] ) / d;
				r[3][j] = 0;
			}
			r[i][3] = 0;
		}
	}
}

//*********************************************************************************//
//			functions.hからここに移動 山田6.19
//*********************************************************************************//

double dr_con( double dig){

	return dig * M_PI / 180;

}

//*********************************************************************************//
//				角度設定の関数(角度上限の設定)
//*********************************************************************************//
void epwm_set( int L_or_R , int port, double theta){

	//int fix;	
	if(L_or_R==0){
		switch(port){
			case 0:
				//fix = 0x0249;
				if( theta < dr_con(-30) ) theta=dr_con(-30);
				if( theta > dr_con( 30) ) theta=dr_con( 30);
				break;
			case 1:
				//fix = 0x0286;
				if( theta < dr_con( -5) ) theta=dr_con( -5);
				if( theta > dr_con( 40) ) theta=dr_con( 40);
				break;
			case 2:
				//fix = 0x0353;
				if( theta < dr_con(-55) ) theta=dr_con(-55);
				if( theta > dr_con( 55) ) theta=dr_con( 55);
				break;
			case 3:
				//fix = 0x0428;
				if( theta < dr_con(-55) ) theta=dr_con(-55);
				if( theta > dr_con( 55) ) theta=dr_con( 55);
				break;
			case 4:
				//fix = 0x0356;
				if( theta < dr_con(-50) ) theta=dr_con(-50);
				if( theta > dr_con( 45) ) theta=dr_con( 45);
				break;
			case 5:
				//fix = 0x01A5;
				if( theta < dr_con(-30) ) theta=dr_con(-30);
				if( theta > dr_con( 30) ) theta=dr_con( 30);
				break;
		}
	}else{
		switch(port){
			case 0:
				//fix = 0x0315;
				if( theta < dr_con(-30) ) theta=dr_con(-30);
				if( theta > dr_con( 30) ) theta=dr_con( 30);
				break;
			case 1:
				//fix = 0x02E7;
				if( theta < dr_con(-40) ) theta=dr_con(-40);
				if( theta > dr_con(  5) ) theta=dr_con(  5);
				break;
			case 2:
				//fix = 0x02B4;
				if( theta < dr_con(-50) ) theta=dr_con(-50);
				if( theta > dr_con( 50) ) theta=dr_con( 50);
				break;
			case 3:
				//fix = 0x02F5;
				if( theta < dr_con(-50) ) theta=dr_con(-50);
				if( theta > dr_con( 50) ) theta=dr_con( 50);
				break;
			case 4:
				//fix = 0x02D8;
				if( theta < dr_con(-50) ) theta=dr_con(-50);
				if( theta > dr_con( 50) ) theta=dr_con( 50);
				break;
			case 5:
				//fix = 0x0336;
				if( theta < dr_con(-35) ) theta=dr_con(-35);
				if( theta > dr_con( 35) ) theta=dr_con( 35);
				break;
		}
	}
	
	//********************************この部分を変更する**************************************//
	//　ユニット番号、ユニットポート番号、トリガカウンタ、トリガカウンタ						//
	//pwm_set( L_or_R, port, 0, ( (18*1600/(26*M_PI))*(theta+M_PI/2) + 700 ) * PWM_UNIT_COUNT + fix);
	//*****************************************************************************************//
}



//********************************************************************************************//

