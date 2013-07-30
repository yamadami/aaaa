/*****************************************************************************************/
//                          ���{�b�g�h��PID�R���g���[�����ڃN���X	
/*****************************************************************************************/
//							2013/07/19 ����0.20�オ���Ă�����
//
/*****************************************************************************************/



class Robot_ZMP : public Robot
{
private:


public:

	double refVel;
	double refHight;
	double poszErrInteg;

	//******************************�ǉ�************************************************//
	double Leg_point[1000][3];			//Leg_point[][0]�͍��E�m�F,[][1]��x���W[][2]��y���W calcZMP,simLoop���Ŏg�p
	#define I 100						//���s�����@���s��
	#define Ts 50						//������̕������@
	#define O   1						//������̕������̕\�������������@�@Ts50�AO5�Ȃ�Ts��5�̔{���������\��
	#define zc 0.05				//�S�����ʂ̍��� �����C�ӂɌ��肷��
	double Pxfix[I] ;					//�C���������n�ʒu(�e���̒��n�ʒu)
	double Pyfix[I] ;					//�C���������n�ʒu
	double Point_x,Point_y,Point_z;		//�����̒l
	double Point_xr,Point_yr,Point_zr;	//�E���̒l
	double sohenx[I][Ts], soheny[I][Ts];//���s�f�Ђ̒l [M][N]M���ڂ�N(1~50)�Ԗ�
	double kari_y[I][Ts];
	double endx[I], endy[I];			//
	int count ;
	
	//**********************************************************************************//

	//**************************�ǉ�**********************************************//
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
		//	�����ōŏ��Ɍv�Z����	//
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
// �R���g���[���[(���C�����[�v)
/*****************************************************************************************/
void Robot_ZMP::controler(Robot_ZMP pre, double steptime, int steps){
	// step1 ���{�b�g�̏�Ԏ擾
	simloop(pre);
	// step2 ���[�^�̊p�x���擾
	// ��
	// step3 �d�S���̎擾

	/*
	for(int i=0;i<I;i++){
		for(int T=0;T<Ts;T++){
			printf("%lf\n",kari_y[i][T]);
		}
		printf("%d\n",i);
	}
	*/


	//**********************************************//
	//			�����Ɍv�Z�͂���Ȃ�				//
	//calcGlPos();				// �d�S�v�Z
	//calcGlVel(pre,steptime);	// �d�S���x�v�Z
	//calcGlAcc(pre, steptime);	// �d�S�����x�v�Z
	//**********************************************//


	// ���n�ڕW�ʒu(x,y,z)�Ƒ��悪���l(10e-2)�ȏ㗣��Ă����ꍇ��
	// �萔�{(<1)���g���ĖڕW�l������ł����D
	//Ashisaki();	//�g��Ȃ��������肷��ۂ�

	// step5 �L���͂̌v�Z
	//double f = shinchouryoku(pre);
	//printf("f=%lf\n",f);

	// step6 �V�r�E�x���r�̊֐ߊp�x�����߂�
	//calcRefJoint(pre, refPos);

	// step7 ���[�^�E�X���C�_�ɗ͂�ݒ肷��
	setMotor(steps);
}

/*****************************************************************************************/
// �t�^���w�{���[�^�̊p�x�Z�b�g.(�R�c)
/*****************************************************************************************/

void Robot_ZMP::move_Foot(double l,double m,double n, int L_or_R){
	

	//printf("aa");
	//************�t�^���w�A���̊֐ߊp�x�̌���***//
	InverseKinematics(l, m, n, L_or_R);
	//*******************************************//


	//****************�������͂���Ȃ�********************//
	/*
	if( L_or_R == 0 ){
		epwm_set( 0, 0,theta[1]-3.14/36);//�����i�������[�p
		epwm_set( 0, 1,theta[2]);//�����������i�������[��
		epwm_set( 0, 2,theta[3]);//���������㕔
		epwm_set( 0, 3,theta[4]);//������������
		epwm_set( 0, 4,theta[5]);//���Ђ�
		epwm_set( 0, 5,theta[6]);//�����񗠁i���񃍁[��
	}
	if( L_or_R == 1 ){
		epwm_set( 1, 0,theta[1]+3.14/36);//�E���i�E�����[�p
		epwm_set( 1, 1,theta[2]);//�E���������i�E�����[��
		epwm_set( 1, 2,theta[3]);//�E�������㕔
		epwm_set( 1, 3,theta[4]);//�E����������
		epwm_set( 1, 4,theta[5]);//�E�Ђ�
		epwm_set( 1, 5,theta[6]);//�E���񗠁i���񃍁[��
	}
	*/
	//*****************************************************//

}

/*****************************************************************************************/
// ���{�b�g�̏�ԕϐ��̍X�V����.(�R�c)
/*****************************************************************************************/
// ���{�b�g�̏�ԕϐ��̍X�V
void Robot_ZMP::simloop(Robot_ZMP pre){
	static int step_time =0;		//�����グ��ۂ̍��݉�
	static int wait = 3;			//wait * 6 msec::: 0.2sec(33)
	static int step=0;				//����*1msec
	static int count = 0;
	static int count_step = 0;			//�����J�E���g
	double height_u=-0.40;//0.16;	//u�V�r�̎����グ�鍂��
	static double flag_Leg =0;		//�؊����t���O
	double stride_z=0.04,d=0.007;	// stride_z�����̍��ݕ� d�����̍��ݕ�
	//int count= 0;	
	//Leg_point[step][0] == 0�Ȃ�E�x����ԁA1�Ȃ獶�x����Ԃ�

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

	//**********************������Ԉێ�50step��***********************************//
	//**********************�ŏ��ȊO�����ɂ͓���Ȃ�*******************************//

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

	

	//****************************�E���x���� *****************************//
	case 1:
		//
		if(step_time<25){
			Point_z= Point_z + stride_z ; // ????????????????
			//������(���s�f�Ђ̍����|�V�r�̎����グ�鍂��)������Ȃ�---------
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
		
		//****************���̒l*********************//
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

	//****************************�E���x�����獶���x���ɐ؂�ւ��� *****************************//
	case 2: 
		Point_yr = soheny[count_step][step_time]-0.04;

		Point_y = soheny[count_step][step_time]+0.04;

		//****************���̒l*********************//
		
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

	//****************************�����x���� *************************************************//
	case 3:
		//



		if(step_time<25){
			Point_zr= Point_zr+stride_z ; // ????????????????
			//������(���s�f�Ђ̍����|�V�r�̎����グ�鍂��)������Ȃ�---------
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

		//****************���̒l*********************//
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

//********************�����x������E���x���ɐ؂�ւ���******************************//
	case 4: 
		
		//*******************************************//
		//****************���̒l*********************//
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
//	�֐����(�R�c)6.27
///*************************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// ��������////////////////////////////////////////////
//double Ts =50;	//�v���O�����p���s����
double Tsup = 0.01*Ts*2; //���s����

//double sohenx[I][Ts], soheny[I][Ts], sohenz[I][Ts];
//double sohenvx[I][Ts], sohenvy[I][Ts];

//zc���ǂ�����̂��H�H

void Robot_ZMP::calcZMP(){
	int i, j, fugou;
	double Tc, C, S, D;				//�萔
	double a = 10.0, b = 1.0;		//�]���֐��̏d��  a�ʒu�@b���x
	double Px[2], Py[2];			//���n�ʒu
	double sx[I], sy[I];			//�O�i����,���E�����̕���
	double xbar, ybar;				//���s�f�Ђ̈ʒu
	double vxbar, vybar;			//���s�f�Ђ̏I�[���x
	double xi[2], yi[2];			//���s�J�n�u�Ԃ̏d�S�ʒu
	double xidot[2], yidot[2];		//���s�J�n�u�Ԃ̑��x
	double xd, xddot, yd, yddot;	//�ڕW��Ԃ̈ʒu�Ƒ��x
	
	//�萔��`
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

	//���s�J�n�u�Ԃ̏d�S�ʒu
	xi[0] = 0.0;
	xidot[0] = 0; 

	yi[0] = 0.0/*0.1*/;
	yidot[0] = 0;

	//�������ʒu�␳
	Pxfix[0] =   0.0;
	Pyfix[0] =   0.0;
	endx[0]=0.0;
	endy[0]=0.0;
	/*******************************************************/
	/*���[�v�J�n*/
	for (i = 0 ; i < I ; i++){
		//��������
		fugou = (int)pow(-1.0,i+2);
		
		//�v�悵�����n�ʒu
		if(i>0){
			Px[1] =  Px[0] + sx[i];
			Py[1] =  Py[0] - fugou * sy[i];
		}else{
			Px[1]=Px[0];
			Py[1]=Py[0];
		}
		//����O�̒��n�ʒu�@�ۑ��p
		Px[0] =  Px[1];
		Py[0] =  Py[1];

		//���s�f�Ђ̃p�����[�^
		xbar = sx[i] / 2;
		ybar = fugou * sy[i] / 2;

		//���s�f�Ђ̏I�[���x
		vxbar = xbar * (C+1.00) /(Tc*S);
		vybar = ybar * (C-1.00) /(Tc*S);

		//(4.54��
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

		//�ڕW��Ԃ̈ʒu�Ƒ��x
		xd = Px[1] + xbar;
		xddot = vxbar;

		yd = Py[1] + ybar;
		yddot = vybar;
		
		//�C�����n�ʒu

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
		//���s�f�Ђ̋O�����v�Z
		double t;

		for( j = 0 ; j < Ts ; j++ ){
			t=Tsup/Ts*j;
			sohenx[i][j] = (xi[1]-Pxfix[i]) * cosh(t/Tc) + Tc * xidot[1] * sinh(t/Tc) + Pxfix[i];
			soheny[i][j] = (yi[1]-Pyfix[i]) * cosh(t/Tc) + Tc * yidot[1] * sinh(t/Tc) + Pyfix[i];//���ꔽ�]������ׂ��H
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

//****************************�����g��Ȃ�*********************************************//
// �V�r�x���r�؂�ւ����肘
bool Robot_ZMP::changeFootX(Robot_ZMP pre){
	double k = 0.12 * getGlVelX() + 0.10 * (refVel-getGlVelX()) + 0.02;
	//printf("k=%lf\n",k);

	//  �@�@  �d�S�ʒuX   > �����l�@&& �@�@�@�@�@�@�O�ɓ|��Ă���
	if( fabs(getGlPosX()) > fabs(k) && ( getSitaZ() - getpre_SitaZ() ) > 0 ){	//&& ( getSitaZ() - pre.getSitaZ() )
		return true;
	}else{
		return false;
	}
}

//**************************************************************************************///

//****************************�����g��Ȃ�**********************************************//
// �V�r�x���r�؂�ւ����肙
bool Robot_ZMP::changeFootY(Robot_ZMP pre){
	double k = 0.05 ;//0.05;

	//        �d�S�ʒuY�@ > �����l  &&          �O�ɓ|��Ă���@�@�@ &&  �d�S���x���r�Ƌt�ɐi��ł���
	if( fabs(getGlPosY()) > fabs(k) && (getSitaZ()-getpre_SitaZ()>0) && (getL_or_R()*getGlVelY()>0.0) ){	//&& (getSitaZ()-pre.getSitaZ()>0)
		return true;
	}else{
		return false;
	}
}
//***************************************************************************************//


//*******************************����͎g���H�g��Ȃ��H**********************************//
/*****************************************************************************************/
// ���[�^�E�X���C�_�ɗ͂�ݒ肷��
/*****************************************************************************************/
void Robot_ZMP::setMotor(int steps){
	double f_Yuukyaku=0;
	double ref_Yuukyaku_length=0;
	count = steps;
	if(steps < 50){
		
		// �����p������点��
		stat=0;
		init_refjoint();
		// �v�Z���ʂ���͂�ݒ肷��
		for(int i=0;i<6;i++){
			setHingeParams(LegLJoint[i], refLegLJointAngle[i]);
			setHingeParams(LegRJoint[i], refLegRJointAngle[i]);
		}
	

		/*
		// ����ڂ��O�i�ɂȂ�悤�ɁA0.1�b�ԗ͂�������i�w���������j�B�v���ǁB
		if(40 < steps && steps < 50){
			//dBodySetForce(Body[0].body,2,0,0);
			// ���r�t���[
			dJointSetHingeParam(LegLJoint[4], dParamFMax, 0);
			//dJointSetHingeParam(LegLJoint[5], dParamFMax, 0);
			// �E���t���[
			dJointSetHingeParam(LegRJoint[4], dParamFMax, 0);
			//dJointSetHingeParam(LegRJoint[5], dParamFMax, 0);
		}	
		*/
	}else{
		
		// �v�Z���ʂ���͂�ݒ肷��
		for(int i=0;i<6;i++){
			setHingeParams(LegLJoint[i], l_theta[i]);
			setHingeParams(LegRJoint[i], r_theta[i]);
		}
		
		// ��{�`�Ƃ��Ďx���r�̑���̓t���[
		//if(getChuukuu() == 0){
		//	if( getL_or_R() == -1 ){
				// ���r�t���[
				//printf("���r�t���[\n");
		//		dJointSetHingeParam(LegLJoint[4], dParamFMax, 0);
		//		dJointSetHingeParam(LegLJoint[5], dParamFMax, 0);
		//	}else{
				// �E���t���[
				//printf("�E�r�t���[\n");
		//		dJointSetHingeParam(LegRJoint[4], dParamFMax, 0);
		//		dJointSetHingeParam(LegRJoint[5], dParamFMax, 0);
		//	}
		}
		

}

//******************************************************************************************//

//*****************************�����g��Ȃ��͂�*********************************************//
void Robot_ZMP::Ashisaki(){
	// ���n�ڕW�ʒu(x,y,z)�Ƒ��悪���l(10e-2)�ȏ㗣��Ă����ꍇ��
	// �萔�{(<1)���g���ĖڕW�l������ł����D
	
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
			a_vec( work_a_distance  , 5e-3, work_distance );	//�Q�Ԗڂ̈����͍��ݔ{��
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
			a_vec( work_a_distance  , 5e-3, work_distance );	//�Q�Ԗڂ̈����͍��ݔ{��
			vec_add( work_chaku_moku,  work_a_distance, ashiSaki );
			vec_copy( refPos, work_chaku_moku );
			//for(int i=0;i<4;i++) printf("work_chaku_moku=%lf\n",work_chaku_moku[i]);
		}
	}
}
//*****************************************************************************************//