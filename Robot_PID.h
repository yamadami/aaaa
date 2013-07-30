/*****************************************************************************************/
//                          ���{�b�g�h��PID�R���g���[�����ڃN���X	
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
// �R���g���[���[
/*****************************************************************************************/
void Robot_PID::controler(Robot_PID pre, double steptime, int steps){
	// step1 ���{�b�g�̏�Ԏ擾
	statusUpdate(pre);
	// step2 ���[�^�̊p�x���擾
	// ��

	// step3 �d�S���̎擾
	calcGlPos();				// �d�S�v�Z
	calcGlVel(pre,steptime);	// �d�S���x�v�Z
	calcGlAcc(pre, steptime);	// �d�S�����x�v�Z

	// step4 ���x���璅�n�ڕW���v�Z

	//Sita�v�Z
	for(int i=0;i<3;i++) pre_Sita[i] = Sita[i];	//�O���ԕۑ� (statusUpdate�ɗ��p pre��p����Ɠ����l�ɂȂ��Ă��܂��̂�)
	calcSita();

	// �r�ؑւ������prev_sita����sita�ŏ㏑������i���ꂪ�Ȃ��ƕs����j
	if(pre.getL_or_R() != getL_or_R()){
		//setSitaX(pre_Sita[0]);
		//setSitaY(pre_Sita[1]);
		setSitaZ(pre_Sita[2]);
	}

	// ���n�ڕW�̌v�Z
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
		// �����n��Ԃ̂Ƃ��́C�ڕW�ʒu��ύX���Ȃ��悤�ɂ���i���{�b�g����ђ��˂錴���ɂȂ邽�߁j
		refPos[0] = pre.refPos[0];
		refPos[1] = pre.refPos[1];
	}

	if(pre.getL_or_R() != getL_or_R()){
		// �؂�ւ�������ς��Ȃ��������������H Gl���傫���ς�邩��
		refPos[0] = pre.refPos[0];
		refPos[1] = pre.refPos[1];
	}

	if ( (stat == 2) || (stat == 3) || (stat == 7) || (stat == 8) ){
		refPos[2] = 0.0;
	}else{
		refPos[2] = 0.12;
	}

	// ���n�ڕW�ʒu(x,y,z)�Ƒ��悪���l(10e-2)�ȏ㗣��Ă����ꍇ��
	// �萔�{(<1)���g���ĖڕW�l������ł����D
	//Ashisaki();	//�g��Ȃ��������肷��ۂ�

	// step5 �L���͂̌v�Z
	double f = shinchouryoku(pre);
	//printf("f=%lf\n",f);

	// step6 �V�r�E�x���r�̊֐ߊp�x�����߂�
	calcRefJoint(pre, refPos);

	// step7 ���[�^�E�X���C�_�ɗ͂�ݒ肷��
	setMotor(steps, f);
}

/*****************************************************************************************/
// ���{�b�g�̏�ԕϐ��̍X�V����D 
/*****************************************************************************************/
// ���{�b�g�̏�ԕϐ��̍X�V
void Robot_PID::statusUpdate(Robot_PID pre){
	int numOnSensorR = SensorRCheck();
	int numOnSensorL = SensorLCheck();
	//printf("SenR:%d\n",numOnSensorR);
	//printf("SenL:%d\n",numOnSensorL);

	int kirikae_jouken_flag=0;
	static int interval=0;

	// ��Ԃ̐ݒ�
	switch(stat){
	case 1: // �����x���� --------------------------------------------------------------/
		interval++;
		if( interval > 5 ){
			// �؂�ւ������@
			if ( changeFootX(pre) ){
				kirikae_jouken_flag = 1;
				printf("���؂�ւ�\n");
			}
			// �؂�ւ������A
			if ( changeFootY(pre) ){	
				kirikae_jouken_flag = 2;
				printf("���؂�ւ�\n");
			}

			// ��̐ؑւ����������������ꍇ�̏���
			if( kirikae_jouken_flag != 0 ){
				// ��Ԃ�i�߂�
				stat = 2;
				interval =0; 
			}
		}
		break;

	case 2: // �E�����n���쒆 ---------------------------------------------------
		interval++;
		if ( numOnSensorR >=1 ){
			// ��Ԃ�i�߂�
			stat = 3;
			interval =0;
		}
		break;

	case 3: // �E���@�����n�� ---------------------------------------------------
		interval++;
		if ( numOnSensorR >= 4 ){
			// ��Ԃ�i�߂�
			stat = 4;
			interval =0; 
		}
		break;

	case 4: // �����x���� ---------------------------------------------------
		interval++;
		if (interval > 1){
			// ��Ԃ�i�߂�
			stat = 5;
			interval =0; 
		}
		break;

	case 5: // �������グ�� ---------------------------------------------------
		interval++;
		if (interval > 5){
			// ��Ԃ�i�߂�
			stat = 6;
			interval =0;
			// �������J�E���g����
			hosuu++;
		}
		break;

	case 6: // �E���x���� ---------------------------------------------------
		interval++;
		if( interval > 5 ){
			// �؂�ւ������@
			if ( changeFootX(pre) ){
				kirikae_jouken_flag = 1;
				printf("���؂�ւ�\n");
			}
			// �؂�ւ������A
			if ( changeFootY(pre) ){
				kirikae_jouken_flag = 2;
				printf("���؂�ւ�\n");
			}

			if(kirikae_jouken_flag != 0){
				// ��Ԃ�i�߂�
				stat = 7;
				interval =0; 
			}
		}
		break;

	case 7: // �������n���쒆 ---------------------------------------------------
		interval++;
		if ( numOnSensorL >=1 ){
			// ��Ԃ�i�߂�
			stat = 8;
			interval =0; 
		}
		break;

	case 8: // �����@�����n�� ---------------------------------------------------
		interval++;
		if ( numOnSensorL >= 4 ){
			// ��Ԃ�i�߂�
			stat = 9;
			interval =0; 
		}
		break;

	case 9: // �����x���� ---------------------------------------------------
		interval++;
		if (interval > 1 ){
			// ��Ԃ�i�߂�
			stat = 10;
			interval =0; 
		}
		break;

	case 10: // �E�����グ�� ---------------------------------------------------
		interval++;
		if (interval >  5 ){
			// ��Ԃ�i�߂�
			stat = 1;
			interval =0; 
			// �������J�E���g����
			hosuu++;
		}
		break;

	}

	// �x���r�̐ݒ�
	if((stat < 4) || (stat > 8)){
		setL_or_R(-1);
	}else{
		setL_or_R(1);
	}
}


// �V�r�x���r�؂�ւ����肘
bool Robot_PID::changeFootX(Robot_PID pre){
	double k = 0.12 * getGlVelX() + 0.10 * (refVel-getGlVelX()) + 0.02;
	//printf("k=%lf\n",k);

	//  �@�@  �d�S�ʒuX   > �����l�@&& �@�@�@�@�@�@�O�ɓ|��Ă���
	if( fabs(getGlPosX()) > fabs(k) && ( getSitaZ() - getpre_SitaZ() ) > 0 ){	//&& ( getSitaZ() - pre.getSitaZ() )
		return true;
	}else{
		return false;
	}
}

// �V�r�x���r�؂�ւ����肙
bool Robot_PID::changeFootY(Robot_PID pre){
	double k = 0.05 ;//0.05;

	//        �d�S�ʒuY�@ > �����l  &&          �O�ɓ|��Ă���@�@�@ &&  �d�S���x���r�Ƌt�ɐi��ł���
	if( fabs(getGlPosY()) > fabs(k) && (getSitaZ()-getpre_SitaZ()>0) && (getL_or_R()*getGlVelY()>0.0) ){	//&& (getSitaZ()-pre.getSitaZ()>0)
		return true;
	}else{
		return false;
	}
}




/*****************************************************************************************/
// ���x���璅�n�ڕW���v�Z 
/*****************************************************************************************/
// ���n�ʒu�v�Z
double Robot_PID::refXCal(){
	// ���n�ڕW���ʒu�̌v�Z
	double refX=0;
	//if( getChuukuu() ){
		// �x���r�Z���T���S�ȏ�I���̂��鎞
		//refX = 1*getGlPosX() + 0.2*getGlVelX() - 0.2*(refVel-getGlVelX());
		//refX = 1*getGlPosX() + 0.02*getGlVelX() - 0.05*(refVel-getGlVelX());
		refX = 1.0*getGlPosX() + 0.1*getGlVelX() - 0.1*(refVel-getGlVelX());
	//}else{
		// �x���r�Z���T���S�ȏ�I���łȂ���
		//refX = 1*getGlPosX() + 0.02*getGlVelX() - 0.05*(refVel-getGlVelX());
	//}
	// ������~�b�^�[
	if(refX > 30.0e-2)	{
		refX = 30.0e-2;
		//printf("refX=%lf,jogen\n",refX);
	}
	// �������~�b�^�[
	if(refX < - 20.0e-2)	{
		refX =  - 20.0e-2;
		//printf("refX=%lf,kagen\n",refX);
	}
	return refX;
}
double Robot_PID::refYCal(){
	// ���n�ڕW���ʒu�̌v�Z
	//double refY = 1*getGlPosY() + 0.026*getGlVelY() + 0.029*getL_or_R();
	double refY = 1*getGlPosY() + 0.026*getGlVelY() + 0.09*getL_or_R();
	// ���n�ڕW�������S���t���ɂȂ����ꍇ�O�ɂ���
	if(refY*getL_or_R() < 0.0){
		refY = 0.0;
	}
	return refY;
}


/*****************************************************************************************/
// �L���͂��v�Z
/*****************************************************************************************/
// �L���͌v�Z�p���\�b�h
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
	// ���x�덷�̎Z�o
	double vxErr = refVel - getGlVelX();
	double prevxErr = getGlVelX() -pre.getGlVelX();

	// �d�͂ɑ΂����
	double f_Mg = getMass() * 9.8 * cos(getSitaZ());
	//printf("mass=%lf\n",getMass());
	//printf("f_Mg=%lf\n",f_Mg);

	// x�p�x���̃v���X�␳
	double f_x_plus  = get_gain_x_plus(getSitaX()) * ( 30.0*vxErr +5.0*(vxErr-prevxErr));
	if(f_x_plus <-10){ f_x_plus  = -10; }
	//printf("get_gain_x_plus(agent.getSitaX())=%lf\n",get_gain_x_plus(agent.getSitaX()));
	//printf("f_x_plus=%lf\n",f_x_plus);

	// x�p�x���̃}�C�i�X�␳
	double f_x_minus = get_gain_x_minus(getSitaX()) * (-30.0*vxErr -5.0*(vxErr-prevxErr));
	if(f_x_minus<-10){ f_x_minus = -10; }
	//printf("f_x_minus=%lf\n",f_x_minus);

	// �����덷����̕␳
	double poszErr = refHight - getGlPosZ();
	//printf("getGlPosZ()=%lf\n",getGlPosZ());
	poszErrInteg = (fabs(poszErr)<0.05) ? (poszErrInteg + poszErr) : (poszErrInteg + sign(poszErr)*0.05) ;
	double f_z_pos = 8.0*poszErr - 20.0*getGlVelZ()+ 0.02*poszErrInteg;
	//printf("f_z_pos=%lf\n",f_z_pos);

	// �L����
	double f = ( 1.0*f_Mg + f_x_plus + f_x_minus + f_z_pos );
	if(f<0){ f=0.0; }
	//printf("f=%lf\n",f);

	return f;

}

/*****************************************************************************************/
// ���[�^�E�X���C�_�ɗ͂�ݒ肷��
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
		
		// �����p������点��
		stat=4;
		init_refjoint();
		// �v�Z���ʂ���͂�ݒ肷��
		for(int i=0;i<6;i++){
			setHingeParams(LegLJoint[i], refLegLJointAngle[i]);
			setHingeParams(LegRJoint[i], refLegRJointAngle[i]);
		}
		/*
		for(int i=0;i<6;i++){
			// �����p���̃��[�^�[�ڕW�p�x��ݒ�
			// ���r
			setHingeParams(LegLJoint[i], 0.0);
			// �E�r
			setHingeParams(LegRJoint[i], 0.0);
		}
		*/
	
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

	}else{
		
		// �v�Z���ʂ���͂�ݒ肷��
		for(int i=0;i<6;i++){
			setHingeParams(LegLJoint[i], refLegLJointAngle[i]);
			setHingeParams(LegRJoint[i], refLegRJointAngle[i]);
		}

		// ��{�`�Ƃ��Ďx���r�̑���̓t���[
		if(getChuukuu() == 0){
			if( getL_or_R() == -1 ){
				// ���r�t���[
				//printf("���r�t���[\n");
				dJointSetHingeParam(LegLJoint[4], dParamFMax, 0);
				dJointSetHingeParam(LegLJoint[5], dParamFMax, 0);
			}else{
				// �E���t���[
				//printf("�E�r�t���[\n");
				dJointSetHingeParam(LegRJoint[4], dParamFMax, 0);
				dJointSetHingeParam(LegRJoint[5], dParamFMax, 0);
			}
		}
/*
		// �V�r�̒����ڕW�����肷��
		if( (stat == 2) || (stat == 3) ){
			// 2:�E�r���n���쒆or3:�E�r�����n��
			ref_Yuukyaku_length =  40E-2; //30E-2; //50E-2;

		}else if( (stat == 7) || (stat == 8) ){
			// 7:���r���n���쒆or8:���r�����n��
			ref_Yuukyaku_length =  40E-2; //30E-2; //50E-2;

		}else if( (stat == 5)  || (stat == 10) ){
			// 5:���r�����グ��or10:�E�r�����グ��
			ref_Yuukyaku_length = -20e-2;

		}else if( (stat == 4)  || (stat == 9) ){
			// 4,9:�����x����
			ref_Yuukyaku_length = 15e-2;

		}else if( (stat == 1)  || (stat == 6) ){
			// 1:���r�x����or6:�E�r�x����
			ref_Yuukyaku_length = -5e-2;//-5e-2; // -10e-2; //-30E-2;

		}else{
			// ���蓾�Ȃ������Ȃ̂ŃG���[�\��
			printf("Motor_set.h  ERROR(stat=%d is an impossible number.) ",stat);
			getchar();
			exit(0);
		}
*/
		
		// �v�Z���ʂ��X���C�_�[�ڕW�����ɐݒ� -------------------------------------------------
		/*
		if(getL_or_R() == -1){
			// ���r
			dJointAddSliderForce(LegLJoint[3], f);			// �x���r�͐L���͂ŋ��߂�f���g��
			// �E�r
			f_Yuukyaku = getSliderForce(ref_Yuukyaku_length,LR_l_3,LR_l_3_vel);
			dJointAddSliderForce(LegRJoint[3], f_Yuukyaku);		// �V�r�͏�Ō��߂����ʂ��g��

		}else{
			// ���r
			f_Yuukyaku = getSliderForce(ref_Yuukyaku_length,LL_l_3,LL_l_3_vel);
			dJointAddSliderForce(LegLJoint[3], f_Yuukyaku);		// �V�r�͏�Ō��߂����ʂ��g��
			// �E�r
			dJointAddSliderForce(LegRJoint[3], f);			// �x���r�͐L���͂ŋ��߂�f���g��

		}
		*/
	}
}

void Robot_PID::Ashisaki(){
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
