/*****************************************************************************************/
//                                     ���{�b�g�N���X	
/*****************************************************************************************/
// ��2013/6/4 pos=dBodyGetPosition(dbodyID);��getPartsPosX(dBodyID);�œ���
// ���`��֌W�̊֐����ȊO�͂�����œ��ꂷ��悤�ɂ��邱��
//
/*****************************************************************************************/

class Robot{
private:
	//---------------------------------------------------------------------------------//
	// ���{�b�g�̃v���C�x�[�g�����o�ϐ��̐錾
	//---------------------------------------------------------------------------------//
	dVector3 absGl;	// ���{�b�g�̏d�S�ʒu�i��΍��W�n�ł̏d�S�ʒu�j
	dVector3 GlPos;	// ���{�b�g�̏d�S�ʒu�i�x���r�ڒn�ʒu�����_�j
	dVector3 GlVel;	// ���{�b�g�̏d�S���x
	dVector3 GlAcc;	// ���{�b�g�̏d�S�����x
	dReal mass;		// ���{�b�g�̑S���ʁi�d�S�v�Z���Ɏ擾�j
	int L_or_R;		// ���{�b�g�̎x���r�i��:-1, �E:1�j
public:
	int stat;
	int hosuu;

	//���n�ڕW�ʒu�蓮����p
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
		init();		///�ϐ�������
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
	//�W���C���g�p�x�̏����ݒ�
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
	//�d�S�֌W�ϐ��錾
	dReal Gl[4];
	float zero_vec[4];

	const dReal *w_R_12;
	dReal w_R_44[4][4];
	dReal inverse_w_R_44[4][4];

	float work_distance[4];
	float work_chaku_moku[4];
	float work_a_distance[4];

	//�d�S�֌W�ϐ�������
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
	// ���{�b�g�̃p�u���b�N�����o�ϐ��̐錾
	//---------------------------------------------------------------------------------//
	MyBoxLink Body[2];		// �{�́i���E���j
	MyBoxLink LegL[8];		// ���r�i���A�F�A�G�A���j
	MyBoxLink LegR[8];		// �E�r�i���A�F�A�G�A���j
	MyBoxLink SensorL[6];		// �������Z���T�i���O�A�E�O�A�����A�E���A����A�E��j
	MyBoxLink SensorR[6];		// �E�����Z���T�i�E�O�A���O�A�E���A�����A�E��A����j
	MyBoxLink SpringL[6];		// �������Z���T�̃o�l����
	MyBoxLink SpringR[6];		// �E�����Z���T�̃o�l����

	dJointID BodyJoint[1];		// Body[0]��Body[1]�̃W���C���g
	dJointID LegLJoint[8];		// ���r�̃W���C���g(3��Slider�A6�E7�Ԃ�Fixed)
	dJointID LegRJoint[8];		// �E�r�̃W���C���g(3��Slider�A6�E7�Ԃ�Fixed)
	dJointID joint;				// �󒆗p
	dJointID SenLJoint[6];		// �������Z���T�W���C���g
	dJointID SenRJoint[6];		// �E�����Z���T�W���C���g
	dJointID SrgLJoint[6];		// �������Z���T�o�l�W���C���g
	dJointID SrgRJoint[6];		// �E�����Z���T�o�l�W���C���g


	//---------------------------------------------------------------------------------//
	// �Q�b�^�[�E�Z�b�^�[
	//---------------------------------------------------------------------------------//
	// �x���r�̃Z�b�^�[�ƃQ�b�^�[�i��:-1, �E:1�j
	void setL_or_R(int l_or_r){ L_or_R = l_or_r; }
	int getL_or_R(){ return L_or_R; }

	//sita�̃Z�b�^�[
	void setSitaX(dReal sitaX){ Sita[0] = sitaX; }
	void setSitaY(dReal sitaY){ Sita[1] = sitaY; }
	void setSitaZ(dReal sitaZ){ Sita[2] = sitaZ; }

	/*
	�x���r�����_�Ƃ����d�S�̃Q�b�^�[
	calcGlPos��GlPos��mass���v�Z
	calcGlVel��GlVel���v�Z
	calcGlAcc��GlAcc���v�Z
	*/
	double getMass(){ return mass;}	// ���{�b�g�̑S���ʂ̃Q�b�^�[
	double getGlPosX(){ return GlPos[0]; }	// �d�S�ʒu���̃Q�b�^�[
	double getGlPosY(){ return GlPos[1]; }	// �d�S�ʒu���̃Q�b�^�[	
	double getGlPosZ(){ return GlPos[2]; }	// �d�S�ʒu���̃Q�b�^�[
	double getSitaX(){ return Sita[0]; }	//sita�̒l�ς������Ƃ��p
	double getSitaY(){ return Sita[1]; }
	double getSitaZ(){ return Sita[2]; }
	double getpre_SitaX(){ return pre_Sita[0]; }	//sita�O��̒l�ۑ��p
	double getpre_SitaY(){ return pre_Sita[1]; }
	double getpre_SitaZ(){ return pre_Sita[2]; }
	double getGlVelX(){ return GlVel[0]; }	// �d�S���x���̃Q�b�^�[
	double getGlVelY(){ return GlVel[1]; }	// �d�S���x���̃Q�b�^�[	
	double getGlVelZ(){ return GlVel[2]; }	// �d�S���x���̃Q�b�^�[
	double getGlAccX(){ return GlAcc[0]; }	// �d�S�����x���̃Q�b�^�[
	double getGlAccY(){ return GlAcc[1]; }	// �d�S�����x���̃Q�b�^�[	
	double getGlAccZ(){ return GlAcc[2]; }	// �d�S�����x���̃Q�b�^�[

	bool getChuukuu(){
		if(getL_or_R() == -1){
			if( SensorLCheck() ==4 ){
				// ���r�x���ō��r�Z���T�����ׂăI�� -> ����t���O�I��
				return false;//true;	//0��on
			}else{
				return true;//false;	//1��off
			}
		}else{
			if( SensorRCheck() ==4 ){
				// �E�r�x���ŉE�r�Z���T�����ׂăI�� -> ����t���O�I��
				return false;//true;	
			}else{
				return true;//false;
			}
		}
		printf("error:getChuukuu\n");
		getchar();
		exit(0);
	}

	
	// ���{�b�g�̐������\�b�h
	void createRobot(dReal x0,dReal y0, dReal z0);

	// ���{�b�g�̕`�惁�\�b�h
	void drawRefPos();
	void drawRobot();
	void drawPendulum();

	// ���{�b�g�̔j�󃁃\�b�h
	void destroyRobot();


	// sita�v�Z
	void calcSita(){
		Sita[0] = atan2(sqrt(pow(GlPos[1],2) + pow(GlPos[2],2)),GlPos[0]);
		Sita[1] = atan2(sqrt(pow(GlPos[2],2) + pow(GlPos[0],2)),GlPos[1]);
		Sita[2] = atan2(sqrt(pow(GlPos[0],2) + pow(GlPos[1],2)),GlPos[2]);
	}

	//---------------------------------------------------------------------------------//
	// �d�S�ʒu�E���x�E�����x�̌v�Z���\�b�h
	//---------------------------------------------------------------------------------//
	// ��΍��W�n�̏d�S�ʒu�̐ݒ�
	void setAbsGl(dReal glx, dReal gly, dReal glz){
		absGl[0] = glx;
		absGl[1] = gly;
		absGl[2] = glz;
	}
	// �p�[�c��ǉ����ďd�S�v�Z�iabsGlGet�ŗ��p�j
	void calcAbsGlAddParts(dBodyID body, const dReal addMass, const dReal sumMass){
		absGl[0] = absGl[0]*(sumMass/(addMass+sumMass)) + getPartsPosX(body)*(addMass/(addMass+sumMass));
		absGl[1] = absGl[1]*(sumMass/(addMass+sumMass)) + getPartsPosY(body)*(addMass/(addMass+sumMass));
		absGl[2] = absGl[2]*(sumMass/(addMass+sumMass)) + getPartsPosZ(body)*(addMass/(addMass+sumMass));
	}

	// ��΍��W�n�d�S�v�Z�iODE��API���p�A��΍��W�n�j
	void calcAbsGl(){
		//const dReal *pos;		// �p�[�c�ʒu���ꎞ�ۊǗp
		dReal sumMass=0;		// �������񂾃p�[�c�̎��ʍ��v

		// ���̂̍��W�ݒ�i��ڂȂ̂ŁA���{�b�g�d�S���p�[�c�̏d�S�j
		//pos = dBodyGetPosition(Body[0].body);			// �p�[�c�ʒu���擾
		setAbsGl(getPartsPosX(Body[0].body),getPartsPosY(Body[0].body),getPartsPosZ(Body[0].body));				// �d�S�ʒu��ۑ�
		sumMass = Body[0].mass;						// �p�[�c���ʂ�ݒ�
		
		// ���̂̏d�S�������݁i��ڈȍ~�͌��݂̏d�S�̑������ތ`�ōČv�Z���Ă����j
		//pos = dBodyGetPosition(Body[1].body);			// �p�[�c�ʒu���擾
		calcAbsGlAddParts(Body[1].body, Body[1].mass, sumMass);	// �p�[�c��ǉ����ďd�S�v�Z
		sumMass += Body[1].mass;					// �p�[�c���ʂ𑫂�����

		// �r�̃p�[�c�d�S��������
		for(int i=0;i<8;i++){
			// ���r���W��������
			//pos = dBodyGetPosition(LegL[i].body);		// �p�[�c�ʒu���擾
			calcAbsGlAddParts(LegL[i].body, LegL[i].mass, sumMass);	// �p�[�c��ǉ����ďd�S�v�Z
			sumMass += LegL[i].mass;				// �p�[�c���ʂ𑫂�����
			// �E�r���W��������
			//pos = dBodyGetPosition(LegR[i].body);		// �p�[�c�ʒu���擾
			calcAbsGlAddParts(LegR[i].body, LegR[i].mass, sumMass);	// �p�[�c��ǉ����ďd�S�v�Z
			sumMass += LegR[i].mass;				// �p�[�c���ʂ𑫂�����
			
		}

		// ���{�b�g�̑S����
		mass = sumMass;
	}

	// ���΍��W�n�d�S�v�Z�iODE��API���p�A�x���r�̑����ڒn�ʒu�����_�Ƃ����ꍇ�j
	void calcGlPos(){
		calcAbsGl();			// ��΍��W�n�ł̏d�S�ʒu�̎擾
		//const dReal *pos;		// �p�[�c�ʒu���ꎞ�ۊǗp
		if(L_or_R == -1){
			//pos = dBodyGetPosition(LegL[7].body);	// ���_���W�ʒu�̎擾
			GlPos[0] = absGl[0] - getPartsPosX(LegL[7].body);
			GlPos[1] = absGl[1] - getPartsPosY(LegL[7].body);
			GlPos[2] = absGl[2] - getPartsPosZ(LegL[7].body);
		}else{
			//pos = dBodyGetPosition(LegR[7].body);	// ���_���W�ʒu�̎擾
			//getPartsPos(LegR[7].body, pos);
			GlPos[0] = absGl[0] - getPartsPosX(LegR[7].body);
			GlPos[1] = absGl[1] - getPartsPosY(LegR[7].body);
			GlPos[2] = absGl[2] - getPartsPosZ(LegR[7].body);
		}


	}

	// �d�S���x�̌v�Z
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

	// �d�S�����x�̌v�Z
	void calcGlAcc(Robot pre0, double steptime){
		GlAcc[0] = (getGlVelX()-pre0.getGlVelX())/steptime;
		GlAcc[1] = (getGlVelY()-pre0.getGlVelY())/steptime;
		GlAcc[2] = (getGlVelZ()-pre0.getGlVelZ())/steptime;
	}


	//---------------------------------------------------------------------------------//
	// �Z���T��Ԏ擾
	//---------------------------------------------------------------------------------//
	// �����Z���T���I�����Ă�����̎擾
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
	// �E���Z���T���I�����Ă�����̎擾
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
	// �r�֐ߊp�x�v�Z
	//---------------------------------------------------------------------------------//
	// ���W���C���g�̖ڕW�p�x�E�ڕW����
	void initRefJointAngle(){
		for(int i=0;i<6;i++){
			refLegLJointAngle[i] = 0.0;
			refLegRJointAngle[i] = 0.0;
		}
	}

	// �V�r�r�t���������_�Ƃ������ݗV�r����ʒu�̎擾	�i���݂͗V�r��joint[0]�̍��W�����_�Ƃ��Ă���B�j
	void getSwingLegStr2EndPos(dReal r[4], const dReal ref[4]){
		//const dReal *pos;	// �p�[�c�ʒu�擾�p
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

	// �V�r�t����(LegX[0])�̎p���̎擾
	void getSwingLegSrtRel(dReal r[][4]){
		const dReal *rel;
		if(getL_or_R()==-1){
			rel = dBodyGetRotation(LegR[1].body);		//����LegR[0]��LegR[1]�ɕς����B���������͂킩��Ȃ� 
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
	//���ǉ��֐�
	
	// �x���r�t����(LegX[1])�̎p���̎擾
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
		r[2] += 8.0E-2;	//�����܂ł̒���
	}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


	////////////////////////////////////////////////////////////////////////////////////////////
	// �W���C���g�̖ڕW�p�x�̌v�Z
	void calcRefJoint(Robot pre, dVector3 refSwingLegPos){

		// �V�r�ڕW����(LegX[7])�ʒu�̌v�Z(�V�r�t����(LegX[0])�����_)
		dReal swingLegSrt2RefPos[4];
		getSwingLegStr2EndPos(swingLegSrt2RefPos, refSwingLegPos);
		
		// �V�r�t����(LegX[0])�̎p���̎擾
		dReal swingLegSrtRel[4][4];
		getSwingLegSrtRel(swingLegSrtRel);

		// �V�r�t����(LegX[0])�̎p���̋t�s��̎擾
		dReal inv_swingLegSrtRel[4][4];
		inverse(inv_swingLegSrtRel, swingLegSrtRel);

		// ��Ōv�Z�����s��ƃx�N�g�����|�����킹��
		dReal sigma[4];
		ftrvadd( sigma, swingLegSrt2RefPos, inv_swingLegSrtRel);

		//for(int i=0;i<3;i++) printf("sigma[%d]=%lf\n",i,sigma[i]);

	//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
	//�ȉ����n�搶�̃R�s�y�Ƃ肠����SHR��

		int flag;
		//�r�̒���(joint�Ԃ̒���)	//���E�����Ȃ̂ō����̂ݐ錾
		float Lxl_2 = 0.0;
		float Lxl_3 = 25.0E-2;
		float Lxl_4 = 25.0E-2;
		float Lxl_5 = 0.0;
		float Lxl_6 = (7.0 + 1.0) * 0.01;

		//�p�x�v�Z�Ɏg����
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
		//  (step8) �V�r�̊Ԑڊp�x�����߂�D
		// =========================================================================

		// �����x���̏ꍇ
		if( getL_or_R() == -1){
			
			// �E�r�i�V�r�j�̌v�Z
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

				// �G���L�т��邩�`�F�b�N�i�ڕW�ʒu�ɋr���L�΂��邩�ǂ����H�j
				if( l_2to4 < (Lxl_3 + Lxl_4) ){

					if( ! ( (l_2to4 + Lxl_3) <= Lxl_4 ) ){
						refLegRJointAngle[2] = - ( alpha_0 + alpha_1 ); // �O�̕������̓��[�^�[�̉�]�������炭��D
						refLegRJointAngle[3] = M_PI - alpha_2;
					}else{
						refLegRJointAngle[2] = getHingeAngle(LegRJoint[2]);
						refLegRJointAngle[3] = getHingeAngle(LegRJoint[3]);
					}
				}else{	//�L�т���Ȃ��ꍇ
					refLegRJointAngle[2] = - alpha_0;
					refLegRJointAngle[3] = 0;
				}
			}else{
				printf("stop -- Yuukyaku_shijikyaku.h");
				getchar();
			}

			if( (stat == 9) ){	//�����x���ɂȂ����Ƃ��̏���
				if( (stat == 9) && (pre.stat == 8) ){
					buf_LRr[1] = getHingeAngle(LegRJoint[1]);
					buf_LRr[2] = getHingeAngle(LegRJoint[2]);
					buf_LRr[3] = getHingeAngle(LegRJoint[3]);
				}
				refLegRJointAngle[1] = buf_LRr[1];
				refLegRJointAngle[2] = buf_LRr[2];
				refLegRJointAngle[3] = buf_LRr[3];
			}

			//	�V�r�̋r�グ�������_�u���Ă���̂ŁC��������R�����g������D���悭�킩���ꉞ�R�����g�A�E�g���Ȃ��ł�����
			//�����R�����g�A�E�g����ƕ����Ȃ��Ȃ�B�s����
			
			if( (stat == 10) ){	//�V�r�����グ��
				refLegRJointAngle[1] = buf_LRr[1];
				refLegRJointAngle[2] = buf_LRr[2] - deg2rad(40);
				refLegRJointAngle[3] = buf_LRr[3] + deg2rad(60);
			}
			

			refLegRJointAngle[0] = 0.0;	//�����]���p�W���C���g�@���ݕs�g�p

			buf_LLr[1] = getHingeAngle(LegLJoint[1]);
			buf_LLr[5] = getHingeAngle(LegLJoint[5]);
			buf_LRr[1] = getHingeAngle(LegRJoint[1]);

			refLegRJointAngle[4] = - (refLegRJointAngle[2] + refLegRJointAngle[3]);	//�n�ʂɑ΂��Đ����ɂȂ�悤�ɂ��邽�߂��ȁH
			//refLegRJointAngle[5] =  (buf_LLr[5] + buf_LLr[1]) - refLegRJointAngle[1] ;
			//refLegRJointAngle[5] =  (buf_LLr[5] + buf_LLr[1]) - buf_LRr[1] ;	//�ǂ������������̂��H
			refLegRJointAngle[5] = - refLegRJointAngle[1];	//�����������悤�ȋC�����邪�E�E�E


		// �E���x���̏ꍇ�̍����̊p�x�v�Z
		}else{
			// ���r�i�V�r�j�̌v�Z
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

				// �G���L�т��邩�`�F�b�N�i�ڕW�ʒu�ɋr���L�΂��邩�ǂ����H�j
				if( l_2to4 < (Lxl_3 + Lxl_4) ){

					if( ! ( (l_2to4 + Lxl_3) <= Lxl_4 ) ){
						refLegLJointAngle[2] = - ( alpha_0 + alpha_1 ); // �O�̕������̓��[�^�[�̉�]�������炭��D
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

			if( (stat == 4) ){	//�����x���ɂȂ����Ƃ��̏���
				if( (stat == 4) && (pre.stat == 3) ){
					buf_LLr[1] = getHingeAngle(LegLJoint[1]);
					buf_LLr[2] = getHingeAngle(LegLJoint[2]);
					buf_LLr[3] = getHingeAngle(LegLJoint[3]);
				}
				refLegLJointAngle[1] = buf_LLr[1];
				refLegLJointAngle[2] = buf_LLr[2];
				refLegLJointAngle[3] = buf_LLr[3];
			}
			//	�V�r�̋r�グ�������_�u���Ă���̂ŁC��������R�����g������D
			
			if( (stat == 5) ){	//�V�r�����グ��
				refLegLJointAngle[1] = buf_LLr[1];
				refLegLJointAngle[2] = buf_LLr[2] - deg2rad(40);
				refLegLJointAngle[3] = buf_LLr[3] + deg2rad(60);
			}
			

			refLegLJointAngle[0] = 0.0;

			buf_LRr[1] = getHingeAngle(LegRJoint[1]);
			buf_LRr[5] = getHingeAngle(LegRJoint[5]);

			refLegLJointAngle[4] = - (refLegLJointAngle[2] + refLegLJointAngle[3]);
			//refLegLJointAngle[5] =  (buf_LRr[5] + buf_LRr[1]) - refLegLJointAngle[1] ;	//������������������
			refLegLJointAngle[5] = - refLegLJointAngle[1];

		} //�E���x���̏ꍇ��else���̕�����


		// =========================================================================
		//  (step9) �x���r�̊֐ߊp�x�����߂�D
		// ========================================================================
		
		//�ȉ��̎x���r�p�x�v�Z�͑��n�搶�̃R�s�ySHR��

		double ref_z = 55.0E-2;//35.0E-2;		//�ڕW����
		double ref_Lx_1[3];
		double Lx_1[3];		
		double ref_Rx_1[3];
		double Rx_1[3];

		// �x���r�ڕW����(LegX[0])�ʒu�̌v�Z(���������_)
		dReal supportLegSrt2RefPos[4];
		getsupportLegStr2EndPos(supportLegSrt2RefPos);

		//for(int i=0;i<3;i++) printf("supportLegSrt2RefPos[%d]=%lf\n",i,supportLegSrt2RefPos[i]);
		
		// �x���r�t����(LegX[0])�̎p���̎擾
		dReal supportLegSrtRel[4][4];
		getsupportLegSrtRel(supportLegSrtRel);

		// �x���r�t����(LegX[0])�̎p���̋t�s��̎擾
		dReal inv_supportLegSrtRel[4][4];
		inverse(inv_supportLegSrtRel, supportLegSrtRel);

		// ��Ōv�Z�����s��ƃx�N�g�����|�����킹��
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

		// �����x���̏ꍇ
		if( getL_or_R() ==-1 ){ 

			// Gl�̍����ŕG�̐܊p���v�Z���Ă��邪�A���̍��������ƂȂ�悤�ɏC���̗v�����邩������Ȃ��D
			//ref_Lx_1[0] =fabs(Lx_1[0]) * (ref_z - Lxl_6) / ( GlPos[2] - Lxl_6 );
			//ref_Lx_1[1] =fabs(Lx_1[1]) * (ref_z - Lxl_6) / ( GlPos[2] - Lxl_6 );
			
			//20130118 ��̃R�����g�ɏ]���ďC���D�i�����͒�����̃X�^�[�g���ɂ����Ȃ�G��L�΂����Ƃ��錻�ۂ��o�����߁j
			ref_Lx_1[0] =fabs(Lx_1[0]) * (ref_z - Lxl_6) / ( Lx_1[2] - Lxl_6 );
			ref_Lx_1[1] =fabs(Lx_1[1]) * (ref_z - Lxl_6) / ( Lx_1[2] - Lxl_6 );
			//printf("ref_Lx_1[0]=%lf\n",ref_Lx_1[0]);
			//printf("ref_Lx_1[1]=%lf\n",ref_Lx_1[1]);

			l_t1 = sqrtf( (ref_z - Lxl_6)*(ref_z - Lxl_6) + (ref_Lx_1[1]) * (ref_Lx_1[1]) );

			l_sha = sqrt( (ref_Lx_1[0])*(ref_Lx_1[0]) + (l_t1 - Lxl_2 - Lxl_5) * (l_t1 - Lxl_2 - Lxl_5));

			alpha_0 = - sign( Lx_1[0] ) * atanf( (ref_Lx_1[0]) / (l_t1 - Lxl_2 - Lxl_5) );
			//printf("alpha_0=%lf\n",alpha_0);

			// �G���L�т��邩�`�F�b�N
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

			//  ���󎞂́C���̂܂܂̒l���w�肷��D
			if( getChuukuu() == 1 ){
				printf("�x���r���̂܂܂̒l�ێ�\n");
				for(int i=0;i<6;i++){
					refLegLJointAngle[i] = getHingeAngle(LegLJoint[i]);
				}
			}


		}else{   // �E���x���̏ꍇ�̉E���̊p�x�v�Z

			// Gl�̍����ŕG�̐܊p���v�Z���Ă��邪�A���̍��������ƂȂ�悤�ɏC���̗v�����邩������Ȃ��D

			//ref_Rx_1[0] =fabs(Rx_1[0]) * (ref_z - Rxl_6) / ( Gl[2] - Rxl_6 );
			//ref_Rx_1[1] =fabs(Rx_1[1]) * (ref_z - Rxl_6) / ( Gl[2] - Rxl_6 );

			//20130118 ��̃R�����g�ɏ]���ďC���D�i�����͒�����̃X�^�[�g���ɂ����Ȃ�G��L�΂����Ƃ��錻�ۂ��o�����߁j
			ref_Rx_1[0] =fabs(Rx_1[0]) * (ref_z - Rxl_6) / ( Rx_1[2] - Rxl_6 );
			ref_Rx_1[1] =fabs(Rx_1[1]) * (ref_z - Rxl_6) / ( Rx_1[2] - Rxl_6 );

			l_t1 = sqrtf( (ref_z - Rxl_6)*(ref_z - Rxl_6) + (ref_Rx_1[1]) * (ref_Rx_1[1]));

			l_sha = sqrt( (ref_Rx_1[0])*(ref_Rx_1[0]) + (l_t1 - Rxl_2 - Rxl_5) * (l_t1 - Rxl_2 - Rxl_5));

			alpha_0 = - sign( Rx_1[0] ) * atanf( (ref_Rx_1[0]) / (l_t1 - Rxl_2 - Rxl_5) );

			// �G���L�т��邩�`�F�b�N
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

			//  ���󎞂́C���̂܂܂̒l���w�肷��D
			if( getChuukuu() == 1 ){
				printf("�x���r���̂܂܂̒l�ێ�\n");
				for(int i=0;i<6;i++){
					refLegRJointAngle[i] = getHingeAngle(LegRJoint[i]);
				}
			}
		}

	}



};
