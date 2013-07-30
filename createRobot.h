/*****************************************************************************************/
// ���{�b�g�̐������\�b�h
/*****************************************************************************************/
void Robot::createRobot(dReal x0,dReal y0, dReal z0)
{
	//dReal x0=0.0, y0=0.0, z0=80.0E-2;	// ���{�b�g�̏����ʒu���S�_

	// ���̂̐��� -----------------------------------------------------------------//
	// ���̂O�i��ԏ�̕����j
	Body[0].setMass(15.0E-3);						// ���ʂ̐ݒ�
	Body[0].setPos(-2.0E-2, 0.0E-2, 11.0E-2);			// �ʒu�̐ݒ�
	Body[0].setSize(16.0E-2, 25.6E-2, 10.0E-2);		// �T�C�Y�̐ݒ�
	Body[0].setAngle(1, 0, 0, 0.0*M_PI/180.0);		// �X���̐ݒ�
	createBox(Body[0], x0, y0, z0);				// �p�[�c����

	// ���̂P
	Body[1].setMass(100.0E-2);						// ���ʂ̐ݒ�
	Body[1].setPos(-2.0E-2, 0.0E-2, 1.0E-2);			// �ʒu�̐ݒ�
	Body[1].setSize(13.0E-2, 25.6E-2, 10.0E-2);			// �T�C�Y�̐ݒ�
	Body[1].setAngle(1, 0, 0, 0.0*M_PI/180.0);		// �X���̐ݒ�
	createBox(Body[1], x0, y0, z0);				// �p�[�c����

	// ���̂P�𓷑̂O�ɃW���C���g
	BodyJoint[0] = dJointCreateHinge(world,0);				// �W���C���g����
	dJointAttach(BodyJoint[0], Body[1].body, Body[0].body);		// �W���C���g����p�[�c
	dJointSetHingeAnchor(BodyJoint[0], x0, y0, z0+2.5E-2 );		// �A���J�[�̈ʒu
	dJointSetHingeAxis(BodyJoint[0], 0, 1, 0);				// ������+x
	dJointSetHingeParam(BodyJoint[0], dParamFMax, 700);			// �g���N���
	dJointSetHingeParam(BodyJoint[0], dParamLoStop, -90.0*M_PI/180.0);	// �p�x����
	dJointSetHingeParam(BodyJoint[0], dParamHiStop,  90.0*M_PI/180.0);	// �p�x���
	
	// �r�̐��� -------------------------------------------------------------------//
	// ����[kg]
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
	// �r�̃T�C�Y�ݒ�
	dReal LegSize[8][3] =
	{
		{  9.0E-2, 4.5E-2, 3.0E-2 },	// LegX[0]��xyz����
		{ 15.1E-2, 7.7E-2, 9.4E-2 },	// LegX[1]��xyz����
		{ 11.7E-2, 3.2E-2, 9.3E-2 },	// LegX[2]��xyz����
		{ 5.3E-2, 11.1E-2, 28.8E-2 },	// LegX[3]��xyz����
		{ 5.3E-2, 11.1E-2, 28.8E-2 },	// LegX[4]��xyz����
		{ 11.5E-2, 7.6E-2, 7.1E-2 },	// LegX[5]��xyz����
		{ 14.8E-2, 5.0E-2, 8.8E-2 },	// LegX[6]��xyz����
		{ 25.0E-2, 14.0E-2, 1.0E-2 }	// LegX[7]��xyz����
	};
	// �r�̈ʒu���W�ݒ�i���r���W�A�E�r��y���W���]�j
	dReal LegPos[8][3] =
	{
		{ - 2.25e-2, 9.0e-2,  - 1.5e-2 },	// LegX[0]��x,y,z���W
		{ - 3.05e-2, 9.0e-2,  - 9.7e-2 },	// LegX[1]��x,y,z���W
		{ - 3.25e-2, 9.55e-2, - 14.95e-2 },	// LegX[2]��x,y,z���W	
		{    0.0E-2, 9.55e-2, - 25.0E-2 },	// LegX[3]��x,y,z���W
		{    0.0E-2, 9.55e-2, - 50.0e-2 },	// LegX[4]��x,y,z���W
		{    -3.2E-2, 9.55e-2, - 61.55e-2 },	// LegX[5]��x,y,z���W
		{    -3.2E-2, 9.0e-2,  - 65.0e-2 },	// LegX[6]��x,y,z���W
		{    0.0E-2, 11.2e-2, - 69.9e-2 }	// LegX[7]��x,y,z���W
	};
	// �W���C���g�̍��W�ݒ�i���r���W�A�E�r��y���W���]�j
	dReal LegAnchor[6][3] =
	{
		{ 0.0E-2, 9.0e-2, -12.5e-2},		// LegX[0]��LegX[1](�q���W)
		{ 0.0E-2, 9.0e-2, -12.5e-2},		// LegX[1]��LegX[2](�q���W)
		{ 0.0E-2, 9.0e-2, -12.5e-2},	// LegX[2]��LegX[3](�q���W)
		{ 0.0E-2, 9.0e-2, -37.5e-2},	// LegX[3]��LegX[4](�q���W)
		{ 0.0E-2, 9.0e-2, -62.5e-2},	// LegX[4]��LegX[5](�q���W)
		{ 0.0E-2, 9.0e-2, -62.5e-2}		// LegX[5]��LegX[6](�q���W)
	};
	// �q���W�W���C���g������
	dReal LegAxis[6][3] = 
	{	
		{  0,  0, -1 },
		{ -1,  0,  0 },
		{  0, -1,  0 },
		{  0, -1,  0 },
		{  0, -1,  0 },
		{ -1,  0,  0 }
	};

	// �r����
	for(int i=0;i<8;i++){
		// ���r����
		LegL[i].setMass(LegMass[i]);									// ���ʂ̐ݒ�
		LegL[i].setSize(LegSize[i][0], LegSize[i][1], LegSize[i][2]);	// �T�C�Y�̐ݒ�
		LegL[i].setPos(LegPos[i][0], LegPos[i][1], LegPos[i][2]);		// �ʒu�̐ݒ�
		LegL[i].setAngle(1, 0, 0, 0*M_PI/180.0);						// �X���̐ݒ�
		createBox(LegL[i], x0, y0, z0);									// �p�[�c����
		// �E�r����
		LegR[i].setMass(LegMass[i]);									// ���ʂ̐ݒ�
		LegR[i].setSize(LegSize[i][0], LegSize[i][1], LegSize[i][2]);	// �T�C�Y�̐ݒ�
		LegR[i].setPos(LegPos[i][0], -LegPos[i][1], LegPos[i][2]);		// �ʒu�̐ݒ�
		LegR[i].setAngle(1, 0, 0, 0*M_PI/180.0);						// �X���̐ݒ�
		createBox(LegR[i], x0, y0, z0);									// �p�[�c����
	}

	// �W���C���g�̐���
	for(int i=0;i<6;i++){
			// ���r�q���W�W���C���g�̐ݒ�
			LegLJoint[i] = dJointCreateHinge(world, 0);
			dJointAttach(LegLJoint[i], LegL[i].body,LegL[i+1].body);
			dJointSetHingeAnchor(LegLJoint[i], x0+LegAnchor[i][0], y0+LegAnchor[i][1], z0+LegAnchor[i][2]);
			dJointSetHingeAxis(LegLJoint[i], LegAxis[i][0], LegAxis[i][1], LegAxis[i][2]);
			//dJointSetHingeParam(LegLJoint[i], dParamBounce, 0);
			dJointSetHingeParam(LegLJoint[i], dParamLoStop, -1 * M_PI);
			dJointSetHingeParam(LegLJoint[i], dParamHiStop, 1 * M_PI);
			// �E�r�q���W�W���C���g�̐ݒ�
			LegRJoint[i] = dJointCreateHinge(world, 0);
			dJointAttach(LegRJoint[i], LegR[i].body,LegR[i+1].body);
			dJointSetHingeAnchor(LegRJoint[i], x0+LegAnchor[i][0], y0-LegAnchor[i][1], z0+LegAnchor[i][2]);
			dJointSetHingeAxis(LegRJoint[i], LegAxis[i][0], LegAxis[i][1], LegAxis[i][2]);
			//dJointSetHingeParam(LegRJoint[i], dParamBounce, 0);
			dJointSetHingeParam(LegRJoint[i], dParamLoStop, -1 * M_PI);
			dJointSetHingeParam(LegRJoint[i], dParamHiStop, 1 * M_PI);

	}

	LegLJoint[6] = dJointCreateFixed(world, 0);				// �Œ�֐�
	dJointAttach(LegLJoint[6], LegL[6].body, LegL[7].body);	// �Œ�֐߂̎�t��
	dJointSetFixed(LegLJoint[6]);							// �Œ�֐߂̐ݒ�

	LegLJoint[7] = dJointCreateFixed(world, 0);				// �Œ�֐�
	dJointAttach(LegLJoint[7], Body[1].body, LegL[0].body);	// �Œ�֐߂̎�t��
	dJointSetFixed(LegLJoint[7]);							// �Œ�֐߂̐ݒ�

	LegRJoint[6] = dJointCreateFixed(world, 0);				// �Œ�֐�
	dJointAttach(LegRJoint[6], LegR[6].body, LegR[7].body);	// �Œ�֐߂̎�t��
	dJointSetFixed(LegRJoint[6]);							// �Œ�֐߂̐ݒ�

	LegRJoint[7] = dJointCreateFixed(world, 0);				// �Œ�֐�
	dJointAttach(LegRJoint[7], Body[1].body, LegR[0].body);	// �Œ�֐߂̎�t��
	dJointSetFixed(LegRJoint[7]);							// �Œ�֐߂̐ݒ�

	//**************�ǉ��A�󒆂Ŏ~�߂邽�߂̂���***************************//
	joint = dJointCreateFixed(world, 0);				// �Œ�֐�
	dJointAttach(joint, Body[0].body, NULL);	// �Œ�֐߂̎�t��
	dJointSetFixed(joint);
	//*********************************************************************//


	// �Z���T�̐��� -----------------------------------------------------------------//
	// �e�p�[�c�̃T�C�Y
	dReal SensorSize[4][3] = {
		{ 4.0E-2, 1.0E-2, 2.0E-2 },	// ���O�Z���T�[�̃T�C�Y
		{ 4.0E-2, 1.0E-2, 2.0E-2 }, // �E�O�Z���T�[�̃T�C�Y
		{ 4.0E-2, 1.0E-2, 2.0E-2 }, // ����Z���T�[�̃T�C�Y
		{ 4.0E-2, 1.0E-2, 2.0E-2 }  // �E��Z���T�[�̃T�C�Y
	};

	// �e�p�[�c�̏����ʒu
	dReal SensorPos[4][3] = {
		{ 10.5E-2 , 11.2e-2 +7.5e-2, - 69.9e-2 -0.5e-2 },
		{ 10.5E-2 , 11.2e-2 -7.5e-2, - 69.9e-2 -0.5e-2 },
		{ -10.5E-2 , 11.2e-2 -7.5e-2, - 69.9e-2 -0.5e-2 },
		{ -10.5E-2 , 11.2e-2 +7.5e-2, - 69.9e-2 -0.5e-2 }
	};

	// �e�p�[�c�̎���
	dReal SensorMass[4] = {1E-3, 1E-3, 1E-3, 1E-3};

	// �Z���T����
	for(int i=0;i<4;i++){
		// �����Z���T����
		SensorL[i].setMass(SensorMass[i]);							// ���ʂ̐ݒ�
		SensorL[i].setSize(SensorSize[i][0], SensorSize[i][1], SensorSize[i][2]);	// �T�C�Y�̐ݒ�
		SensorL[i].setPos(SensorPos[i][0], SensorPos[i][1], SensorPos[i][2]);		// �ʒu�̐ݒ�
		SensorL[i].setAngle(1, 0, 0, 0*M_PI/180.0);					// �X���̐ݒ�
		createBox(SensorL[i], x0, y0, z0);							// �p�[�c����
		// �����Z���T�o�l����
		/*
		SpringL[i].setMass(SensorMass[i]);							// ���ʂ̐ݒ�
		SpringL[i].setSize(SensorSize[i][0], SensorSize[i][1], SensorSize[i][2]);	// �T�C�Y�̐ݒ�
		SpringL[i].setPos(SensorPos[i][0], SensorPos[i][1], SensorPos[i][2]-0.5E-2);		// �ʒu�̐ݒ�
		SpringL[i].setAngle(1, 0, 0, 0*M_PI/180.0);					// �X���̐ݒ�
		createBox(SpringL[i], x0, y0, z0);							// �p�[�c����
*/
		// �����Z���T��LegX[7]�ɌŒ�
		SenLJoint[i] = dJointCreateFixed(world, 0);					// �Œ�֐�
		dJointAttach(SenLJoint[i], SensorL[i].body, LegL[7].body);	// �Œ�֐߂̎�t��
		dJointSetFixed(SenLJoint[i]);								// �Œ�֐߂̐ݒ�
		/*
		// �����Z���T�o�l��ݒ�
		SrgLJoint[i] = dJointCreateSlider(world, 0);				// �X���C�_�[�Z���T
		dJointAttach(SrgLJoint[i], LegL[7].body, SpringL[i].body);	
		dJointSetSliderAxis(SrgLJoint[i],0,0,1);
		dJointSetSliderParam(SrgLJoint[i],dParamLoStop, 0);	
		dJointSetSliderParam(SrgLJoint[i],dParamHiStop, 0);
		dJointSetSliderParam(SrgLJoint[i],dParamStopCFM,0.2);
		dJointSetSliderParam(SrgLJoint[i],dParamStopERP,0.2); 
*/

		// �E���Z���T����
		SensorR[i].setMass(SensorMass[i]);							// ���ʂ̐ݒ�
		SensorR[i].setSize(SensorSize[i][0], SensorSize[i][1], SensorSize[i][2]);	// �T�C�Y�̐ݒ�
		SensorR[i].setPos(SensorPos[i][0], -SensorPos[i][1], SensorPos[i][2]);		// �ʒu�̐ݒ�
		SensorR[i].setAngle(1, 0, 0, 0*M_PI/180.0);					// �X���̐ݒ�
		createBox(SensorR[i], x0, y0, z0);							// �p�[�c����
		/*
		// �E���Z���T�o�l����
		SpringR[i].setMass(SensorMass[i]);							// ���ʂ̐ݒ�
		SpringR[i].setSize(SensorSize[i][0], SensorSize[i][1], SensorSize[i][2]);	// �T�C�Y�̐ݒ�
		SpringR[i].setPos(SensorPos[i][0], -SensorPos[i][1], SensorPos[i][2]-0.5E-2);		// �ʒu�̐ݒ�
		SpringR[i].setAngle(1, 0, 0, 0*M_PI/180.0);					// �X���̐ݒ�
		createBox(SpringR[i], x0, y0, z0);							// �p�[�c����
*/
		// �E���Z���T��LegX[7]�ɌŒ�
		SenRJoint[i] = dJointCreateFixed(world, 0);					// �Œ�֐�
		dJointAttach(SenRJoint[i], SensorR[i].body, LegR[7].body);	// �Œ�֐߂̎�t��
		dJointSetFixed(SenRJoint[i]);								// �Œ�֐߂̐ݒ�
/*
		// �E���Z���T�o�l��ݒ�
		SrgRJoint[i] = dJointCreateSlider(world, 0);				// �X���C�_�[�Z���T
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
// ���{�b�g�̕`�惁�\�b�h
/*****************************************************************************************/
void Robot::drawRobot()
{

	// �p�[�c�̕`��
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

// �|���U�q�̕`��i�d�S�ʒu�ƃx�[�X����̒����j
void Robot::drawPendulum()
{
	// �d�S�ʒu�̕`��
	const dReal *R;
	R = dBodyGetRotation(Body[0].body);
	dsSetColor(1.0, 0.0, 0.0); 
	dsDrawSphereD(absGl, R, (float)0.02);
	
	//body[1]�̈ʒu�\��
	const dReal *body0;
	body0 = dBodyGetPosition(Body[1].body);
	dsDrawSphereD(body0, R, (float)0.02);

	// �x�[�X����d�S�ւ̒����`��
	const dReal *base;
	if(getL_or_R()==-1){
		base = dBodyGetPosition(LegL[7].body);
	}else{
		base = dBodyGetPosition(LegR[7].body);
	}
	dsDrawLineD(absGl,base);

}

// ���n�ڕW�ʒu�̕`��
void Robot::drawRefPos()
{
	// ���n�ڕW�ʒu�̕`��
	const dReal *R;
	R = dBodyGetRotation(Body[0].body);
	dsSetColor(1.0, 0.0, 0.0);

	dsDrawSphereD(refPos, R, (float)0.02);

}




/*****************************************************************************************/
// ���{�b�g�̔j�󃁃\�b�h
/*****************************************************************************************/
void Robot::destroyRobot()
{
	// �o�[�c�̔j��
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

	// �W���C���g�̔j��
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
