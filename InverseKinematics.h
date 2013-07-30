/********************************************************/
/*	copyright by Shinichi Hasebe (2008-2011)			*/
/********************************************************/
#include <math.h>

/********************************************************/
/*	ãtâ^ìÆäwåvéZ										*/
/********************************************************/


//**********************í«â¡(éRìc***********************//
#define FOOT_HEIGHT 0.01			//ë´ÉpÅ[ÉcÇÃàÍî‘â∫ÇÃïîï™
#define WAIST_WIDTH	0.04			//ë´ÇÃïù(ï‡ïùÇ≈ÇÕÇ»Ç¢)
#define CALF		28.8E-2			//Ç”Ç≠ÇÁÇÕÇ¨(LegX[3]ÇÃí∑Ç≥)
#define THIGH		28.8E-2			//ëæÇ‡Ç‡    (LegX[4]ÇÃí∑Ç≥)
#define ZS			70.0E-2//96.5E-2			//ë´ëSëÃÇÃçÇÇ≥(LegX[0]Å`LegX[7]ÇÃçÇÇ≥)

double theta[7];
double l_theta[7];
double r_theta[7];
//******************************************************//
/*	ë´ä÷êﬂÇÃäpìx[rad]	*/


double sign_in(double x){
	if     (x>0){return  1;}
	else if(x<0){return -1;}
	else        {return  0;} 
}

void InverseKinematics(double x, double y, double z, int L_or_R)
{
	
	double D, beta, gamma, Cb, Cg, R[4][4];

	if(L_or_R==0){D =   WAIST_WIDTH;}
	else	 {D = - WAIST_WIDTH;}

	double p2[3] = {0, D, ZS};

	//theta[0] = atan2(y-D, x);

	theta[0] = 0.0;//3.14/36;

	//printf("a");

	double r[3]  = {(p2[0] - x) *   cos(theta[0]) + (p2[1] - y) * sin(theta[0]),
					(p2[0] - x) * - sin(theta[0]) + (p2[1] - y) * cos(theta[0]),
					 p2[2] - (z + FOOT_HEIGHT + 0.032  )			};

	//*****************åvéZéÆÇÌÇ©ÇÁÇ»Ç¢ÇØÇ«Ç±Ç±Ç≈é~Ç‹ÇÈ07/12*************//
	if( sqrt(pow(r[0],2)+pow(r[1],2)+pow(r[2],2)) > THIGH + CALF){
		//printf("Leg Impossible!\n");
		return;
	}
	//*******************************************************************//
	
	Cb       = (pow(THIGH, 2) - (pow(r[0], 2) + pow(r[1], 2) + pow(r[2], 2)) + pow(CALF, 2))/
			   (2 * THIGH * CALF);

	Cg       = (pow(CALF, 2) + (pow(r[0], 2) + pow(r[1], 2) + pow(r[2], 2)) - pow(THIGH, 2))/
			   (2 * CALF * sqrt(pow(r[0], 2) + pow(r[1], 2) + pow(r[2], 2)));

	beta     =   atan2(sqrt(1 - pow(Cb, 2)),Cb);	//ïGì‡ë§ÇÃäpìx

	gamma    =   atan2(sqrt(1 - pow(Cg, 2)),Cg);	//

	theta[4] =   M_PI - beta;

	theta[5] = - atan2(r[0], sign_in(r[2]) * sqrt(pow(r[1],2) + pow(r[2],2))) - gamma;

	theta[6] = - atan2(r[1], r[2]);		//ÉÇÅ[É^âÒì]ï˚å¸Ç™àŸÇ»ÇÈÇΩÇﬂïœçX

	R[3][1]  =   sin(theta[4]+theta[5]) * cos(theta[6]);
	
	R[3][3]  =   cos(theta[4]+theta[5]) * cos(theta[6]);

	theta[1] =   theta[0];

	theta[2] =   theta[6];				//theta[6]ïœçXÇ…î∫Ç¢ïœçX
	
	theta[3] =   atan2(-R[3][1], R[3][3]);
	

	//*******************Ç±ÇÍÇPÇ∆ÇUÇ™îΩëŒÅHÅH*******************//
	if(L_or_R == 0){
		l_theta[0] = theta[1];
		l_theta[1] = theta[2];
		l_theta[2] = theta[3];
		l_theta[3] = theta[4];
		l_theta[4] = theta[5];
		l_theta[5] = theta[6];
	}

	else if(L_or_R == 1){
		r_theta[0] = theta[1];
		r_theta[1] = theta[2];
		r_theta[2] = theta[3];
		r_theta[3] = theta[4];
		r_theta[4] = theta[5];
		r_theta[5] = theta[6];
	}
	

}

