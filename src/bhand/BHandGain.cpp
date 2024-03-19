#include "BHand.h"
#include "memory.h"
#include "math.h"

void BHand::SetGains(int motionType)
{
	switch (motionType)
	{
	case eMotionType_HOME:
	case eMotionType_ENVELOP:
	case eMotionType_JOINT_PD:
		{
			// Might be a good idea to update the d gains for the new hand to dampen some oscillation

			//// ALLEGRO HAND 1.0
			//// Finger 1
			//_kp[0][0] = 600;
			//_kp[0][1] = 1000;
			//_kp[0][2] = 1000;
			//_kp[0][3] = 1000;
			//_kd[0][0] = 15;
			//_kd[0][1] = 20;
			//_kd[0][2] = 15;
			//_kd[0][3] = 15;

			//// Finger 2
			//_kp[1][0] = 600;
			//_kp[1][1] = 1000;
			//_kp[1][2] = 1000;
			//_kp[1][3] = 1000;
			//_kd[1][0] = 15;
			//_kd[1][1] = 20;
			//_kd[1][2] = 15;
			//_kd[1][3] = 15;
			//// Finger 3
			//_kp[2][0] = 600;
			//_kp[2][1] = 1000;
			//_kp[2][2] = 1000;
			//_kp[2][3] = 1000;
			//_kd[2][0] = 15;
			//_kd[2][1] = 20;
			//_kd[2][2] = 15;
			//_kd[2][3] = 15;

			//// Finger 4
			//_kp[3][0] = 1000;
			//_kp[3][1] = 600;
			//_kp[3][2] = 600;
			//_kp[3][3] = 600;
			//_kd[3][0] = 30;
			//_kd[3][1] = 20;
			//_kd[3][2] = 20;
			//_kd[3][3] = 15;


			//// New gains, updated using BR014 for testing on May 1, 2013
			//// ALLEGRO HAND 2.1
			//// Finger 1
			//_kp[0][0] = 600;
			//_kp[0][1] = 1000;
			//_kp[0][2] = 1000;
			//_kp[0][3] = 1000;
			//_kd[0][0] = 15;   // this finger has really low friction at the first joint on BR014
			//_kd[0][1] = 20;   // make sure these values look good on all hands. likely change them 
			//_kd[0][2] = 15;   // the to match the ones below.
			//_kd[0][3] = 15;

			//// Finger 2
			//_kp[1][0] = 600;
			//_kp[1][1] = 1000;
			//_kp[1][2] = 1000;
			//_kp[1][3] = 1000;
			//_kd[1][0] = 15;
			//_kd[1][1] = 20;
			//_kd[1][2] = 15;
			//_kd[1][3] = 15; // 15 -> 30 to avoid some oscillation at the middle finger tip on v2.0

			//// Finger 3
			//_kp[2][0] = 600;
			//_kp[2][1] = 1000;
			//_kp[2][2] = 1000;
			//_kp[2][3] = 1000;
			//_kd[2][0] = 15;
			//_kd[2][1] = 20;
			//_kd[2][2] = 15;
			//_kd[2][3] = 15;

			//// Finger 4
			//_kp[3][0] = 1000;
			//_kp[3][1] = 600;
			//_kp[3][2] = 600;
			//_kp[3][3] = 600;
			//_kd[3][0] = 30;
			//_kd[3][1] = 35;
			//_kd[3][2] = 20;
			//_kd[3][3] = 15;


			//// New gains, updated using BR014 for testing on May 1, 2013
			//// ALLEGRO HAND 2.1
			//// Finger 1,2 ,3 (2014 01 10)
			//_kp[0][0] = _kp[1][0] = _kp[2][0] = 300; //500 40
			//_kp[0][1] = _kp[1][1] = _kp[2][1] = 600;
			//_kp[0][2] = _kp[1][2] = _kp[2][2] = 700;
			//_kp[0][3] = _kp[1][3] = _kp[2][3] = 400;
			//_kd[0][0] = _kd[1][0] = _kd[2][0] = 25;   // this finger has really low friction at the first joint on BR014
			//_kd[0][1] = _kd[1][1] = _kd[2][1] = 40;   // make sure these values look good on all hands. likely change them 
			//_kd[0][2] = _kd[1][2] = _kd[2][2] = 45;   // the to match the ones below.
			//_kd[0][3] = _kd[1][3] = _kd[2][3] = 30;

			//// Finger 4
			//_kp[3][0] = 1000;
			//_kp[3][1] = 600;
			//_kp[3][2] = 600;
			//_kp[3][3] = 600;
			//_kd[3][0] = 30;
			//_kd[3][1] = 35;
			//_kd[3][2] = 20;
			//_kd[3][3] = 15;

			////// Finger 2
			////_kp[1][0] = 300;
			////_kp[1][1] = 600;
			////_kp[1][2] = 700;
			////_kp[1][3] = 400;
			////_kd[1][0] = 25;
			////_kd[1][1] = 40;
			////_kd[1][2] = 45;
			////_kd[1][3] = 30; // 15 -> 30 to avoid some oscillation at the middle finger tip on v2.0

			////// Finger 3
			////_kp[2][0] = 300;
			////_kp[2][1] = 600;
			////_kp[2][2] = 700;
			////_kp[2][3] = 400;
			////_kd[2][0] = 25;
			////_kd[2][1] = 40;
			////_kd[2][2] = 45;
			////_kd[2][3] = 30;

			////// Finger 4
			////_kp[3][0] = 1000;
			////_kp[3][1] = 600;
			////_kp[3][2] = 600;
			////_kp[3][3] = 600;
			////_kd[3][0] = 30;
			////_kd[3][1] = 35;
			////_kd[3][2] = 20;
			////_kd[3][3] = 15;


			// ALLEGRO HAND 3.0
			// Finger 1,2 ,3 (2014 01 11)
			_kp[0][0] = _kp[1][0] = _kp[2][0] = 500; //500 40
			_kp[0][1] = _kp[1][1] = _kp[2][1] = 800;
			_kp[0][2] = _kp[1][2] = _kp[2][2] = 900;
			_kp[0][3] = _kp[1][3] = _kp[2][3] = 500;
			_kd[0][0] = _kd[1][0] = _kd[2][0] = 25;   // this finger has really low friction at the first joint on BR014
			_kd[0][1] = _kd[1][1] = _kd[2][1] = 50;   // make sure these values look good on all hands. likely change them 
			_kd[0][2] = _kd[1][2] = _kd[2][2] = 55;   // the to match the ones below.
			_kd[0][3] = _kd[1][3] = _kd[2][3] = 40;

			// Finger 4
			_kp[3][0] = 1000;
			_kp[3][1] = 700;
			_kp[3][2] = 600;
			_kp[3][3] = 600;
			_kd[3][0] = 50;
			_kd[3][1] = 50;
			_kd[3][2] = 50;
			_kd[3][3] = 40;


			// Added by Alex to null force for Arrow display
			_f_des[0] = 0;
			_f_des[1] = 0;
			_f_des[2] = 0;
			_f_des[3] = 0;
		}
		break;

	case eMotionType_MOVE_OBJ:
	case eMotionType_GRAVITY_COMP:
	case eMotionType_OBJECT_MOVING:
	case eMotionType_FINGERTIP_MOVING:
	case eMotionType_READY:
		{
			//// Finger 1
			//_kp[0][0] = 120; // was 120 orignially
			//_kp[0][1] = 120;
			//_kp[0][2] = 120;
			//_kp[0][3] = 120;
			//_kd[0][0] = sqrt(_kp[0][0])*0.004*1.0;
			//_kd[0][1] = sqrt(_kp[0][1])*0.004*1.0;
			//_kd[0][2] = sqrt(_kp[0][2])*0.004*1.0;
			//_kd[0][3] = sqrt(_kp[0][3])*0.004*1.0;
			//_kp_task[0][0] = 0;
			//_kp_task[0][1] = 0;
			//_kp_task[0][2] = 0;
			//_kp_task[0][3] = 0;
			//_kd_task[0][0] = 0;
			//_kd_task[0][1] = 0;
			//_kd_task[0][2] = 0;
			//_kd_task[0][3] = 0;

			//// Finger 2
			//_kp[1][0] = 120;
			//_kp[1][1] = 120;
			//_kp[1][2] = 120;
			//_kp[1][3] = 120;
			//_kd[1][0] = sqrt(_kp[1][0])*0.004*1.0;
			//_kd[1][1] = sqrt(_kp[1][1])*0.004*1.0;
			//_kd[1][2] = sqrt(_kp[1][2])*0.004*1.0;
			//_kd[1][3] = sqrt(_kp[1][3])*0.004*1.0;
			//_kp_task[1][0] = 0;
			//_kp_task[1][1] = 0;
			//_kp_task[1][2] = 0;
			//_kp_task[1][3] = 0;
			//_kd_task[1][0] = 0;
			//_kd_task[1][1] = 0;
			//_kd_task[1][2] = 0;
			//_kd_task[1][3] = 0;

			//// Finger 3
			//_kp[2][0] = 120;
			//_kp[2][1] = 120;
			//_kp[2][2] = 120;
			//_kp[2][3] = 120;
			//_kd[2][0] = sqrt(_kp[2][0])*0.004*1.0;
			//_kd[2][1] = sqrt(_kp[2][1])*0.004*1.0;
			//_kd[2][2] = sqrt(_kp[2][2])*0.004*1.0;
			//_kd[2][3] = sqrt(_kp[2][3])*0.004*1.0;
			//_kp_task[2][0] = 0;
			//_kp_task[2][1] = 0;
			//_kp_task[2][2] = 0;
			//_kp_task[2][3] = 0;
			//_kd_task[2][0] = 0;
			//_kd_task[2][1] = 0;
			//_kd_task[2][2] = 0;
			//_kd_task[2][3] = 0;

			//// Finger 4
			//_kp[3][0] = 120;
			//_kp[3][1] = 120;
			//_kp[3][2] = 120;
			//_kp[3][3] = 120;
			//_kd[3][0] = sqrt(_kp[3][0])*0.004*1.0;
			//_kd[3][1] = sqrt(_kp[3][1])*0.004*1.0;
			//_kd[3][2] = sqrt(_kp[3][2])*0.004*1.0;
			//_kd[3][3] = sqrt(_kp[3][3])*0.004*1.0;
			//_kp_task[3][0] = 0;
			//_kp_task[3][1] = 0;
			//_kp_task[3][2] = 0;
			//_kp_task[3][3] = 0;
			//_kd_task[3][0] = 0;
			//_kd_task[3][1] = 0;
			//_kd_task[3][2] = 0;
			//_kd_task[3][3] = 0;


			
			// ALLEGRO HAND 3.0
			// Fingers 1,2 ,3 (2014 01 10)
			_kp[0][0] = _kp[1][0] = _kp[2][0] = 80; // was 120 orignially
			_kp[0][1] = _kp[1][1] = _kp[2][1] = 90;
			_kp[0][2] = _kp[1][2] = _kp[2][2] = 90;
			_kp[0][3] = _kp[1][3] = _kp[2][3] = 80;
			_kd[0][0] = _kd[1][0] = _kd[2][0] = sqrt(_kp[0][0])*0.004*1.0;
			_kd[0][1] = _kd[1][1] = _kd[2][1] = sqrt(_kp[0][1])*0.004*2.0;
			_kd[0][2] = _kd[1][2] = _kd[2][2] = sqrt(_kp[0][2])*0.004*2.0;
			_kd[0][3] = _kd[1][3] = _kd[2][3] = sqrt(_kp[0][3])*0.004*1.0;
			_kp_task[0][0] = _kp_task[1][0] = _kp_task[2][0] = 0;
			_kp_task[0][1] = _kp_task[1][1] = _kp_task[2][1] = 0;
			_kp_task[0][2] = _kp_task[1][2] = _kp_task[2][2] = 0;
			_kp_task[0][3] = _kp_task[1][3] = _kp_task[2][3] = 0;
			_kd_task[0][0] = _kd_task[1][0] = _kd_task[2][0] = 0;
			_kd_task[0][1] = _kd_task[1][1] = _kd_task[2][1] = 0;
			_kd_task[0][2] = _kd_task[1][2] = _kd_task[2][2] = 0;
			_kd_task[0][3] = _kd_task[1][3] = _kd_task[2][3] = 0;

			// Finger 4
			_kp[3][0] = 90;//120;
			_kp[3][1] = 90;//120;
			_kp[3][2] = 90;//120;
			_kp[3][3] = 90;//120;
			_kd[3][0] = sqrt(_kp[3][0])*0.004*4.0;
			_kd[3][1] = sqrt(_kp[3][1])*0.004*2.0;
			_kd[3][2] = sqrt(_kp[3][2])*0.004*1.0;
			_kd[3][3] = sqrt(_kp[3][3])*0.004*1.0;
			_kp_task[3][0] = 0;
			_kp_task[3][1] = 0;
			_kp_task[3][2] = 0;
			_kp_task[3][3] = 0;
			_kd_task[3][0] = 0;
			_kd_task[3][1] = 0;
			_kd_task[3][2] = 0;
			_kd_task[3][3] = 0;


			// Etc
			_f_des[0] = 0;
			_f_des[1] = 0;
			_f_des[2] = 0;
			_f_des[3] = 0;
		}
		break;

	case eMotionType_PRE_SHAPE:
	case eMotionType_GRASP_3:
	case eMotionType_GRASP_4:
		{
			_f_des[0] = (eMotionType_PRE_SHAPE == motionType ? 40.0 : 4.0);
			_f_des[1] = 3.0;
			_f_des[2] = (eMotionType_GRASP_4 == motionType ? 3.0 : 0.0);
			_f_des[3] = 3.0;

			// Finger 1
			_kp[0][0] = 0;
			_kp[0][1] = 0;
			_kp[0][2] = 0;
			_kp[0][3] = 0;
			_kd[0][0] = sqrt(_f_des[0])*0.01;
			_kd[0][1] = sqrt(_f_des[0])*0.01;
			_kd[0][2] = sqrt(_f_des[0])*0.01;
			_kd[0][3] = sqrt(_f_des[0])*0.01;
			_kp_task[0][0] = 0;
			_kp_task[0][1] = 0;
			_kp_task[0][2] = 0;
			_kp_task[0][3] = 0;
			_kd_task[0][0] = sqrt(_f_des[0])*0.0;
			_kd_task[0][1] = sqrt(_f_des[0])*0.0;
			_kd_task[0][2] = sqrt(_f_des[0])*0.0;
			_kd_task[0][3] = sqrt(_f_des[0])*0.0;

			// Finger 2
			_kp[1][0] = 0;
			_kp[1][1] = 0;
			_kp[1][2] = 0;
			_kp[1][3] = 0;
			_kd[1][0] = sqrt(_f_des[1])*0.01;
			_kd[1][1] = sqrt(_f_des[1])*0.01;
			_kd[1][2] = sqrt(_f_des[1])*0.01;
			_kd[1][3] = sqrt(_f_des[1])*0.01;
			_kp_task[1][0] = 0;
			_kp_task[1][1] = 0;
			_kp_task[1][2] = 0;
			_kp_task[1][3] = 0;
			_kd_task[1][0] = sqrt(_f_des[1])*0.0;
			_kd_task[1][1] = sqrt(_f_des[1])*0.0;
			_kd_task[1][2] = sqrt(_f_des[1])*0.0;
			_kd_task[1][3] = sqrt(_f_des[1])*0.0;

			// Finger 3
			_kp[2][0] = (eMotionType_GRASP_3 == motionType ? 120.0 : 0.0);
			_kp[2][1] = (eMotionType_GRASP_3 == motionType ? 120.0 : 0.0);
			_kp[2][2] = (eMotionType_GRASP_3 == motionType ? 120.0 : 0.0);
			_kp[2][3] = (eMotionType_GRASP_3 == motionType ? 120.0 : 0.0);
			_kd[2][0] = (eMotionType_GRASP_3 == motionType ? sqrt(_kp[2][0])*0.005*1.0 : sqrt(_f_des[2])*0.01);
			_kd[2][1] = (eMotionType_GRASP_3 == motionType ? sqrt(_kp[2][1])*0.005*3.0 : sqrt(_f_des[2])*0.01);
			_kd[2][2] = (eMotionType_GRASP_3 == motionType ? sqrt(_kp[2][2])*0.005*2.0 : sqrt(_f_des[2])*0.01);
			_kd[2][3] = (eMotionType_GRASP_3 == motionType ? sqrt(_kp[2][3])*0.005*1.0 : sqrt(_f_des[2])*0.01);
			_kp_task[2][0] = 0;
			_kp_task[2][1] = 0;
			_kp_task[2][2] = 0;
			_kp_task[2][3] = 0;
			_kd_task[2][0] = (eMotionType_GRASP_3 == motionType ? sqrt(_kp[2][0])*0.04 : sqrt(_f_des[2])*0.0);
			_kd_task[2][1] = (eMotionType_GRASP_3 == motionType ? sqrt(_kp[2][1])*0.04 : sqrt(_f_des[2])*0.0);
			_kd_task[2][2] = (eMotionType_GRASP_3 == motionType ? sqrt(_kp[2][2])*0.04 : sqrt(_f_des[2])*0.0);
			_kd_task[2][3] = (eMotionType_GRASP_3 == motionType ? sqrt(_kp[2][3])*0.04 : sqrt(_f_des[2])*0.0);

			// Finger 4
			_kp[3][0] = 0;
			_kp[3][1] = 0;
			_kp[3][2] = 0;
			_kp[3][3] = 0;
			_kd[3][0] = sqrt(_f_des[3])*0.03;
			_kd[3][1] = sqrt(_f_des[3])*0.03;
			_kd[3][2] = sqrt(_f_des[3])*0.001;
			_kd[3][3] = sqrt(_f_des[3])*0.001;
			_kp_task[3][0] = 0;
			_kp_task[3][1] = 0;
			_kp_task[3][2] = 0;
			_kp_task[3][3] = 0;
			_kd_task[3][0] = sqrt(_f_des[3])*0.0;
			_kd_task[3][1] = sqrt(_f_des[3])*0.0;
			_kd_task[3][2] = sqrt(_f_des[3])*0.0;
			_kd_task[3][3] = sqrt(_f_des[3])*0.0;
		}
		break;

	case eMotionType_PINCH_IT:
		{
			_x_des[1] = _x[1] - 0.01; // set desired position for middle finger
			_y_des[1] = _y[1];
			_z_des[1] = _z[1] + 0.02;

			_f_des[0] = 5.0;
			_f_des[1] = 0;
			_f_des[2] = 0;
			_f_des[3] = 4;

			_kp[0][0] = 0;
			_kp[0][1] = 0;
			_kp[0][2] = 0;
			_kp[0][3] = 0;

			_kp[1][0] = 120;
			_kp[1][1] = 120;
			_kp[1][2] = 120;
			_kp[1][3] = 120;

			_kp[2][0] = 120;
			_kp[2][1] = 120;
			_kp[2][2] = 120;
			_kp[2][3] = 120;

			_kp[3][0] = 0;
			_kp[3][1] = 0;
			_kp[3][2] = 0;
			_kp[3][3] = 0;

			_kp_task[0][0] = 0;
			_kp_task[0][1] = 0;
			_kp_task[0][2] = 0;
			_kp_task[0][3] = 0;

			_kp_task[1][0] = 0;
			_kp_task[1][1] = 0;
			_kp_task[1][2] = 0;
			_kp_task[1][3] = 0;

			_kp_task[2][0] = 0;
			_kp_task[2][1] = 0;
			_kp_task[2][2] = 0;
			_kp_task[2][3] = 0;

			_kp_task[3][0] = 0;
			_kp_task[3][1] = 0;
			_kp_task[3][2] = 0;
			_kp_task[3][3] = 0;

			_kd[0][0] = sqrtf(_f_des[0])*0.005f;
			_kd[0][1] = sqrtf(_f_des[0])*0.005f;
			_kd[0][2] = sqrtf(_f_des[0])*0.005f;
			_kd[0][3] = sqrtf(_f_des[0])*0.005f;

			_kd[1][0] = sqrtf(_kp[1][0])*0.005f*1.0f;
			_kd[1][1] = sqrtf(_kp[1][1])*0.005f*3.0f;
			_kd[1][2] = sqrtf(_kp[1][2])*0.005f*2.0f;
			_kd[1][3] = sqrtf(_kp[1][3])*0.005f*1.0f;

			_kd[2][0] = sqrtf(_kp[2][0])*0.005f*1.0f;
			_kd[2][1] = sqrtf(_kp[2][1])*0.005f*3.0f;
			_kd[2][2] = sqrtf(_kp[2][2])*0.005f*2.0f;
			_kd[2][3] = sqrtf(_kp[2][3])*0.005f*1.0f;

			_kd[3][0] = sqrtf(_f_des[3])*0.03f;
			_kd[3][1] = sqrtf(_f_des[3])*0.03f;
			_kd[3][2] = sqrtf(_f_des[3])*0.03f;
			_kd[3][3] = sqrtf(_f_des[3])*0.03f;

			_kd_task[0][0] = sqrtf(_f_des[0])*0.001f;
			_kd_task[0][1] = sqrtf(_f_des[0])*0.001f;
			_kd_task[0][2] = sqrtf(_f_des[0])*0.001f;
			_kd_task[0][3] = sqrtf(_f_des[0])*0.001f;

			_kd_task[1][0] = sqrtf(_kp[1][0])*0.04f;
			_kd_task[1][1] = sqrtf(_kp[1][1])*0.04f;
			_kd_task[1][2] = sqrtf(_kp[1][2])*0.04f;
			_kd_task[1][3] = sqrtf(_kp[1][3])*0.04f;

			_kd_task[2][0] = sqrtf(_kp[2][0])*0.04f;
			_kd_task[2][1] = sqrtf(_kp[2][1])*0.04f;
			_kd_task[2][2] = sqrtf(_kp[2][2])*0.04f;
			_kd_task[2][3] = sqrtf(_kp[2][3])*0.04f;

			_kd_task[3][0] = sqrtf(_f_des[3])*0.001f;
			_kd_task[3][1] = sqrtf(_f_des[3])*0.001f;
			_kd_task[3][2] = sqrtf(_f_des[3])*0.001f;
			_kd_task[3][3] = sqrtf(_f_des[3])*0.001f;
		}
		break;

	case eMotionType_PINCH_MT:
		{
			_x_des[0] = _x[0] - 0.01; // set desired position for index finger
			_y_des[0] = _y[0];
			_z_des[0] = _z[0] + 0.02;

			_f_des[0] = 0;
			_f_des[1] = 5.0;
			_f_des[2] = 0;
			_f_des[3] = 4.0;

			_kp[0][0] = 120;
			_kp[0][1] = 120;
			_kp[0][2] = 120;
			_kp[0][3] = 120;

			_kp[1][0] = 0;
			_kp[1][1] = 0;
			_kp[1][2] = 0;
			_kp[1][3] = 0;

			_kp[2][0] = 120;
			_kp[2][1] = 120;
			_kp[2][2] = 120;
			_kp[2][3] = 120;

			_kp[3][0] = 0;
			_kp[3][1] = 0;
			_kp[3][2] = 0;
			_kp[3][3] = 0;

			_kp_task[0][0] = 0;
			_kp_task[0][1] = 0;
			_kp_task[0][2] = 0;
			_kp_task[0][3] = 0;

			_kp_task[1][0] = 0;
			_kp_task[1][1] = 0;
			_kp_task[1][2] = 0;
			_kp_task[1][3] = 0;

			_kp_task[2][0] = 0;
			_kp_task[2][1] = 0;
			_kp_task[2][2] = 0;
			_kp_task[2][3] = 0;

			_kp_task[3][0] = 0;
			_kp_task[3][1] = 0;
			_kp_task[3][2] = 0;
			_kp_task[3][3] = 0;

			_kd[0][0] = sqrtf(_kp[0][0])*0.005f*1.0f;
			_kd[0][1] = sqrtf(_kp[0][1])*0.005f*3.0f;
			_kd[0][2] = sqrtf(_kp[0][2])*0.005f*2.0f;
			_kd[0][3] = sqrtf(_kp[0][3])*0.005f*1.0f;

			_kd[1][0] = sqrtf(_f_des[1])*0.005f;
			_kd[1][1] = sqrtf(_f_des[1])*0.005f;
			_kd[1][2] = sqrtf(_f_des[1])*0.005f;
			_kd[1][3] = sqrtf(_f_des[1])*0.005f;

			_kd[2][0] = sqrtf(_kp[2][0])*0.005f*1.0f;
			_kd[2][1] = sqrtf(_kp[2][1])*0.005f*3.0f;
			_kd[2][2] = sqrtf(_kp[2][2])*0.005f*2.0f;
			_kd[2][3] = sqrtf(_kp[2][3])*0.005f*1.0f;

			_kd[3][0] = sqrtf(_f_des[3])*0.03f;
			_kd[3][1] = sqrtf(_f_des[3])*0.03f;
			_kd[3][2] = sqrtf(_f_des[3])*0.03f;
			_kd[3][3] = sqrtf(_f_des[3])*0.03f;

			_kd_task[0][0] = sqrtf(_kp[0][0])*0.04f;
			_kd_task[0][1] = sqrtf(_kp[0][1])*0.04f;
			_kd_task[0][2] = sqrtf(_kp[0][2])*0.04f;
			_kd_task[0][3] = sqrtf(_kp[0][3])*0.04f;

			_kd_task[1][0] = sqrtf(_f_des[1])*0.001f;
			_kd_task[1][1] = sqrtf(_f_des[1])*0.001f;
			_kd_task[1][2] = sqrtf(_f_des[1])*0.001f;
			_kd_task[1][3] = sqrtf(_f_des[1])*0.001f;

			_kd_task[2][0] = sqrtf(_kp[2][0])*0.04f;
			_kd_task[2][1] = sqrtf(_kp[2][1])*0.04f;
			_kd_task[2][2] = sqrtf(_kp[2][2])*0.04f;
			_kd_task[2][3] = sqrtf(_kp[2][3])*0.04f;

			_kd_task[3][0] = sqrtf(_f_des[3])*0.001f;
			_kd_task[3][1] = sqrtf(_f_des[3])*0.001f;
			_kd_task[3][2] = sqrtf(_f_des[3])*0.001f;
			_kd_task[3][3] = sqrtf(_f_des[3])*0.001f;
		}
		break;

	case eMotionType_NONE:
	default:
		{
			memset(_kp, 0, sizeof(_kp));
			memset(_kd, 0, sizeof(_kd));
			memset(_kp_task, 0, sizeof(_kp_task));
			memset(_kd_task, 0, sizeof(_kd_task));
		}
		break;
	}
}
