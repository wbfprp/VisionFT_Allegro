#include "BHand/BHand.h"
#include "memory.h"
#include "math.h"
#include "stdio.h"

BHand::BHand(eHandType ht)
{
	_dT = 0.003;
	_handType = ht;
	_motionType = eMotionType_NONE;

	memset(_q, 0, sizeof(_q));
	memset(_q_filtered, 0, sizeof(_q_filtered));
	memset(_q_pre, 0, sizeof(_q_pre));
	memset(_q_filtered_pre, 0, sizeof(_q_filtered_pre));
	memset(_qdot, 0, sizeof(_qdot));
	memset(_qdot_filtered, 0, sizeof(_qdot_filtered));
	memset(_qdot_pre, 0, sizeof(_qdot_pre));
	memset(_tau_des, 0, sizeof(_tau_des));
	memset(_q_des, 0, sizeof(_q_des));

	_envelop_torque_scalar = 1.0;

	memset(_x, 0, sizeof(_x));
	memset(_y, 0, sizeof(_y));
	memset(_z, 0, sizeof(_z));
	memset(_x_filtered, 0, sizeof(_x_filtered));
	memset(_y_filtered, 0, sizeof(_y_filtered));
	memset(_z_filtered, 0, sizeof(_z_filtered));
	memset(_x_pre, 0, sizeof(_x_pre));
	memset(_y_pre, 0, sizeof(_y_pre));
	memset(_z_pre, 0, sizeof(_z_pre));
	memset(_x_filtered_pre, 0, sizeof(_x_filtered_pre));
	memset(_y_filtered_pre, 0, sizeof(_y_filtered_pre));
	memset(_z_filtered_pre, 0, sizeof(_z_filtered_pre));

	memset(_xdot, 0, sizeof(_xdot));
	memset(_ydot, 0, sizeof(_ydot));
	memset(_zdot, 0, sizeof(_zdot));
	memset(_xdot_filtered, 0, sizeof(_xdot_filtered));
	memset(_ydot_filtered, 0, sizeof(_ydot_filtered));
	memset(_zdot_filtered, 0, sizeof(_zdot_filtered));
	memset(_xdot_pre, 0, sizeof(_xdot_pre));
	memset(_ydot_pre, 0, sizeof(_ydot_pre));
	memset(_zdot_pre, 0, sizeof(_zdot_pre));

	memset(_kp, 0, sizeof(_kp));
	memset(_kd, 0, sizeof(_kd));
	memset(_kp_task, 0, sizeof(_kp_task));
	memset(_kd_task, 0, sizeof(_kd_task));

	memset(_J, 0, sizeof(_J));

	memset(_G, 0, sizeof(_G));

	memset(_f_des, 0, sizeof(_f_des));

	memset(_x_des, 0, sizeof(_x_des));
	memset(_y_des, 0, sizeof(_y_des));
	memset(_z_des, 0, sizeof(_z_des));

	/*_mass[0][0] = 0.010f * 1.5f; // Original BHand Library masses used
	_mass[0][1] = 0.063f * 1.5f;
	_mass[0][2] = 0.035f * 1.5f;
	_mass[0][3] = 0.028f * 1.5f;

	_mass[1][0] = 0.010f * 1.5f;
	_mass[1][1] = 0.063f * 1.5f;
	_mass[1][2] = 0.035f * 1.5f;
	_mass[1][3] = 0.028f * 1.5f;

	_mass[2][0] = 0.010f * 1.5f;
	_mass[2][1] = 0.063f * 1.5f;
	_mass[2][2] = 0.035f * 1.5f;
	_mass[2][3] = 0.028f * 1.5f;

	_mass[3][0] = 0.082f * 1.5f;
	_mass[3][1] = 0.010f * 1.5f;
	_mass[3][2] = 0.035f * 1.5f;
	_mass[3][3] = 0.056f * 1.0f;*/

	_mass[0][0] = 0.0112f * 1.5f; // Massed  from wiki. Measured at SimLab
	_mass[0][1] = 0.0682f * 1.5f;
	_mass[0][2] = 0.0402f * 1.5f;
	_mass[0][3] = 0.0270f * 1.5f;

	_mass[1][0] = 0.0112f * 1.5f;
	_mass[1][1] = 0.0682f * 1.5f;
	_mass[1][2] = 0.0402f * 1.5f;
	_mass[1][3] = 0.0270f * 1.5f;

	_mass[2][0] = 0.0112f * 1.5f;
	_mass[2][1] = 0.0682f * 1.5f;
	_mass[2][2] = 0.0402f * 1.5f;
	_mass[2][3] = 0.0270f * 1.5f;

	_mass[3][0] = 0.0900f * 1.5f;
	_mass[3][1] = 0.0112f * 1.5f;
	//_mass[3][2] = 0.0510f * 15.5f;
	//_mass[3][3] = 0.0600f * 2.0f; 
	_mass[3][2] = 0.0510f * 1.5f;
	_mass[3][3] = 0.0600f * 1.5f;
	// changes because of wire. no effect yet

	// initialize body orientation
	memset(_R, 0, sizeof(_R[0]) * 9);
	_R[0] = _R[4] = _R[8] = 1.0;
}

BHand::~BHand(void)
{
}

eHandType BHand::GetType()
{
	return _handType;
}

void BHand::SetTimeInterval(double dT)
{
	_dT = dT;
}

double BHand::GetTimeInterval()
{
	return _dT;
	//printf("%f",_dT);
}

void BHand::SetMotionType(int motionType)
{
	_curT = 0;
	_motionType = (eMotionType)motionType;
	SetGains(motionType);
}

void BHand::SetJointPosition(double* q)
{
	memcpy(_q_pre, _q, SIZEOF_VARRAY);
	memcpy(_q, q, SIZEOF_VARRAY);
	memcpy(_q_filtered_pre, _q_filtered, SIZEOF_VARRAY);
	memcpy(_qdot_pre, _qdot, SIZEOF_VARRAY);
	for (int i = 0; i < NOF; i++)
	{
		for (int j = 0; j < NOJ; j++)
		{
			_q_filtered[i][j] = (0.6 * _q_filtered[i][j]) + (0.198 * _q_pre[i][j]) + (0.198 * _q[i][j]);
			_qdot[i][j] = (_q_filtered[i][j] - _q_filtered_pre[i][j]) / _dT;
			_qdot_filtered[i][j] = (0.6 * _qdot_filtered[i][j]) + (0.198 * _qdot_pre[i][j]) + (0.198 * _qdot[i][j]);
		}

		S1_C[i] = sin(_q[i][0]);
		C1_C[i] = cos(_q[i][0]);
		S2_C[i] = sin(_q[i][1]);
		C2_C[i] = cos(_q[i][1]);
		S3_C[i] = sin(_q[i][2]);
		C3_C[i] = cos(_q[i][2]);
		S4_C[i] = sin(_q[i][3]);
		C4_C[i] = cos(_q[i][3]);
		S23_C[i] = sin(_q[i][1] + _q[i][2]);
		C23_C[i] = cos(_q[i][1] + _q[i][2]);
		S34_C[i] = sin(_q[i][2] + _q[i][3]);
		C34_C[i] = cos(_q[i][2] + _q[i][3]);
		S234_C[i] = sin(_q[i][1] + _q[i][2] + _q[i][3]);
		C234_C[i] = cos(_q[i][1] + _q[i][2] + _q[i][3]);
	}

	SolveFK();
	CalculateJacobian();
	//CalculateGravity();
	CalculateGravityEx();
}

void BHand::UpdateControl(double time)
{
	_curT += _dT;

	switch (_motionType)
	{
	case eMotionType_HOME:
		Motion_HomePosition();
		return;
		break;

	case eMotionType_READY:
		Motion_Ready();
		break;

	case eMotionType_GRAVITY_COMP:
		Motion_GravityComp();
		//printf ("grav\n");
		break;

	case eMotionType_MOVE_OBJ:
		Motion_ReadyToMove();
		break;

		//case eMotionType_MOVE_FINGERTIP:
		//	Motion_ObjectMoving();
		//	break;

	case eMotionType_PRE_SHAPE:
		Motion_PreShape();
		break;

	case eMotionType_GRASP_3:
		Motion_Grasp3();
		break;

	case eMotionType_GRASP_4:
		Motion_Grasp4();
		break;

	case eMotionType_PINCH_IT:
		Motion_PinchIT();
		break;

	case eMotionType_PINCH_MT:
		Motion_PinchMT();
		break;

	case eMotionType_OBJECT_MOVING:
		Motion_ObjectMoving();
		break;

	case eMotionType_FINGERTIP_MOVING:
		Motion_FingertipMoving();
		break;

	case eMotionType_FINGERTIP_MOVING_SIMPLE:
		Motion_FingertipMoving_Simple();
		break;

	case eMotionType_ENVELOP:
		Motion_Envelop();
		return;
		break;

	case eMotionType_JOINT_PD:
		Motion_JointPD();
		return;
		break;

	case eMotionType_NONE:
	default:
		memset(_tau_des, 0, SIZEOF_VARRAY);
		return;
		break;
	}


	int i = 0;
	float o = 0.5f;
	float c = 1.0f;

	float fv[4][4];
	float fc[4][4];

	float Scalef = 800.0f;

	float e_x[4];
	float e_y[4];
	float e_z[4];

	float e_x_object;
	float e_y_object;
	float e_z_object;

	float xc_object;
	float yc_object;
	float zc_object;

	//float x_d_object;
	//float y_d_object;
	//float z_d_object;

	float t_JointDamping[4][4];
	float t_TaskDamping[4][4];
	float t_Friction[4][4];
	float t_Position[4][4];
	float t_Pinching[4][4];
	float t_Position_Object[4][4];
	float t_Gravity[4][4];

	///////////////////////////////////////////
	//// coulomb friction coefficient
	//// Tune these for gravity compensation in Allegro Hand 2.0
	//fc[0][0] = 10.0f / Scalef * o;
	//fc[0][1] = 20.0f / Scalef * o;
	////fc[0][2] = 50.0f / Scalef * o;
	//fc[0][2] = 20.0f / Scalef * o;
	//fc[0][3] = 10.0f / Scalef * o; // maybe make this higher for all?

	//fc[1][0] = 10.0f / Scalef * o;
	//fc[1][1] = 30.0f / Scalef * o;
	////fc[1][2] = 50.0f / Scalef * o;
	//fc[1][2] = 20.0f / Scalef * o;
	//fc[1][3] = 10.0f / Scalef * o;

	//fc[2][0] = 10.0f / Scalef * o;
	//fc[2][1] = 30.0f / Scalef * o;
	////fc[2][2] = 50.0f / Scalef * o;
	//fc[2][2] = 20.0f / Scalef * o;
	//fc[2][3] = 10.0f / Scalef * o;

	//fc[3][0] = 0.0f / Scalef * o;
	//fc[3][1] = 30.0f / Scalef * o;
	//fc[3][2] = 30.0f / Scalef * o;
	//fc[3][3] = 30.0f / Scalef * o;











	//// coulomb friction coefficient
	//// Tune these for gravity compensation in Allegro Hand 2.0
	//fc[0][0] = 10.0f / Scalef * o;
	//fc[0][1] = 30.0f / Scalef * o;
	//fc[0][2] = 20.0f / Scalef * o;
	//fc[0][3] = 10.0f / Scalef * o; // maybe make this higher for all?

	//fc[1][0] = 10.0f / Scalef * o;
	//fc[1][1] = 30.0f / Scalef * o;
	//fc[1][2] = 20.0f / Scalef * o;
	//fc[1][3] = 10.0f / Scalef * o;

	//fc[2][0] = 10.0f / Scalef * o;
	//fc[2][1] = 30.0f / Scalef * o;
	//fc[2][2] = 20.0f / Scalef * o;
	//fc[2][3] = 10.0f / Scalef * o;

	//fc[3][0] = 0.0f / Scalef * o;
	//fc[3][1] = 30.0f / Scalef * o;
	//fc[3][2] = 30.0f / Scalef * o;
	//fc[3][3] = 30.0f / Scalef * o;

	/////////////////////////////////////////////
	//// viscous friction coefficient
	//fv[0][0] = 3.0f / Scalef * c;
	//fv[0][1] = 3.0f / Scalef * c;
	//fv[0][2] = 3.0f / Scalef * c;
	//fv[0][3] = 0.5f / Scalef * c;

	//fv[1][0] = 3.0f / Scalef * c;
	//fv[1][1] = 3.0f / Scalef * c;
	//fv[1][2] = 3.0f / Scalef * c;
	//fv[1][3] = 1.0f / Scalef * c;

	//fv[2][0] = 3.0f / Scalef * c;
	//fv[2][1] = 3.0f / Scalef * c;
	//fv[2][2] = 3.0f / Scalef * c;
	//fv[2][3] = 0.5f / Scalef * c;

	//fv[3][0] = 2.0f / Scalef * c;
	//fv[3][1] = 1.0f / Scalef * c;
	//fv[3][2] = 2.0f / Scalef * c;
	//fv[3][3] = 2.0f / Scalef * c;








	////////// ALLEGRO HAND 2.1 ////////////////////
	// coulomb friction coefficient
	// Tune these for gravity compensation in Allegro Hand 2.0
	fc[0][0] = fc[1][0] = fc[2][0] = 2.0f / Scalef * o;
	fc[0][1] = fc[1][1] = fc[2][1] = 30.0f / Scalef * o;
	fc[0][2] = fc[1][2] = fc[2][2] = 20.0f / Scalef * o;
	fc[0][3] = fc[1][3] = fc[2][3] = 10.0f / Scalef * o; // maybe make this higher for all?

	//fc[1][0] = 20.0f / Scalef * o;
	//fc[1][1] = 30.0f / Scalef * o;
	//fc[1][2] = 20.0f / Scalef * o;
	//fc[1][3] = 10.0f / Scalef * o;

	//fc[2][0] = 20.0f / Scalef * o;
	//fc[2][1] = 30.0f / Scalef * o;
	//fc[2][2] = 20.0f / Scalef * o;
	//fc[2][3] = 10.0f / Scalef * o;

	fc[3][0] = 0.0f / Scalef * o;
	fc[3][1] = 30.0f / Scalef * o;
	fc[3][2] = 30.0f / Scalef * o;
	fc[3][3] = 30.0f / Scalef * o;

	////////// ALLEGRO HAND 2.1 ////////////////////
	// viscous friction coefficient
	fv[0][0] = fv[1][0] = fv[2][0] = 2.0f / Scalef * c;//20.0f / Scalef * c;
	fv[0][1] = fv[1][1] = fv[2][1] = 3.0f / Scalef * c;
	fv[0][2] = fv[1][2] = fv[2][2] = 3.0f / Scalef * c;
	fv[0][3] = fv[1][3] = fv[2][3] = 0.5f / Scalef * c;

	//fv[1][0] = 10.0f / Scalef * c;
	//fv[1][1] = 3.0f / Scalef * c;
	//fv[1][2] = 3.0f / Scalef * c;
	//fv[1][3] = 1.0f / Scalef * c;

	//fv[2][0] = 10.0f / Scalef * c;
	//fv[2][1] = 3.0f / Scalef * c;
	//fv[2][2] = 3.0f / Scalef * c;
	//fv[2][3] = 0.5f / Scalef * c;

	fv[3][0] = 2.0f / Scalef * c;
	fv[3][1] = 1.0f / Scalef * c;
	fv[3][2] = 2.0f / Scalef * c;
	fv[3][3] = 2.0f / Scalef * c;
	///////////////////////////////////////////





	//
	xc_object = (_x[0] + _x[1] + _x[3]) / 3;
	yc_object = (_y[0] + _y[1] + _y[3]) / 3;
	zc_object = (_z[0] + _z[1] + _z[3]) / 3;

	x_d_object = 0.0f;
	y_d_object = 0.0f; // 10.0f*sinf(5*_curT);
	z_d_object = 0.0f;

	////printf("Disp. x: %f\t y: %f\t z: %f\t\n", x_d_object, y_d_object, z_d_object);

	e_x_object = x_d_object - xc_object;
	e_y_object = y_d_object - yc_object;
	e_z_object = z_d_object - zc_object;

	////printf("Obj. x: %f\t y: %f\t z: %f\t\n\n", e_x_object, e_y_object, e_z_object);

	for (i = 0; i < NOF; i++)
	{
		e_x[i] = (_x_des[i] - _x[i]);
		e_y[i] = (_y_des[i] - _y[i]);
		e_z[i] = (_z_des[i] - _z[i]);

		//printf("e_xyz:\t%f \t %f \t %f\n", e_x[i],e_y[i],e_z[i]);

		///////////////////////////////////////////
		// joint damping
		t_JointDamping[i][0] = 1.0f * _kd[i][0] * _qdot_filtered[i][0];
		t_JointDamping[i][1] = 1.0f * _kd[i][1] * _qdot_filtered[i][1];
		t_JointDamping[i][2] = 1.0f * _kd[i][2] * _qdot_filtered[i][2];
		t_JointDamping[i][3] = 1.0f * _kd[i][3] * _qdot_filtered[i][3];

		///////////////////////////////////////////
		// task damping
		t_TaskDamping[i][0] = 0;
		t_TaskDamping[i][1] = 0;
		t_TaskDamping[i][2] = 0;
		t_TaskDamping[i][3] = 0;

		///////////////////////////////////////////
		// friction
		t_Friction[i][0] = (fv[i][0] * _qdot_filtered[i][0]) + (fc[i][0] * (float)tanh(_qdot_filtered[i][0] * 50.0f));
		t_Friction[i][1] = (fv[i][1] * _qdot_filtered[i][1]) + (fc[i][1] * (float)tanh(_qdot_filtered[i][1] * 40.0f));
		t_Friction[i][2] = (fv[i][2] * _qdot_filtered[i][2]) + (fc[i][2] * (float)tanh(_qdot_filtered[i][2] * 50.0f));
		t_Friction[i][3] = (fv[i][3] * _qdot_filtered[i][3]) + (fc[i][3] * (float)tanh(_qdot_filtered[i][3] * 10.0f));

		///////////////////////////////////////////
		// task position
		t_Position[i][0] = (_J[i][0][0] * _kp[i][0] * (e_x[i]) + _J[i][1][0] * _kp[i][0] * (e_y[i]) + _J[i][2][0] * _kp[i][0] * (e_z[i]));
		t_Position[i][1] = (_J[i][0][1] * _kp[i][1] * (e_x[i]) + _J[i][1][1] * _kp[i][1] * (e_y[i]) + _J[i][2][1] * _kp[i][1] * (e_z[i]));
		t_Position[i][2] = (_J[i][0][2] * _kp[i][2] * (e_x[i]) + _J[i][1][2] * _kp[i][2] * (e_y[i]) + _J[i][2][2] * _kp[i][2] * (e_z[i]));
		t_Position[i][3] = (_J[i][0][3] * _kp[i][3] * (e_x[i]) + _J[i][1][3] * _kp[i][3] * (e_y[i]) + _J[i][2][3] * _kp[i][3] * (e_z[i]));

		///////////////////////////////////////////
		// desired pinching force
		t_Pinching[i][0] = (_J[i][0][0] * _f_des[i] * (e_x[i]) + _J[i][1][0] * _f_des[i] * (e_y[i]) + _J[i][2][0] * _f_des[i] * (e_z[i]));
		t_Pinching[i][1] = (_J[i][0][1] * _f_des[i] * (e_x[i]) + _J[i][1][1] * _f_des[i] * (e_y[i]) + _J[i][2][1] * _f_des[i] * (e_z[i]));
		t_Pinching[i][2] = (_J[i][0][2] * _f_des[i] * (e_x[i]) + _J[i][1][2] * _f_des[i] * (e_y[i]) + _J[i][2][2] * _f_des[i] * (e_z[i]));
		t_Pinching[i][3] = (_J[i][0][3] * _f_des[i] * (e_x[i]) + _J[i][1][3] * _f_des[i] * (e_y[i]) + _J[i][2][3] * _f_des[i] * (e_z[i]));

		///////////////////////////////////////////
		// object position
		t_Position_Object[i][0] = (_J[i][0][0] * _kp_task[i][0] * (e_x_object)+_J[i][1][0] * _kp_task[i][0] * (e_y_object)+_J[i][2][0] * _kp_task[i][0] * (e_z_object));
		t_Position_Object[i][1] = (_J[i][0][1] * _kp_task[i][1] * (e_x_object)+_J[i][1][1] * _kp_task[i][1] * (e_y_object)+_J[i][2][1] * _kp_task[i][1] * (e_z_object));
		t_Position_Object[i][2] = (_J[i][0][2] * _kp_task[i][2] * (e_x_object)+_J[i][1][2] * _kp_task[i][2] * (e_y_object)+_J[i][2][2] * _kp_task[i][2] * (e_z_object));
		t_Position_Object[i][3] = (_J[i][0][3] * _kp_task[i][3] * (e_x_object)+_J[i][1][3] * _kp_task[i][3] * (e_y_object)+_J[i][2][3] * _kp_task[i][3] * (e_z_object));

		///////////////////////////////////////////
		// gravity
		t_Gravity[i][0] = _G[i][0];
		t_Gravity[i][1] = _G[i][1];
		t_Gravity[i][2] = _G[i][2];
		t_Gravity[i][3] = _G[i][3];

		///////////////////////////////////////////
		// total
		_tau_des[i][0] = ((t_Position[i][0] + t_Position_Object[i][0] - t_TaskDamping[i][0] - t_JointDamping[i][0] + t_Pinching[i][0] + t_Gravity[i][0] + t_Friction[i][0]) * (1));
		_tau_des[i][1] = ((t_Position[i][1] + t_Position_Object[i][1] - t_TaskDamping[i][1] - t_JointDamping[i][1] + t_Pinching[i][1] + t_Gravity[i][1] + t_Friction[i][1]) * (1));
		_tau_des[i][2] = ((t_Position[i][2] + t_Position_Object[i][2] - t_TaskDamping[i][2] - t_JointDamping[i][2] + t_Pinching[i][2] + t_Gravity[i][2] + t_Friction[i][2]) * (1));
		_tau_des[i][3] = ((t_Position[i][3] + t_Position_Object[i][3] - t_TaskDamping[i][3] - t_JointDamping[i][3] + t_Pinching[i][3] + t_Gravity[i][3] + t_Friction[i][3]) * (1));
	}
}

void BHand::GetJointTorque(double* tau)
{
	memcpy(tau, _tau_des, SIZEOF_VARRAY);
}

void BHand::Motion_HomePosition()
{
	static double q_home_left[NOF][NOJ] = {
		{  0 * DEG2RAD,	-10 * DEG2RAD,	45 * DEG2RAD,		45 * DEG2RAD},
		{  0 * DEG2RAD,	-10 * DEG2RAD,	45 * DEG2RAD,		45 * DEG2RAD},
		{ -5 * DEG2RAD,	 -5 * DEG2RAD,	50 * DEG2RAD,		45 * DEG2RAD},
		{ 50 * DEG2RAD,	 25 * DEG2RAD,	15 * DEG2RAD,		45 * DEG2RAD}
	};

	static double q_home_right[NOF][NOJ] = {
		{  0 * DEG2RAD,	-10 * DEG2RAD,	45 * DEG2RAD,		45 * DEG2RAD},
		{  0 * DEG2RAD,	-10 * DEG2RAD,	45 * DEG2RAD,		45 * DEG2RAD},
		{  5 * DEG2RAD,	 -5 * DEG2RAD,	50 * DEG2RAD,		45 * DEG2RAD},
		{ 50 * DEG2RAD,	 25 * DEG2RAD,	15 * DEG2RAD,		45 * DEG2RAD}
	};

	for (int i = 0; i < NOF; i++) {
		for (int j = 0; j < NOJ; j++) {
			if (_handType == eHandType_Left)
				_tau_des[i][j] = _kp[i][j] * (q_home_left[i][j] - _q_filtered[i][j]) - _kd[i][j] * _qdot_filtered[i][j];
			else
				_tau_des[i][j] = _kp[i][j] * (q_home_right[i][j] - _q_filtered[i][j]) - _kd[i][j] * _qdot_filtered[i][j];
			_tau_des[i][j] /= 800.0; // pwm to torque
		}
	}
}

void BHand::Motion_ReadyToMove() // Same as BHand::Motion_GravityComp()
{
	_x_des[0] = _x[0];
	_y_des[0] = _y[0];
	_z_des[0] = _z[0];

	_x_des[1] = _x[1];
	_y_des[1] = _y[1];
	_z_des[1] = _z[1];

	_x_des[2] = _x[2];
	_y_des[2] = _y[2];
	_z_des[2] = _z[2];

	_x_des[3] = _x[3];
	_y_des[3] = _y[3];
	_z_des[3] = _z[3];
}

void BHand::Motion_Ready()
{
	if (_handType == eHandType_Left)
	{
		_x_des[0] = 0.08;
		_y_des[0] = -0.048;
		_z_des[0] = 0.10;

		_x_des[1] = 0.08;
		_y_des[1] = 0.0;
		_z_des[1] = 0.10;

		_x_des[2] = 0.08;
		_y_des[2] = 0.045;
		_z_des[2] = 0.08;

		_x_des[3] = 0.11;
		_y_des[3] = -0.040;
		_z_des[3] = -0.04;
	}
	else
	{
		_x_des[0] = 0.08;
		_y_des[0] = 0.048;
		_z_des[0] = 0.10;

		_x_des[1] = 0.08;
		_y_des[1] = 0.0;
		_z_des[1] = 0.10;

		_x_des[2] = 0.08;
		_y_des[2] = -0.045;
		_z_des[2] = 0.08;

		_x_des[3] = 0.11;
		_y_des[3] = 0.040;
		_z_des[3] = -0.04;
	}
}



void BHand::Motion_GravityComp()
{


	_x_des[0] = _x[0];
	_y_des[0] = _y[0];
	_z_des[0] = _z[0];

	_x_des[1] = _x[1];
	_y_des[1] = _y[1];
	_z_des[1] = _z[1];

	_x_des[2] = _x[2];
	_y_des[2] = _y[2];
	_z_des[2] = _z[2];

	_x_des[3] = _x[3];
	_y_des[3] = _y[3];
	_z_des[3] = _z[3];
}

void BHand::Motion_PreShape()
{
}

void BHand::Motion_Grasp3()
{
	float distance[4];
	float delta_x[4];
	float delta_y[4];
	float delta_z[4];
	float alpha[4];

	float center_x_geo, center_y_geo, center_z_geo;

	//alpha[0] = ( m_fd[0]*sinf(10*m_Time_cur) ) + ;
	//alpha[0] = ( 2.0f*sinf(5*_curT) ) + 2.5f;

	//alpha[0] = ( 10.0f*sinf(2*_curT) ) + 13.5f; //stronger, unstable


	alpha[0] = _f_des[0];

	center_x_geo = (_x[0] + _x[1]) / 2.0f;
	center_y_geo = (_y[0] + _y[1]) / 2.0f;
	center_z_geo = (_z[0] + _z[1]) / 2.0f;



	_x_des[0] = _x[3]; //index
	_x_des[1] = _x[3]; //middle
	_x_des[3] = center_x_geo; //thumb

	_y_des[0] = _y[3]; //+y_move;
	_y_des[1] = _y[3]; //+y_move;
	_y_des[3] = center_y_geo; //+y_move;

	_z_des[0] = _z[3]; //+0.01;
	_z_des[1] = _z[3]; //+0.01;
	_z_des[3] = center_z_geo;

	distance[0] = sqrtf((_x[0] - _x_des[0]) * (_x[0] - _x_des[0]) + (_y[0] - _y_des[0]) * (_y[0] - _y_des[0]) + (_z[0] - _z_des[0]) * (_z[0] - _z_des[0]));
	distance[1] = sqrtf((_x[1] - _x_des[1]) * (_x[1] - _x_des[1]) + (_y[1] - _y_des[1]) * (_y[1] - _y_des[1]) + (_z[1] - _z_des[1]) * (_z[1] - _z_des[1]));
	distance[3] = sqrtf((_x[3] - _x_des[3]) * (_x[3] - _x_des[3]) + (_y[3] - _y_des[3]) * (_y[3] - _y_des[3]) + (_z[3] - _z_des[3]) * (_z[3] - _z_des[3]));

	delta_x[0] = (_x_des[0] - _x[0]) / (distance[0]);
	delta_y[0] = (_y_des[0] - _y[0]) / (distance[0]);
	delta_z[0] = (_z_des[0] - _z[0]) / (distance[0]);

	delta_x[1] = (_x_des[1] - _x[1]) / (distance[1]);
	delta_y[1] = (_y_des[1] - _y[1]) / (distance[1]);
	delta_z[1] = (_z_des[1] - _z[1]) / (distance[1]);

	delta_x[3] = (_x_des[3] - _x[3]) / (distance[3]);
	delta_y[3] = (_y_des[3] - _y[3]) / (distance[3]);
	delta_z[3] = (_z_des[3] - _z[3]) / (distance[3]);

	_x_des[0] = _x[0] + delta_x[0];
	_y_des[0] = _y[0] + delta_y[0];
	_z_des[0] = _z[0] + delta_z[0];

	_x_des[1] = _x[1] + delta_x[1];
	_y_des[1] = _y[1] + delta_y[1];
	_z_des[1] = _z[1] + delta_z[1];

	_x_des[3] = _x[3] + delta_x[3];
	_y_des[3] = _y[3] + delta_y[3];
	_z_des[3] = _z[3] + delta_z[3];

	alpha[1] = (-1) * alpha[0] * (delta_x[0] - delta_x[3] * delta_y[0] / delta_y[3]) / (delta_x[1] - delta_x[3] * delta_y[1] / delta_y[3]);
	alpha[3] = (-1) * alpha[0] * delta_z[0] - alpha[1] * delta_z[1] + 0.003f * 9.81f;

	_f_des[0] = alpha[0];// + 0.03f*sinf(_curT);
	_f_des[1] = alpha[1];// + 0.03f*sinf(_curT);
	_f_des[3] = alpha[3];// + 0.03f*sinf(_curT);

	// printf("%f %f %f\n", _f_des[0],_f_des[1],_f_des[3]);
	//_f_des[0] = alpha[0]*sinf(_curT) + 0.1f;
	//_f_des[1] = alpha[1]*sinf(_curT) + 0.1f;
	//_f_des[3] = alpha[3]*sinf(_curT) + 0.1f;
}

void BHand::Motion_Grasp4()
{
	float distance[4];
	float delta_x[4];
	float delta_y[4];
	float delta_z[4];
	float alpha[4];

	float center_x_geo, center_y_geo, center_z_geo;

	alpha[0] = _f_des[0];

	center_x_geo = (_x[0] + _x[1] + _x[2]) / 3.0f;
	center_y_geo = (_y[0] + _y[1] + _y[2]) / 3.0f;
	center_z_geo = (_z[0] + _z[1] + _z[2]) / 3.0f;

	_x_des[0] = _x[3];
	_x_des[1] = _x[3];
	_x_des[2] = _x[3];
	_x_des[3] = center_x_geo;

	_y_des[0] = _y[3];
	_y_des[1] = _y[3];
	_y_des[2] = _y[3];
	_y_des[3] = center_y_geo;

	_z_des[0] = _z[3] + 0.02;
	_z_des[1] = _z[3] + 0.02;
	_z_des[2] = _z[3] + 0.02;
	_z_des[3] = center_z_geo;

	distance[0] = sqrtf((_x[0] - _x_des[0]) * (_x[0] - _x_des[0]) + (_y[0] - _y_des[0]) * (_y[0] - _y_des[0]) + (_z[0] - _z_des[0]) * (_z[0] - _z_des[0]));
	distance[1] = sqrtf((_x[1] - _x_des[1]) * (_x[1] - _x_des[1]) + (_y[1] - _y_des[1]) * (_y[1] - _y_des[1]) + (_z[1] - _z_des[1]) * (_z[1] - _z_des[1]));
	distance[2] = sqrtf((_x[2] - _x_des[2]) * (_x[2] - _x_des[2]) + (_y[2] - _y_des[2]) * (_y[2] - _y_des[2]) + (_z[2] - _z_des[2]) * (_z[2] - _z_des[2]));
	distance[3] = sqrtf((_x[3] - _x_des[3]) * (_x[3] - _x_des[3]) + (_y[3] - _y_des[3]) * (_y[3] - _y_des[3]) + (_z[3] - _z_des[3]) * (_z[3] - _z_des[3]));

	delta_x[0] = (_x_des[0] - _x[0]) / (distance[0]);
	delta_y[0] = (_y_des[0] - _y[0]) / (distance[0]);
	delta_z[0] = (_z_des[0] - _z[0]) / (distance[0]);

	delta_x[1] = (_x_des[1] - _x[1]) / (distance[1]);
	delta_y[1] = (_y_des[1] - _y[1]) / (distance[1]);
	delta_z[1] = (_z_des[1] - _z[1]) / (distance[1]);

	delta_x[2] = (_x_des[2] - _x[2]) / (distance[2]);
	delta_y[2] = (_y_des[2] - _y[2]) / (distance[2]);
	delta_z[2] = (_z_des[2] - _z[2]) / (distance[2]);

	delta_x[3] = (_x_des[3] - _x[3]) / (distance[3]);
	delta_y[3] = (_y_des[3] - _y[3]) / (distance[3]);
	delta_z[3] = (_z_des[3] - _z[3]) / (distance[3]);

	_x_des[0] = _x[0] + delta_x[0];
	_y_des[0] = _y[0] + delta_y[0];
	_z_des[0] = _z[0] + delta_z[0];

	_x_des[1] = _x[1] + delta_x[1];
	_y_des[1] = _y[1] + delta_y[1];
	_z_des[1] = _z[1] + delta_z[1];

	_x_des[2] = _x[2] + delta_x[2];
	_y_des[2] = _y[2] + delta_y[2];
	_z_des[2] = _z[2] + delta_z[2];

	_x_des[3] = _x[3] + delta_x[3];
	_y_des[3] = _y[3] + delta_y[3];
	_z_des[3] = _z[3] + delta_z[3];

	alpha[1] = (distance[0] / distance[1]) * alpha[0];
	alpha[2] = alpha[0];

	delta_x[0] = alpha[0] * delta_x[0];
	delta_y[0] = alpha[0] * delta_y[0];
	delta_z[0] = alpha[0] * delta_z[0];

	delta_x[1] = alpha[1] * delta_x[1];
	delta_y[1] = alpha[1] * delta_y[1];
	delta_z[1] = alpha[1] * delta_z[1];

	delta_x[2] = alpha[2] * delta_x[2];
	delta_y[2] = alpha[2] * delta_y[2];
	delta_z[2] = alpha[2] * delta_z[2];

	alpha[3] = sqrtf((delta_x[0] + delta_x[1] + delta_x[2]) * (delta_x[0] + delta_x[1] + delta_x[2]) + (delta_y[0] + delta_y[1] + delta_y[2]) * (delta_y[0] + delta_y[1] + delta_y[2]) + (delta_z[0] + delta_z[1] + delta_z[2]) * (delta_z[0] + delta_z[1] + delta_z[2]));

	_f_des[1] = alpha[1];
	_f_des[2] = alpha[2];
	_f_des[3] = alpha[3];
}

void BHand::Motion_PinchIT()
{
	float distance[4];
	float delta_x[4];
	float delta_y[4];
	float delta_z[4];
	float alpha[4];
	float yTPos = 0.0f;
	static float pinchOffset_y = -0.01;//0.002;

	static bool dir = true;
	if (!isFSSent) {
		dir = !dir;
		if (dir)
			pinchOffset_y = 0.01;
		else
			pinchOffset_y = -0.01;
		isFSSent = true;
	}


	float center_x_geo, center_y_geo, center_z_geo;

	
	float pinchOffset_z = 0.055;

	alpha[0] = _f_des[0];

	center_x_geo = (_x[0] + _x[3]) / 2.0f;
	center_y_geo = (_y[0] + _y[3]) / 2.0f;
	center_z_geo = (_z[0] + _z[3]) / 2.0f;

	_x_des[0] = center_x_geo;
	_x_des[3] = center_x_geo;

	_y_des[0] = center_y_geo + pinchOffset_y / 2.0;
	_y_des[3] = center_y_geo - pinchOffset_y / 2.0;

	_z_des[0] = center_z_geo + pinchOffset_z / 2.0;
	_z_des[3] = center_z_geo - pinchOffset_z / 2.0;

	distance[0] = sqrtf((_x[0] - _x_des[0]) * (_x[0] - _x_des[0]) + (_y[0] - _y_des[0]) * (_y[0] - _y_des[0]) + (_z[0] - _z_des[0]) * (_z[0] - _z_des[0]));
	distance[3] = sqrtf((_x[3] - _x_des[3]) * (_x[3] - _x_des[3]) + (_y[3] - _y_des[3]) * (_y[3] - _y_des[3]) + (_z[3] - _z_des[3]) * (_z[3] - _z_des[3]));

	delta_x[0] = (_x_des[0] - _x[0]) / (distance[0]);
	delta_y[0] = (_y_des[0] - _y[0]) / (distance[0]);
	delta_z[0] = (_z_des[0] - _z[0]) / (distance[0]);

	delta_x[3] = (_x_des[3] - _x[3]) / (distance[3]);
	delta_y[3] = (_y_des[3] - _y[3]) / (distance[3]);
	delta_z[3] = (_z_des[3] - _z[3]) / (distance[3]);

	_x_des[0] = _x[0] + delta_x[0];
	_y_des[0] = _y[0] + delta_y[0];
	_z_des[0] = _z[0] + delta_z[0];// +pinchOffset_z / 2.0;

	if (_handType == eHandType_Left)
	{
		_x_des[1] = 0.08f;
		_y_des[1] = 0.0f;
		_z_des[1] = 0.095f;

		_x_des[2] = 0.08f;
		_y_des[2] = 0.040f;
		_z_des[2] = 0.095f;
	}
	else
	{
		_x_des[1] = 0.08f;
		_y_des[1] = 0.0f;
		_z_des[1] = 0.095f;

		_x_des[2] = 0.08f;
		_y_des[2] = -0.040f;
		_z_des[2] = 0.095f;
	}

	_x_des[3] = _x[3] + delta_x[3];
	_y_des[3] = _y[3] + delta_y[3];
	_z_des[3] = _z[3] + delta_z[3];// -pinchOffset_z / 2.0;

	alpha[3] = (distance[0] / distance[3]) * alpha[0];

	_f_des[3] = alpha[3];
}

void BHand::Motion_PinchMT()
{
	float distance[4];
	float delta_x[4];
	float delta_y[4];
	float delta_z[4];
	float alpha[4];


	float center_x_geo, center_y_geo, center_z_geo;

	alpha[1] = _f_des[1];

	center_x_geo = (_x[1] + _x[3]) / 2.0f;
	center_y_geo = (_y[1] + _y[3]) / 2.0f;
	center_z_geo = (_z[1] + _z[3]) / 2.0f;

	_x_des[1] = center_x_geo;
	_x_des[3] = center_x_geo;

	_y_des[1] = center_y_geo;
	_y_des[3] = center_y_geo;

	_z_des[1] = center_z_geo;
	_z_des[3] = center_z_geo;

	distance[1] = sqrtf((_x[1] - _x_des[1]) * (_x[1] - _x_des[1]) + (_y[1] - _y_des[1]) * (_y[1] - _y_des[1]) + (_z[1] - _z_des[1]) * (_z[1] - _z_des[1]));
	distance[3] = sqrtf((_x[3] - _x_des[3]) * (_x[3] - _x_des[3]) + (_y[3] - _y_des[3]) * (_y[3] - _y_des[3]) + (_z[3] - _z_des[3]) * (_z[3] - _z_des[3]));

	delta_x[1] = (_x_des[1] - _x[1]) / (distance[1]);
	delta_y[1] = (_y_des[1] - _y[1]) / (distance[1]);
	delta_z[1] = (_z_des[1] - _z[1]) / (distance[1]);

	delta_x[3] = (_x_des[3] - _x[3]) / (distance[3]);
	delta_y[3] = (_y_des[3] - _y[3]) / (distance[3]);
	delta_z[3] = (_z_des[3] - _z[3]) / (distance[3]);

	_x_des[1] = _x[1] + delta_x[1];
	_y_des[1] = _y[1] + delta_y[1];
	_z_des[1] = _z[1] + delta_z[1];

	if (_handType == eHandType_Left)
	{
		_x_des[0] = 0.08f;
		_y_des[0] = -0.05f;
		_z_des[0] = 0.095f;

		_x_des[2] = 0.08f;
		_y_des[2] = 0.040f;
		_z_des[2] = 0.095f;
	}
	else
	{
		_x_des[0] = 0.08f;
		_y_des[0] = 0.05f;
		_z_des[0] = 0.095f;

		_x_des[2] = 0.08f;
		_y_des[2] = -0.040f;
		_z_des[2] = 0.095f;
	}



	_x_des[3] = _x[3] + delta_x[3];
	_y_des[3] = _y[3] + delta_y[3];
	_z_des[3] = _z[3] + delta_z[3];



	alpha[3] = (distance[1] / distance[3]) * alpha[1];

	_f_des[3] = alpha[3];
}

void BHand::Motion_ObjectMoving()
{

}

void BHand::Motion_FingertipMoving()
{

	_x_des[0] = _set_x_des[0];
	_y_des[0] = _set_y_des[0];
	_z_des[0] = _set_z_des[0];

	_x_des[1] = _set_x_des[1];
	_y_des[1] = _set_y_des[1];
	_z_des[1] = _set_z_des[1];

	_x_des[2] = _set_x_des[2];
	_y_des[2] = _set_y_des[2];
	_z_des[2] = _set_z_des[2];

	_x_des[3] = _set_x_des[3];
	_y_des[3] = _set_y_des[3];
	_z_des[3] = _set_z_des[3];

}

void BHand::Motion_FingertipMoving_Simple()
{

}

void BHand::Motion_Envelop()
{
	static double q_des_left[NOF][NOJ] = {
		{ 10 * DEG2RAD,	60 * DEG2RAD,		0 * DEG2RAD,		0 * DEG2RAD},
		{  0 * DEG2RAD,	60 * DEG2RAD,		0 * DEG2RAD,		0 * DEG2RAD},
		{-20 * DEG2RAD,	 0 * DEG2RAD,		0 * DEG2RAD,		0 * DEG2RAD},
		{ 80 * DEG2RAD,	30 * DEG2RAD,		0 * DEG2RAD,		0 * DEG2RAD}
	};

	static double q_des_right[NOF][NOJ] = {
		{-10 * DEG2RAD,	60 * DEG2RAD,		0 * DEG2RAD,		0 * DEG2RAD},
		{  0 * DEG2RAD,	60 * DEG2RAD,		0 * DEG2RAD,		0 * DEG2RAD},
		{ 20 * DEG2RAD,	 0 * DEG2RAD,		0 * DEG2RAD,		0 * DEG2RAD},
		{ 80 * DEG2RAD,	30 * DEG2RAD,		0 * DEG2RAD,		0 * DEG2RAD}
	};

	if (_curT > 0.15)
	{
		if (_handType == eHandType_Left)
		{
			_tau_des[3][0] = _kp[3][0] * (q_des_left[3][0] - _q_filtered[3][0]) - _kd[3][0] * _qdot_filtered[3][0];
			_tau_des[3][0] /= 800.0; // pwm to torque
			_tau_des[3][1] = _kp[3][1] * (q_des_left[3][1] - _q_filtered[3][1]) - _kd[3][1] * _qdot_filtered[3][1];
			_tau_des[3][1] /= 800.0; // pwm to torque
		}
		else
		{
			_tau_des[3][0] = _kp[3][0] * (q_des_right[3][0] - _q_filtered[3][0]) - _kd[3][0] * _qdot_filtered[3][0];
			_tau_des[3][0] /= 800.0; // pwm to torque
			_tau_des[3][1] = _kp[3][1] * (q_des_right[3][1] - _q_filtered[3][1]) - _kd[3][1] * _qdot_filtered[3][1];
			_tau_des[3][1] /= 800.0; // pwm to torque
		}
	}

	if (_handType == eHandType_Left)
	{
		_tau_des[0][0] = _kp[0][0] * (q_des_left[0][0] - _q_filtered[0][0]) - _kd[0][0] * _qdot_filtered[0][0];
		_tau_des[0][0] /= 800.0; // pwm to torque
		_tau_des[1][0] = _kp[1][0] * (q_des_left[1][0] - _q_filtered[1][0]) - _kd[1][0] * _qdot_filtered[1][0];
		_tau_des[1][0] /= 800.0; // pwm to torque
		_tau_des[2][0] = _kp[2][0] * (q_des_left[2][0] - _q_filtered[2][0]) - _kd[2][0] * _qdot_filtered[2][0];
		_tau_des[2][0] /= 800.0; // pwm to torque
	}
	else
	{
		_tau_des[0][0] = _kp[0][0] * (q_des_right[0][0] - _q_filtered[0][0]) - _kd[0][0] * _qdot_filtered[0][0];
		_tau_des[0][0] /= 800.0; // pwm to torque
		_tau_des[1][0] = _kp[1][0] * (q_des_right[1][0] - _q_filtered[1][0]) - _kd[1][0] * _qdot_filtered[1][0];
		_tau_des[1][0] /= 800.0; // pwm to torque
		_tau_des[2][0] = _kp[2][0] * (q_des_right[2][0] - _q_filtered[2][0]) - _kd[2][0] * _qdot_filtered[2][0];
		_tau_des[2][0] /= 800.0; // pwm to torque
	}

	if (_curT > 0 && _curT <= 0.05)
	{
		_tau_des[0][1] = 0;
		_tau_des[0][2] = 0;
		_tau_des[0][3] = 0;

		_tau_des[1][1] = 0;
		_tau_des[1][2] = 0;
		_tau_des[1][3] = 0;

		_tau_des[2][1] = (double)600 / 800.0;
		_tau_des[2][2] = 0;
		_tau_des[2][3] = 0;

		_tau_des[3][2] = 0;
		_tau_des[3][3] = 0;
	}
	else if (_curT > 0.05 && _curT <= 0.1)
	{
		_tau_des[0][1] = 0;
		_tau_des[0][2] = 0;
		_tau_des[0][3] = 0;

		_tau_des[1][1] = (double)600 / 800.0;
		_tau_des[1][2] = 0;
		_tau_des[1][3] = 0;

		_tau_des[2][1] = (double)600 / 800.0;
		_tau_des[2][2] = (double)360 / 800.0;
		_tau_des[2][3] = 0;

		_tau_des[3][2] = 0;
		_tau_des[3][3] = 0;
	}
	else if (_curT > 0.1 && _curT <= 0.2)
	{
		_tau_des[0][1] = (double)600 / 800.0;
		_tau_des[0][2] = 0;
		_tau_des[0][3] = 0;

		_tau_des[1][1] = (double)600 / 800.0;
		_tau_des[1][2] = (double)360 / 800.0;
		_tau_des[1][3] = 0;

		_tau_des[2][1] = (double)600 / 800.0;
		_tau_des[2][2] = (double)360 / 800.0;
		_tau_des[2][3] = (double)180 / 800.0;

		_tau_des[3][2] = (double)600 / 800.0;
		_tau_des[3][3] = 0;
	}
	else if (_curT > 0.2)
	{
		_tau_des[0][1] = (double)600 / 800.0;
		_tau_des[0][2] = (double)360 / 800.0;
		_tau_des[0][3] = (double)180 / 800.0;

		_tau_des[1][1] = (double)600 / 800.0;
		_tau_des[1][2] = (double)360 / 800.0;
		_tau_des[1][3] = (double)180 / 800.0;

		_tau_des[2][1] = (double)600 / 800.0;
		_tau_des[2][2] = (double)360 / 800.0;
		_tau_des[2][3] = (double)180 / 800.0;

		_tau_des[3][2] = (double)600 / 800.0;
		_tau_des[3][3] = (double)420 / 800.0;
	}

	//double _envelop_torque_scalar = 0.1;
		//_envelop_torque_scalar = 0.5;

	for (int i = 0; i < NOF; i++)
		for (int j = 0; j < NOJ; j++)
			_tau_des[i][j] *= _envelop_torque_scalar;

}

void BHand::Motion_JointPD()
{
	for (int i = 0; i < NOF; i++) {
		for (int j = 0; j < NOJ; j++) {
			_tau_des[i][j] = _kp[i][j] * (_q_des[i][j] - _q_filtered[i][j]) - _kd[i][j] * _qdot_filtered[i][j];
			_tau_des[i][j] /= 800.0; // pwm to torque
		}
	}
}

void BHand::SolveFK()
{
	for (int i = 0; i < NOF; i++)
	{
		_x_pre[i] = _x[i];
		_y_pre[i] = _y[i];
		_z_pre[i] = _z[i];
		_x_filtered_pre[i] = _x_filtered[i];
		_y_filtered_pre[i] = _y_filtered[i];
		_z_filtered_pre[i] = _z_filtered[i];
	}

	if (_handType == eHandType_Left)
		SolveFKLeft();
	else
		SolveFKRight();

	for (int i = 0; i < NOF; i++)
	{
		_x_filtered[i] = (0.6f * _x_filtered[i]) + (0.198f * _x_pre[i]) + (0.198f * _x[i]);
		_y_filtered[i] = (0.6f * _y_filtered[i]) + (0.198f * _y_pre[i]) + (0.198f * _y[i]);
		_z_filtered[i] = (0.6f * _z_filtered[i]) + (0.198f * _z_pre[i]) + (0.198f * _z[i]);

		_xdot_pre[i] = _xdot[i];
		_ydot_pre[i] = _ydot[i];
		_zdot_pre[i] = _zdot[i];

		_xdot[i] = (_x_filtered[i] - _x_filtered_pre[i]) / _dT;
		_ydot[i] = (_y_filtered[i] - _y_filtered_pre[i]) / _dT;
		_zdot[i] = (_z_filtered[i] - _z_filtered_pre[i]) / _dT;

		_xdot_filtered[i] = (0.6f * _xdot_filtered[i]) + (0.198f * _xdot_pre[i]) + (0.198f * _xdot[i]);
		_ydot_filtered[i] = (0.6f * _ydot_filtered[i]) + (0.198f * _ydot_pre[i]) + (0.198f * _ydot[i]);
		_zdot_filtered[i] = (0.6f * _zdot_filtered[i]) + (0.198f * _zdot_pre[i]) + (0.198f * _zdot[i]);
	}
}

void BHand::SolveFKLeft()
{
	// FK
	_x[0] = 257.0f / 10000.0f * (C1_C[0] * S2_C[0] * C3_C[0] + C1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-C1_C[0] * S2_C[0] * S3_C[0] + C1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] + 24.0f / 625.0f * C1_C[0] * S2_C[0] * C3_C[0] + 24.0f / 625.0f * C1_C[0] * C2_C[0] * S3_C[0] + 27.0f / 500.0f * C1_C[0] * S2_C[0];
	_y[0] = 257.0f / 10000.0f * ((4981.0f / 5000.0f * S1_C[0] * S2_C[0] - 871.0f / 10000.0f * C2_C[0]) * C3_C[0] + (4981.0f / 5000.0f * S1_C[0] * C2_C[0] + 871.0f / 10000.0f * S2_C[0]) * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[0] * S2_C[0] - 871.0f / 10000.0f * C2_C[0]) * S3_C[0] + (4981.0f / 5000.0f * S1_C[0] * C2_C[0] + 871.0f / 10000.0f * S2_C[0]) * C3_C[0]) * S4_C[0] - 4505231.0f / 100000000.0f + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * S2_C[0] - 871.0f / 10000.0f * C2_C[0]) * C3_C[0] + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * C2_C[0] + 871.0f / 10000.0f * S2_C[0]) * S3_C[0] + 134487.0f / 2500000.0f * S1_C[0] * S2_C[0] + 23517.0f / 5000000.0f * C2_C[0];
	_z[0] = 257.0f / 10000.0f * ((-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * C3_C[0] + (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-(-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] + (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0]) * S4_C[0] + 689941.0f / 50000000.0f + 24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * C3_C[0] + 24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0] - 23517.0f / 5000000.0f * S1_C[0] * S2_C[0] + 134487.0f / 2500000.0f * C2_C[0];

	_x[1] = 257.0f / 10000.0f * (C1_C[1] * S2_C[1] * C3_C[1] + C1_C[1] * C2_C[1] * S3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * S4_C[1] + 24.0f / 625.0f * C1_C[1] * S2_C[1] * C3_C[1] + 24.0f / 625.0f * C1_C[1] * C2_C[1] * S3_C[1] + 27.0f / 500.0f * C1_C[1] * S2_C[1];
	_y[1] = 257.0f / 10000.0f * (S1_C[1] * S2_C[1] * C3_C[1] + S1_C[1] * C2_C[1] * S3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * S3_C[1] + S1_C[1] * C2_C[1] * C3_C[1]) * S4_C[1] + 24.0f / 625.0f * S1_C[1] * S2_C[1] * C3_C[1] + 24.0f / 625.0f * S1_C[1] * C2_C[1] * S3_C[1] + 27.0f / 500.0f * S1_C[1] * S2_C[1];
	_z[1] = 257.0f / 10000.0f * (C2_C[1] * C3_C[1] - S2_C[1] * S3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-C2_C[1] * S3_C[1] - S2_C[1] * C3_C[1]) * S4_C[1] + 161.0f / 10000.0f + 24.0f / 625.0f * C2_C[1] * C3_C[1] - 24.0f / 625.0f * S2_C[1] * S3_C[1] + 27.0f / 500.0f * C2_C[1];

	_x[2] = 257.0f / 10000.0f * (C1_C[2] * S2_C[2] * C3_C[2] + C1_C[2] * C2_C[2] * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-C1_C[2] * S2_C[2] * S3_C[2] + C1_C[2] * C2_C[2] * C3_C[2]) * S4_C[2] + 24.0f / 625.0f * C1_C[2] * S2_C[2] * C3_C[2] + 24.0f / 625.0f * C1_C[2] * C2_C[2] * S3_C[2] + 27.0f / 500.0f * C1_C[2] * S2_C[2];
	_y[2] = 257.0f / 10000.0f * ((4981.0f / 5000.0f * S1_C[2] * S2_C[2] + 871.0f / 10000.0f * C2_C[2]) * C3_C[2] + (4981.0f / 5000.0f * S1_C[2] * C2_C[2] - 871.0f / 10000.0f * S2_C[2]) * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[2] * S2_C[2] + 871.0f / 10000.0f * C2_C[2]) * S3_C[2] + (4981.0f / 5000.0f * S1_C[2] * C2_C[2] - 871.0f / 10000.0f * S2_C[2]) * C3_C[2]) * S4_C[2] + 4505231.0f / 100000000.0f + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * S2_C[2] + 871.0f / 10000.0f * C2_C[2]) * C3_C[2] + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * C2_C[2] - 871.0f / 10000.0f * S2_C[2]) * S3_C[2] + 134487.0f / 2500000.0f * S1_C[2] * S2_C[2] - 23517.0f / 5000000.0f * C2_C[2];
	_z[2] = 257.0f / 10000.0f * ((871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * C3_C[2] + (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-(871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] + (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2]) * S4_C[2] + 689941.0f / 50000000.0f + 24.0f / 625.0f * (871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * C3_C[2] + 24.0f / 625.0f * (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2] + 23517.0f / 5000000.0f * S1_C[2] * S2_C[2] + 134487.0f / 2500000.0f * C2_C[2];

	_x[3] = (25.0f * C1_C[3] + 274.0f * S1_C[3] + 257.0f * C3_C[3] * S1_C[3] + 185.0f * C34_C[3] * S1_C[3] + 185.0f * S34_C[3] * C1_C[3] * S2_C[3] + 257.0f * C1_C[3] * S2_C[3] * S3_C[3] - 91.0f) / 5000.0f;
	_y[3] = -(2729588.0f * C1_C[3] - 249050.0f * S1_C[3] + 2560234.0f * C1_C[3] * C3_C[3] + 223847.0f * C2_C[3] * S3_C[3] + 1842970.0f * C34_C[3] * C1_C[3] + 161135.0f * S34_C[3] * C2_C[3] - 2560234.0f * S1_C[3] * S2_C[3] * S3_C[3] - 1842970.0f * S34_C[3] * S1_C[3] * S2_C[3] + 848689.0f) / 50000000.0f;
	_z[3] = -(238654.0f * C1_C[3] - 21775.0f * S1_C[3] + 223847.0f * C1_C[3] * C3_C[3] - 2560234.0f * C2_C[3] * S3_C[3] + 161135.0f * C34_C[3] * C1_C[3] - 1842970.0f * S34_C[3] * C2_C[3] - 223847.0f * S1_C[3] * S2_C[3] * S3_C[3] - 161135.0f * S34_C[3] * S1_C[3] * S2_C[3] + 3654642.0f) / 50000000.0f;
}

void BHand::SolveFKRight()
{
	// FK
	_x[0] = 257.0f / 10000.0f * (C1_C[0] * S2_C[0] * C3_C[0] + C1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-C1_C[0] * S2_C[0] * S3_C[0] + C1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] + 24.0f / 625.0f * C1_C[0] * S2_C[0] * C3_C[0] + 24.0f / 625.0f * C1_C[0] * C2_C[0] * S3_C[0] + 27.0f / 500.0f * C1_C[0] * S2_C[0];
	_y[0] = 257.0f / 10000.0f * ((4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * C3_C[0] + (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * S3_C[0] + (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0]) * S4_C[0] + 4505231.0f / 100000000.0f + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * C3_C[0] + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * S3_C[0] + 134487.0f / 2500000.0f * S1_C[0] * S2_C[0] + 23517.0f / 5000000.0f * C2_C[0];
	_z[0] = 257.0f / 10000.0f * ((-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * C3_C[0] + (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-(-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] + (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0]) * S4_C[0] + 689941.0f / 50000000.0f + 24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * C3_C[0] + 24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0] - 23517.0f / 5000000.0f * S1_C[0] * S2_C[0] + 134487.0f / 2500000.0f * C2_C[0];

	_x[1] = 257.0f / 10000.0f * (C1_C[1] * S2_C[1] * C3_C[1] + C1_C[1] * C2_C[1] * S3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * S4_C[1] + 24.0f / 625.0f * C1_C[1] * S2_C[1] * C3_C[1] + 24.0f / 625.0f * C1_C[1] * C2_C[1] * S3_C[1] + 27.0f / 500.0f * C1_C[1] * S2_C[1];
	_y[1] = 257.0f / 10000.0f * (S1_C[1] * S2_C[1] * C3_C[1] + S1_C[1] * C2_C[1] * S3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * S3_C[1] + S1_C[1] * C2_C[1] * C3_C[1]) * S4_C[1] + 24.0f / 625.0f * S1_C[1] * S2_C[1] * C3_C[1] + 24.0f / 625.0f * S1_C[1] * C2_C[1] * S3_C[1] + 27.0f / 500.0f * S1_C[1] * S2_C[1];
	_z[1] = 257.0f / 10000.0f * (C2_C[1] * C3_C[1] - S2_C[1] * S3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-C2_C[1] * S3_C[1] - S2_C[1] * C3_C[1]) * S4_C[1] + 161.0f / 10000.0f + 24.0f / 625.0f * C2_C[1] * C3_C[1] - 24.0f / 625.0f * S2_C[1] * S3_C[1] + 27.0f / 500.0f * C2_C[1];

	_x[2] = 257.0f / 10000.0f * (C1_C[2] * S2_C[2] * C3_C[2] + C1_C[2] * C2_C[2] * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-C1_C[2] * S2_C[2] * S3_C[2] + C1_C[2] * C2_C[2] * C3_C[2]) * S4_C[2] + 24.0f / 625.0f * C1_C[2] * S2_C[2] * C3_C[2] + 24.0f / 625.0f * C1_C[2] * C2_C[2] * S3_C[2] + 27.0f / 500.0f * C1_C[2] * S2_C[2];
	_y[2] = 257.0f / 10000.0f * ((4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * C3_C[2] + (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * S3_C[2] + (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2]) * S4_C[2] - 4505231.0f / 100000000.0f + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * C3_C[2] + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * S3_C[2] + 134487.0f / 2500000.0f * S1_C[2] * S2_C[2] - 23517.0f / 5000000.0f * C2_C[2];
	_z[2] = 257.0f / 10000.0f * ((871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * C3_C[2] + (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-(871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] + (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2]) * S4_C[2] + 689941.0f / 50000000.0f + 24.0f / 625.0f * (871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * C3_C[2] + 24.0f / 625.0f * (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2] + 23517.0f / 5000000.0f * S1_C[2] * S2_C[2] + 134487.0f / 2500000.0f * C2_C[2];

	_x[3] = (25.0f * C1_C[3] + 274.0f * S1_C[3] + 257.0f * C3_C[3] * S1_C[3] + 185.0f * C34_C[3] * S1_C[3] + 185.0f * S34_C[3] * C1_C[3] * S2_C[3] + 257.0f * C1_C[3] * S2_C[3] * S3_C[3] - 91.0f) / 5000.0f;
	_y[3] = (2729588.0f * C1_C[3] - 249050.0f * S1_C[3] + 2560234.0f * C1_C[3] * C3_C[3] + 223847.0f * C2_C[3] * S3_C[3] + 1842970.0f * C34_C[3] * C1_C[3] + 161135.0f * S34_C[3] * C2_C[3] - 2560234.0f * S1_C[3] * S2_C[3] * S3_C[3] - 1842970.0f * S34_C[3] * S1_C[3] * S2_C[3] + 848689.0f) / 50000000.0f;
	_z[3] = -(238654.0f * C1_C[3] - 21775.0f * S1_C[3] + 223847.0f * C1_C[3] * C3_C[3] - 2560234.0f * C2_C[3] * S3_C[3] + 161135.0f * C34_C[3] * C1_C[3] - 1842970.0f * S34_C[3] * C2_C[3] - 223847.0f * S1_C[3] * S2_C[3] * S3_C[3] - 161135.0f * S34_C[3] * S1_C[3] * S2_C[3] + 3654642.0f) / 50000000.0f;
}

void BHand::CalculateJacobian()
{
	_J[0][0][0] = 257.0f / 10000.0f * (-S1_C[0] * S2_C[0] * C3_C[0] - S1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (S1_C[0] * S2_C[0] * S3_C[0] - S1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] - 24.0f / 625.0f * S1_C[0] * S2_C[0] * C3_C[0] - 24.0f / 625.0f * S1_C[0] * C2_C[0] * S3_C[0] - 27.0f / 500.0f * S1_C[0] * S2_C[0];
	_J[0][0][1] = 257.0f / 10000.0f * (-C1_C[0] * S2_C[0] * S3_C[0] + C1_C[0] * C2_C[0] * C3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-C1_C[0] * C2_C[0] * S3_C[0] - C1_C[0] * S2_C[0] * C3_C[0]) * S4_C[0] + 24.0f / 625.0f * C1_C[0] * C2_C[0] * C3_C[0] - 24.0f / 625.0f * C1_C[0] * S2_C[0] * S3_C[0] + 27.0f / 500.0f * C1_C[0] * C2_C[0];
	_J[0][0][2] = 257.0f / 10000.0f * (-C1_C[0] * S2_C[0] * S3_C[0] + C1_C[0] * C2_C[0] * C3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-C1_C[0] * C2_C[0] * S3_C[0] - C1_C[0] * S2_C[0] * C3_C[0]) * S4_C[0] - 24.0f / 625.0f * C1_C[0] * S2_C[0] * S3_C[0] + 24.0f / 625.0f * C1_C[0] * C2_C[0] * C3_C[0];
	_J[0][0][3] = -257.0f / 10000.0f * (C1_C[0] * S2_C[0] * C3_C[0] + C1_C[0] * C2_C[0] * S3_C[0]) * S4_C[0] + 257.0f / 10000.0f * (-C1_C[0] * S2_C[0] * S3_C[0] + C1_C[0] * C2_C[0] * C3_C[0]) * C4_C[0];

	_J[0][1][0] = 257.0f / 10000.0f * (4981.0f / 5000.0f * C1_C[0] * S2_C[0] * C3_C[0] + 4981.0f / 5000.0f * C1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-4981.0f / 5000.0f * C1_C[0] * S2_C[0] * S3_C[0] + 4981.0f / 5000.0f * C1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] + 14943.0f / 390625.0f * C1_C[0] * S2_C[0] * C3_C[0] + 14943.0f / 390625.0f * C1_C[0] * C2_C[0] * S3_C[0] + 134487.0f / 2500000.0f * C1_C[0] * S2_C[0];
	_J[0][1][1] = 257.0f / 10000.0f * ((4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0] + (-4981.0f / 5000.0f * S1_C[0] * S2_C[0] - 871.0f / 10000.0f * C2_C[0]) * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * S3_C[0] + (-4981.0f / 5000.0f * S1_C[0] * S2_C[0] - 871.0f / 10000.0f * C2_C[0]) * C3_C[0]) * S4_C[0] + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0] + 24.0f / 625.0f * (-4981.0f / 5000.0f * S1_C[0] * S2_C[0] - 871.0f / 10000.0f * C2_C[0]) * S3_C[0] + 134487.0f / 2500000.0f * S1_C[0] * C2_C[0] - 23517.0f / 5000000.0f * S2_C[0];
	_J[0][1][2] = 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * S3_C[0] + (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * C3_C[0] - (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * S3_C[0]) * S4_C[0] - 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * S3_C[0] + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0];
	_J[0][1][3] = -257.0f / 10000.0f * ((4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * C3_C[0] + (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * S3_C[0]) * S4_C[0] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[0] * S2_C[0] + 871.0f / 10000.0f * C2_C[0]) * S3_C[0] + (4981.0f / 5000.0f * S1_C[0] * C2_C[0] - 871.0f / 10000.0f * S2_C[0]) * C3_C[0]) * C4_C[0];

	_J[0][2][0] = 257.0f / 10000.0f * (-871.0f / 10000.0f * C1_C[0] * S2_C[0] * C3_C[0] - 871.0f / 10000.0f * C1_C[0] * C2_C[0] * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (871.0f / 10000.0f * C1_C[0] * S2_C[0] * S3_C[0] - 871.0f / 10000.0f * C1_C[0] * C2_C[0] * C3_C[0]) * S4_C[0] - 2613.0f / 781250.0f * C1_C[0] * S2_C[0] * C3_C[0] - 2613.0f / 781250.0f * C1_C[0] * C2_C[0] * S3_C[0] - 23517.0f / 5000000.0f * C1_C[0] * S2_C[0];
	_J[0][2][1] = 257.0f / 10000.0f * ((-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0] + (871.0f / 10000.0f * S1_C[0] * S2_C[0] - 4981.0f / 5000.0f * C2_C[0]) * S3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-(-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0] + (871.0f / 10000.0f * S1_C[0] * S2_C[0] - 4981.0f / 5000.0f * C2_C[0]) * C3_C[0]) * S4_C[0] + 24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0] + 24.0f / 625.0f * (871.0f / 10000.0f * S1_C[0] * S2_C[0] - 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] - 23517.0f / 5000000.0f * S1_C[0] * C2_C[0] - 134487.0f / 2500000.0f * S2_C[0];
	_J[0][2][2] = 257.0f / 10000.0f * (-(-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] + (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0]) * C4_C[0] + 257.0f / 10000.0f * (-(-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * C3_C[0] - (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0]) * S4_C[0] - 24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] + 24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0];
	_J[0][2][3] = -257.0f / 10000.0f * ((-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * C3_C[0] + (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * S3_C[0]) * S4_C[0] + 257.0f / 10000.0f * (-(-871.0f / 10000.0f * S1_C[0] * S2_C[0] + 4981.0f / 5000.0f * C2_C[0]) * S3_C[0] + (-871.0f / 10000.0f * S1_C[0] * C2_C[0] - 4981.0f / 5000.0f * S2_C[0]) * C3_C[0]) * C4_C[0];

	_J[1][0][0] = 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * C3_C[1] - S1_C[1] * C2_C[1] * S3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (S1_C[1] * S2_C[1] * S3_C[1] - S1_C[1] * C2_C[1] * C3_C[1]) * S4_C[1] - 24.0f / 625.0f * S1_C[1] * S2_C[1] * C3_C[1] - 24.0f / 625.0f * S1_C[1] * C2_C[1] * S3_C[1] - 27.0f / 500.0f * S1_C[1] * S2_C[1];
	_J[1][0][1] = 257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-C1_C[1] * C2_C[1] * S3_C[1] - C1_C[1] * S2_C[1] * C3_C[1]) * S4_C[1] + 24.0f / 625.0f * C1_C[1] * C2_C[1] * C3_C[1] - 24.0f / 625.0f * C1_C[1] * S2_C[1] * S3_C[1] + 27.0f / 500.0f * C1_C[1] * C2_C[1];
	_J[1][0][2] = 257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-C1_C[1] * C2_C[1] * S3_C[1] - C1_C[1] * S2_C[1] * C3_C[1]) * S4_C[1] - 24.0f / 625.0f * C1_C[1] * S2_C[1] * S3_C[1] + 24.0f / 625.0f * C1_C[1] * C2_C[1] * C3_C[1];
	_J[1][0][3] = -257.0f / 10000.0f * (C1_C[1] * S2_C[1] * C3_C[1] + C1_C[1] * C2_C[1] * S3_C[1]) * S4_C[1] + 257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1];

	_J[1][1][0] = 257.0f / 10000.0f * (C1_C[1] * S2_C[1] * C3_C[1] + C1_C[1] * C2_C[1] * S3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-C1_C[1] * S2_C[1] * S3_C[1] + C1_C[1] * C2_C[1] * C3_C[1]) * S4_C[1] + 24.0f / 625.0f * C1_C[1] * S2_C[1] * C3_C[1] + 24.0f / 625.0f * C1_C[1] * C2_C[1] * S3_C[1] + 27.0f / 500.0f * C1_C[1] * S2_C[1];
	_J[1][1][1] = 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * S3_C[1] + S1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * C3_C[1] - S1_C[1] * C2_C[1] * S3_C[1]) * S4_C[1] + 24.0f / 625.0f * S1_C[1] * C2_C[1] * C3_C[1] - 24.0f / 625.0f * S1_C[1] * S2_C[1] * S3_C[1] + 27.0f / 500.0f * S1_C[1] * C2_C[1];
	_J[1][1][2] = 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * S3_C[1] + S1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * C3_C[1] - S1_C[1] * C2_C[1] * S3_C[1]) * S4_C[1] - 24.0f / 625.0f * S1_C[1] * S2_C[1] * S3_C[1] + 24.0f / 625.0f * S1_C[1] * C2_C[1] * C3_C[1];
	_J[1][1][3] = -257.0f / 10000.0f * (S1_C[1] * S2_C[1] * C3_C[1] + S1_C[1] * C2_C[1] * S3_C[1]) * S4_C[1] + 257.0f / 10000.0f * (-S1_C[1] * S2_C[1] * S3_C[1] + S1_C[1] * C2_C[1] * C3_C[1]) * C4_C[1];

	_J[1][2][0] = 0;
	_J[1][2][1] = 257.0f / 10000.0f * (-C2_C[1] * S3_C[1] - S2_C[1] * C3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (S2_C[1] * S3_C[1] - C2_C[1] * C3_C[1]) * S4_C[1] - 24.0f / 625.0f * S2_C[1] * C3_C[1] - 24.0f / 625.0f * C2_C[1] * S3_C[1] - 27.0f / 500.0f * S2_C[1];
	_J[1][2][2] = 257.0f / 10000.0f * (-C2_C[1] * S3_C[1] - S2_C[1] * C3_C[1]) * C4_C[1] + 257.0f / 10000.0f * (S2_C[1] * S3_C[1] - C2_C[1] * C3_C[1]) * S4_C[1] - 24.0f / 625.0f * C2_C[1] * S3_C[1] - 24.0f / 625.0f * S2_C[1] * C3_C[1];
	_J[1][2][3] = -257.0f / 10000.0f * (C2_C[1] * C3_C[1] - S2_C[1] * S3_C[1]) * S4_C[1] + 257.0f / 10000.0f * (-C2_C[1] * S3_C[1] - S2_C[1] * C3_C[1]) * C4_C[1];

	_J[2][0][0] = 257.0f / 10000.0f * (-S1_C[2] * S2_C[2] * C3_C[2] - S1_C[2] * C2_C[2] * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (S1_C[2] * S2_C[2] * S3_C[2] - S1_C[2] * C2_C[2] * C3_C[2]) * S4_C[2] - 24.0f / 625.0f * S1_C[2] * S2_C[2] * C3_C[2] - 24.0f / 625.0f * S1_C[2] * C2_C[2] * S3_C[2] - 27.0f / 500.0f * S1_C[2] * S2_C[2];
	_J[2][0][1] = 257.0f / 10000.0f * (-C1_C[2] * S2_C[2] * S3_C[2] + C1_C[2] * C2_C[2] * C3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-C1_C[2] * C2_C[2] * S3_C[2] - C1_C[2] * S2_C[2] * C3_C[2]) * S4_C[2] + 24.0f / 625.0f * C1_C[2] * C2_C[2] * C3_C[2] - 24.0f / 625.0f * C1_C[2] * S2_C[2] * S3_C[2] + 27.0f / 500.0f * C1_C[2] * C2_C[2];
	_J[2][0][2] = 257.0f / 10000.0f * (-C1_C[2] * S2_C[2] * S3_C[2] + C1_C[2] * C2_C[2] * C3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-C1_C[2] * C2_C[2] * S3_C[2] - C1_C[2] * S2_C[2] * C3_C[2]) * S4_C[2] - 24.0f / 625.0f * C1_C[2] * S2_C[2] * S3_C[2] + 24.0f / 625.0f * C1_C[2] * C2_C[2] * C3_C[2];
	_J[2][0][3] = -257.0f / 10000.0f * (C1_C[2] * S2_C[2] * C3_C[2] + C1_C[2] * C2_C[2] * S3_C[2]) * S4_C[2] + 257.0f / 10000.0f * (-C1_C[2] * S2_C[2] * S3_C[2] + C1_C[2] * C2_C[2] * C3_C[2]) * C4_C[2];

	_J[2][1][0] = 257.0f / 10000.0f * (4981.0f / 5000.0f * C1_C[2] * S2_C[2] * C3_C[2] + 4981.0f / 5000.0f * C1_C[2] * C2_C[2] * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-4981.0f / 5000.0f * C1_C[2] * S2_C[2] * S3_C[2] + 4981.0f / 5000.0f * C1_C[2] * C2_C[2] * C3_C[2]) * S4_C[2] + 14943.0f / 390625.0f * C1_C[2] * S2_C[2] * C3_C[2] + 14943.0f / 390625.0f * C1_C[2] * C2_C[2] * S3_C[2] + 134487.0f / 2500000.0f * C1_C[2] * S2_C[2];
	_J[2][1][1] = 257.0f / 10000.0f * ((4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2] + (-4981.0f / 5000.0f * S1_C[2] * S2_C[2] + 871.0f / 10000.0f * C2_C[2]) * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * S3_C[2] + (-4981.0f / 5000.0f * S1_C[2] * S2_C[2] + 871.0f / 10000.0f * C2_C[2]) * C3_C[2]) * S4_C[2] + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2] + 24.0f / 625.0f * (-4981.0f / 5000.0f * S1_C[2] * S2_C[2] + 871.0f / 10000.0f * C2_C[2]) * S3_C[2] + 134487.0f / 2500000.0f * S1_C[2] * C2_C[2] + 23517.0f / 5000000.0f * S2_C[2];
	_J[2][1][2] = 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * S3_C[2] + (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * C3_C[2] - (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * S3_C[2]) * S4_C[2] - 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * S3_C[2] + 24.0f / 625.0f * (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2];
	_J[2][1][3] = -257.0f / 10000.0f * ((4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * C3_C[2] + (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * S3_C[2]) * S4_C[2] + 257.0f / 10000.0f * (-(4981.0f / 5000.0f * S1_C[2] * S2_C[2] - 871.0f / 10000.0f * C2_C[2]) * S3_C[2] + (4981.0f / 5000.0f * S1_C[2] * C2_C[2] + 871.0f / 10000.0f * S2_C[2]) * C3_C[2]) * C4_C[2];

	_J[2][2][0] = 257.0f / 10000.0f * (871.0f / 10000.0f * C1_C[2] * S2_C[2] * C3_C[2] + 871.0f / 10000.0f * C1_C[2] * C2_C[2] * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-871.0f / 10000.0f * C1_C[2] * S2_C[2] * S3_C[2] + 871.0f / 10000.0f * C1_C[2] * C2_C[2] * C3_C[2]) * S4_C[2] + 2613.0f / 781250.0f * C1_C[2] * S2_C[2] * C3_C[2] + 2613.0f / 781250.0f * C1_C[2] * C2_C[2] * S3_C[2] + 23517.0f / 5000000.0f * C1_C[2] * S2_C[2];
	_J[2][2][1] = 257.0f / 10000.0f * ((871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2] + (-871.0f / 10000.0f * S1_C[2] * S2_C[2] - 4981.0f / 5000.0f * C2_C[2]) * S3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-(871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2] + (-871.0f / 10000.0f * S1_C[2] * S2_C[2] - 4981.0f / 5000.0f * C2_C[2]) * C3_C[2]) * S4_C[2] + 24.0f / 625.0f * (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2] + 24.0f / 625.0f * (-871.0f / 10000.0f * S1_C[2] * S2_C[2] - 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] + 23517.0f / 5000000.0f * S1_C[2] * C2_C[2] - 134487.0f / 2500000.0f * S2_C[2];
	_J[2][2][2] = 257.0f / 10000.0f * (-(871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] + (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2]) * C4_C[2] + 257.0f / 10000.0f * (-(871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * C3_C[2] - (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2]) * S4_C[2] - 24.0f / 625.0f * (871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] + 24 / 625.0f * (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2];
	_J[2][2][3] = -257.0f / 10000.0f * ((871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * C3_C[2] + (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * S3_C[2]) * S4_C[2] + 257.0f / 10000.0f * (-(871.0f / 10000.0f * S1_C[2] * S2_C[2] + 4981.0f / 5000.0f * C2_C[2]) * S3_C[2] + (871.0f / 10000.0f * S1_C[2] * C2_C[2] - 4981.0f / 5000.0f * S2_C[2]) * C3_C[2]) * C4_C[2];

	_J[3][0][0] = (137.0f * C1_C[3]) / 2500.0f - S1_C[3] / 200.0f + (257.0f * C1_C[3] * C3_C[3]) / 5000.0f + (37.0f * C34_C[3] * C1_C[3]) / 1000.0f - (257.0f * S1_C[3] * S2_C[3] * S3_C[3]) / 5000.0f - (37.0f * S34_C[3] * S1_C[3] * S2_C[3]) / 1000.0f;
	_J[3][0][1] = (37.0f * S34_C[3] * C1_C[3] * C2_C[3]) / 1000.0f + (257.0f * C1_C[3] * C2_C[3] * S3_C[3]) / 5000.0f;
	_J[3][0][2] = (37.0f * C34_C[3] * C1_C[3] * S2_C[3]) / 1000.0f - (37.0f * S34_C[3] * S1_C[3]) / 1000.0f - (257.0f * S1_C[3] * S3_C[3]) / 5000.0f + (257.0f * C1_C[3] * C3_C[3] * S2_C[3]) / 5000.0f;
	_J[3][0][3] = (37.0f * C34_C[3] * C1_C[3] * S2_C[3]) / 1000.0f - (37.0f * S34_C[3] * S1_C[3]) / 1000.0f;

	if (_handType == eHandType_Left)
	{
		_J[3][1][0] = (4981.0f * C1_C[3]) / 1000000.0f + (682397.0f * S1_C[3]) / 12500000.0f + (1280117.0f * C3_C[3] * S1_C[3]) / 25000000.0f + (184297.0f * C34_C[3] * S1_C[3]) / 5000000.0f + (184297.0f * S34_C[3] * C1_C[3] * S2_C[3]) / 5000000.0f + (1280117.0f * C1_C[3] * S2_C[3] * S3_C[3]) / 25000000.0f;
		_J[3][1][1] = (223847.0f * S2_C[3] * S3_C[3]) / 50000000.0f + (32227.0f * S34_C[3] * S2_C[3]) / 10000000.0f + (184297.0f * S34_C[3] * C2_C[3] * S1_C[3]) / 5000000.0f + (1280117.0f * C2_C[3] * S1_C[3] * S3_C[3]) / 25000000.0f;
		_J[3][1][2] = -(223847.0f * C2_C[3] * C3_C[3]) / 50000000.0f + (1280117.0f * C1_C[3] * S3_C[3]) / 25000000.0f - (32227.0f * C34_C[3] * C2_C[3]) / 10000000.0f + (184297.0f * S34_C[3] * C1_C[3]) / 5000000.0f + (184297.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 5000000.0f + (1280117.0f * C3_C[3] * S1_C[3] * S2_C[3]) / 25000000.0f;
		_J[3][1][3] = -(32227.0f * C34_C[3] * C2_C[3]) / 10000000.0f + (184297.0f * S34_C[3] * C1_C[3]) / 5000000.0f + (184297.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 5000000.0f;
	}
	else {
		_J[3][1][0] = -(4981.0f * C1_C[3]) / 1000000.0f - (682397.0f * S1_C[3]) / 12500000.0f - (1280117.0f * C3_C[3] * S1_C[3]) / 25000000.0f - (184297.0f * C34_C[3] * S1_C[3]) / 5000000.0f - (184297.0f * S34_C[3] * C1_C[3] * S2_C[3]) / 5000000.0f - (1280117.0f * C1_C[3] * S2_C[3] * S3_C[3]) / 25000000.0f;
		_J[3][1][1] = -(223847.0f * S2_C[3] * S3_C[3]) / 50000000.0f - (32227.0f * S34_C[3] * S2_C[3]) / 10000000.0f - (184297.0f * S34_C[3] * C2_C[3] * S1_C[3]) / 5000000.0f - (1280117.0f * C2_C[3] * S1_C[3] * S3_C[3]) / 25000000.0f;
		_J[3][1][2] = (223847.0f * C2_C[3] * C3_C[3]) / 50000000.0f - (1280117.0f * C1_C[3] * S3_C[3]) / 25000000.0f + (32227.0f * C34_C[3] * C2_C[3]) / 10000000.0f - (184297.0f * S34_C[3] * C1_C[3]) / 5000000.0f - (184297.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 5000000.0f - (1280117.0f * C3_C[3] * S1_C[3] * S2_C[3]) / 25000000.0f;
		_J[3][1][3] = (32227.0f * C34_C[3] * C2_C[3]) / 10000000.0f - (184297.0f * S34_C[3] * C1_C[3]) / 5000000.0f - (184297.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 5000000.0f;
	}

	_J[3][2][0] = (871.0f * C1_C[3]) / 2000000.0f + (119327.0f * S1_C[3]) / 25000000.0f + (223847.0f * C3_C[3] * S1_C[3]) / 50000000.0f + (32227.0f * C34_C[3] * S1_C[3]) / 10000000.0f + (32227.0f * S34_C[3] * C1_C[3] * S2_C[3]) / 10000000.0f + (223847.0f * C1_C[3] * S2_C[3] * S3_C[3]) / 50000000.0f;
	_J[3][2][1] = (32227.0f * S34_C[3] * C2_C[3] * S1_C[3]) / 10000000.0f - (184297.0f * S34_C[3] * S2_C[3]) / 5000000.0f - (1280117.0f * S2_C[3] * S3_C[3]) / 25000000.0f + (223847.0f * C2_C[3] * S1_C[3] * S3_C[3]) / 50000000.0f;
	_J[3][2][2] = (1280117.0f * C2_C[3] * C3_C[3]) / 25000000.0f + (223847.0f * C1_C[3] * S3_C[3]) / 50000000.0f + (184297.0f * C34_C[3] * C2_C[3]) / 5000000.0f + (32227.0f * S34_C[3] * C1_C[3]) / 10000000.0f + (32227.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 10000000.0f + (223847.0f * C3_C[3] * S1_C[3] * S2_C[3]) / 50000000.0f;
	_J[3][2][3] = (184297.0f * C34_C[3] * C2_C[3]) / 5000000.0f + (32227.0f * S34_C[3] * C1_C[3]) / 10000000.0f + (32227.0f * C34_C[3] * S1_C[3] * S2_C[3]) / 10000000.0f;
}

void BHand::CalculateGravity()
{
	double g = 9.81f;
	double mass1[4] = { _mass[0][0], _mass[1][0], _mass[2][0], _mass[3][0] };
	double mass2[4] = { _mass[0][1], _mass[1][1], _mass[2][1], _mass[3][1] };
	double mass3[4] = { _mass[0][2], _mass[1][2], _mass[2][2], _mass[3][2] };
	double mass4[4] = { _mass[0][3], _mass[1][3], _mass[2][3], _mass[3][3] };

	_G[0][0] = 0;
	_G[0][1] = -27.0f * g * mass2[0] * S2_C[0] + g * mass3[0] * (-25.886f * S23_C[0] + 0.137f * C23_C[0] - 54.0f * S2_C[0]) + g * mass4[0] * (-20.571f * S234_C[0] + 0.193f * C234_C[0] - 38.4f * S23_C[0] - 54.0f * S2_C[0]);
	_G[0][2] = g * mass3[0] * (-25.886f * S23_C[0] + 0.137f * C23_C[0]) + g * mass4[0] * (-20.571f * S234_C[0] + 0.193f * C234_C[0] - 38.4f * S23_C[0]);
	_G[0][3] = g * mass4[0] * (-20.571f * S234_C[0] + 0.193f * C234_C[0]);

	_G[1][0] = 0;
	_G[1][1] = -27.0f * g * mass2[1] * S2_C[1] + g * mass3[1] * (-25.886f * S23_C[1] + 0.137f * C23_C[1] - 54.0f * S2_C[1]) + g * mass4[1] * (-20.571f * S234_C[1] + 0.193f * C234_C[1] - 38.4f * S23_C[1] - 54.0f * S2_C[1]);
	_G[1][2] = g * mass3[1] * (-25.886f * S23_C[1] + 0.137f * C23_C[1]) + g * mass4[1] * (-20.571f * S234_C[1] + 0.193f * C234_C[1] - 38.4f * S23_C[1]);
	_G[1][3] = g * mass4[1] * (-20.571f * S234_C[1] + 0.193f * C234_C[1]);

	_G[2][0] = g * mass1[2] * (-0.049473f * C1_C[2] - 0.005139f * S1_C[2]) + 2.3517f * g * mass2[2] * C1_C[2] * S2_C[2] + 0.0871f * g * mass3[2] * C1_C[2] * (25.886f * S23_C[2] - 0.137f * C23_C[2] + 54.0f * S2_C[2]) + g * mass4[2] * (0.0871f * C1_C[2] * (20.571f * S234_C[2] - 0.193f * C234_C[2] + 38.4f * S23_C[2] + 54.0f * S2_C[2]) - 0.001742f * S1_C[2]);
	_G[2][1] = g * mass2[2] * (2.3517f * S1_C[2] * C2_C[2] - 26.8974f * S2_C[2]) + g * mass3[2] * (0.0871f * S1_C[2] * (25.886f * C23_C[2] + 0.137f * S23_C[2] + 54.0f * C2_C[2]) - 25.78763f * S23_C[2] + 0.136479f * C23_C[2] - 53.7948f * S2_C[2]) + g * mass4[2] * (0.0871f * S1_C[2] * (20.571f * C234_C[2] + 0.193f * S234_C[2] + 38.4f * C23_C[2] + 54.0f * C2_C[2]) - 20.49283f * S234_C[2] + 0.192267f * C234_C[2] - 38.25408f * S23_C[2] - 53.7948f * S2_C[2]);
	_G[2][2] = g * mass3[2] * (0.0871f * S1_C[2] * (25.886f * C23_C[2] + 0.137f * S23_C[2]) - 25.78763f * S23_C[2] + 0.136479f * C23_C[2]) + g * mass4[2] * (0.0871f * S1_C[2] * (20.571f * C234_C[2] + 0.193f * S234_C[2] + 38.4f * C23_C[2]) - 20.49283f * S234_C[2] + 0.192267f * C234_C[2] - 38.25408f * S23_C[2]);
	_G[2][3] = g * mass4[2] * (0.0871f * S1_C[2] * (20.571f * C234_C[2] + 0.193f * S234_C[2]) - 20.49283f * S234_C[2] + 0.192267f * C234_C[2]);

	_G[3][0] = (g * mass1[3] * (2110433.0f * C1_C[3] + 2516319.0f * S1_C[3])) / 2500000.0f + g * mass4[3] * ((871.0f * C1_C[3] * (257.0f * S2_C[3] * S3_C[3] - 685.0f * C34_C[3] * S2_C[3] + 62570.0f * S34_C[3] * S2_C[3] + 25.0f)) / 50000000.0f + (871 * S1_C[3] * (62570.0f * C34_C[3] + 685.0f * S34_C[3] + 257.0f * C3_C[3] + 274.0f)) / 50000000.0f) + (g * mass2[3] * (43550.0f * C1_C[3] + 51107667.0f * S1_C[3] - 513890.0f * C1_C[3] * C2_C[3] - 4947280.0f * C1_C[3] * S2_C[3])) / 100000000.0f + g * mass3[3] * ((871.0f * S1_C[3] * (62570.0f * C3_C[3] + 685.0f * S3_C[3] + 274.0f)) / 50000000.0f + (871.0f * C1_C[3] * (12514.0f * S2_C[3] * S3_C[3] - 137.0f * C3_C[3] * S2_C[3] + 5.0f)) / 10000000.0f);
	_G[3][1] = (g * mass2[3] * (5877580.0f * C2_C[3] + 56584160.0f * S2_C[3] - 4947280.0f * C2_C[3] * S1_C[3] + 513890.0f * S1_C[3] * S2_C[3])) / 100000000.0f - g * mass4[3] * ((1280117.0f * S2_C[3] * S3_C[3]) / 25000000.0f - (871.0f * S1_C[3] * (257.0f * C2_C[3] * S3_C[3] - 685.0f * C34_C[3] * C2_C[3] + 62570.0f * S34_C[3] * C2_C[3])) / 50000000.0f - (682397.0f * C34_C[3] * S2_C[3]) / 5000000.0f + (31166117.0f * S34_C[3] * S2_C[3]) / 2500000.0f) - g * mass3[3] * ((31166117.0f * S2_C[3] * S3_C[3]) / 2500000.0f - (682397.0f * C3_C[3] * S2_C[3]) / 5000000.0f + (871.0f * S1_C[3] * (137.0f * C2_C[3] * C3_C[3] - 12514.0f * C2_C[3] * S3_C[3])) / 10000000.0f);
	_G[3][2] = g * mass3[3] * ((31166117.0f * C2_C[3] * C3_C[3]) / 2500000.0f - (871.0f * C1_C[3] * (685.0f * C3_C[3] - 62570.0f * S3_C[3])) / 50000000.0f + (682397.0f * C2_C[3] * S3_C[3]) / 5000000.0f + (871.0f * S1_C[3] * (12514.0f * C3_C[3] * S2_C[3] + 137.0f * S2_C[3] * S3_C[3])) / 10000000.0f) + g * mass4[3] * ((1280117.0f * C2_C[3] * C3_C[3]) / 25000000.0f + (871.0f * S1_C[3] * (257.0f * C3_C[3] * S2_C[3] + 62570.0f * C34_C[3] * S2_C[3] + 685.0f * S34_C[3] * S2_C[3])) / 50000000.0f + (31166117.0f * C34_C[3] * C2_C[3]) / 2500000.0f + (682397.0f * S34_C[3] * C2_C[3]) / 5000000.0f + (871.0f * C1_C[3] * (62570.0f * S34_C[3] - 685.0f * C34_C[3] + 257.0f * S3_C[3])) / 50000000.0f);
	_G[3][3] = g * mass4[3] * ((871.0f * S1_C[3] * (62570.0f * C34_C[3] * S2_C[3] + 685.0f * S34_C[3] * S2_C[3])) / 50000000.0f - (871.0f * C1_C[3] * (685.0f * C34_C[3] - 62570.0f * S34_C[3])) / 50000000.0f + (31166117.0f * C34_C[3] * C2_C[3]) / 2500000.0f + (682397.0f * S34_C[3] * C2_C[3]) / 5000000.0f);

	for (int i = 0; i < NOF; i++)
	{
		_G[i][0] = _G[i][0] * 0.001f;
		_G[i][1] = _G[i][1] * 0.001f;
		_G[i][2] = _G[i][2] * 0.001f;
		_G[i][3] = _G[i][3] * 0.001f;
	}
}

void BHand::CalculateGravityEx()
{
	// 글로벌 좌표계는 y축이 엄지방향, 나머지 손가락이 z방향, 손바닥이 x 방향

	double g = 9.81f;
	double mass1[4] = { _mass[0][0], _mass[0][1], _mass[0][2], _mass[0][3] };  // 검지
	double mass2[4] = { _mass[1][0], _mass[1][1], _mass[1][2], _mass[1][3] };  // 중지
	double mass3[4] = { _mass[2][0], _mass[2][1], _mass[2][2], _mass[2][3] };  // 약지
	double mass4[4] = { _mass[3][0], _mass[3][1], _mass[3][2], _mass[3][3] };  // 엄지
	for (int i = 0; i < 4; i++) {
		mass1[i] *= 1.5;
		mass2[i] *= 1.5;
		mass3[i] *= 1.5;
		mass4[i] *= 1.5;
	}


	double length1[4] = { 0.0f, 0.0462f, 0.0307f, 0.0157f * 1.2f }; // 검지의 링크길이들.
	double length2[4] = { 0.0f, 0.0462f, 0.0307f, 0.0157f * 1.2f }; // 중지의 링크길이들.
	double length3[4] = { 0.0f, 0.0462f, 0.0307f, 0.0157f * 1.2f }; // 약지의 링크길이들.
	double length4[4] = { 0.0646f, 0.0f, 0.0307f, 0.0313f * 1.2f }; // 엄지의 링크길이들.

	/// 참고: 로보틱스랩 상에서의 링크길이들 
	// 검지, 중지, 약지, length1= 0; length2=0.0538, length3=0.0307, length4=0.0157;
	// 엄지, length1= , length2= ; length 3= ; length 4 = 


	// 센서에서 로테이션 메트릭스를 바로 받는지, 각도값으로 받을지 결정해야함. 

	double Transpose_Rot3[3][3];
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			Transpose_Rot3[j][i] = _R[i * 3 + j];
	}

	/////////////////////////////// 중지의 중력보상 토크 계산 ///////////////////////////////////////
	double J2V2[3][4]; // 중지의 2번 조인트의 글로벌자코비안	
	double J2V3[3][4]; // 중지의 3번 조인트의 글로벌자코비안
	double J2V4[3][4]; // 중지의 4번 조인트의 글로벌자코비안

	J2V2[0][0] = -0.5 * length2[1] * S1_C[1] * S2_C[1];
	J2V2[0][1] = length2[1] * C1_C[1] * C2_C[1] / 2;
	J2V2[0][2] = 0.0;
	J2V2[0][3] = 0.0;
	J2V2[1][0] = 0.5 * length2[1] * C1_C[1] * S2_C[1];
	J2V2[1][1] = 0.5 * length2[1] * S1_C[1] * C2_C[1];
	J2V2[1][2] = 0.0;
	J2V2[1][3] = 0.0;
	J2V2[2][0] = 0.0;
	J2V2[2][1] = -length2[1] / 2 * S2_C[1];
	J2V2[2][2] = 0.0;
	J2V2[2][3] = 0.0;

	double tmp = -(length2[2] * S23_C[1]) / 2;
	double tmp2 = -length2[1] * S2_C[1];
	J2V3[0][0] = -length2[1] * S1_C[1] * S2_C[1] - length2[2] * S1_C[1] * S23_C[1] / 2;
	J2V3[0][1] = length2[1] * C1_C[1] * C2_C[1] + length2[2] * C1_C[1] * C23_C[1] / 2;
	J2V3[0][2] = length2[2] * C1_C[1] * C23_C[1] / 2;
	J2V3[0][3] = 0.0;
	J2V3[1][0] = length2[1] * C1_C[1] * S2_C[1] + 0.5 * length2[2] * C1_C[1] * S23_C[1];
	J2V3[1][1] = length2[1] * S1_C[1] * C2_C[1] + 0.5 * length2[2] * S1_C[1] * C23_C[1];
	J2V3[1][2] = 0.5 * length2[2] * S1_C[1] * C23_C[1];
	J2V3[1][3] = 0.0;
	J2V3[2][0] = 0.0;
	J2V3[2][1] = -length2[1] * S2_C[1] - length2[2] * S23_C[1] / 2;
	J2V3[2][2] = -length2[2] * S23_C[1] / 2;
	J2V3[2][3] = 0.0;

	J2V4[0][0] = -length2[1] * S1_C[1] * S2_C[1] - length2[2] * S1_C[1] * S23_C[1] - length2[3] * S1_C[1] * S234_C[1] * 0.5;
	J2V4[0][1] = length2[1] * C1_C[1] * C2_C[1] + length2[2] * C1_C[1] * C23_C[1] + length2[3] * C1_C[1] * C234_C[1] * 0.5;
	J2V4[0][2] = length2[2] * C1_C[1] * C23_C[1] + length2[3] * C1_C[1] * C234_C[1] * 0.5;
	J2V4[0][3] = length2[3] * C1_C[1] * C234_C[1] * 0.5;
	J2V4[1][0] = length2[1] * C1_C[1] * S2_C[1] + length2[2] * C1_C[1] * S23_C[1] + length2[3] * 0.5 * C1_C[1] * S234_C[1];
	J2V4[1][1] = length2[1] * S1_C[1] * C2_C[1] + length2[2] * S1_C[1] * C23_C[1] + length2[3] * S1_C[1] * C234_C[1] * 0.5;
	J2V4[1][2] = length2[2] * S1_C[1] * C23_C[1] + length2[3] * S1_C[1] * C234_C[1] * 0.5;
	J2V4[1][3] = length2[3] * S1_C[1] * C234_C[1] * 0.5;
	J2V4[2][0] = 0.0;
	J2V4[2][1] = -length2[1] * S2_C[1] - length2[2] * S23_C[1] - length2[3] * S234_C[1] * 0.5;
	J2V4[2][2] = -length2[2] * S23_C[1] - length2[3] * 0.5 * S234_C[1];
	J2V4[2][3] = -length2[3] * 0.5 * S234_C[1];

	double G2[3][3]; // 중지의 글로벌 중력
	G2[0][0] = 0.0;
	G2[0][1] = 0.0;
	G2[0][2] = mass2[1] * g; // 중지의 두 번째 조인트의 무게
	G2[1][0] = 0.0;
	G2[1][1] = 0.0;
	G2[1][2] = mass2[2] * g;
	G2[2][0] = 0.0;
	G2[2][1] = 0.0;
	G2[2][2] = mass2[3] * g;

	double Transpose_Rot[3][3];

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			Transpose_Rot[i][j] = Transpose_Rot3[i][j];

	_G[1][0] = J2V2[0][0] * Transpose_Rot[0][2] * G2[0][2] + J2V3[0][0] * Transpose_Rot[0][2] * G2[1][2] + J2V4[0][0] * Transpose_Rot[0][2] * G2[2][2] + J2V2[1][0] * Transpose_Rot[1][2] * G2[0][2] + J2V3[1][0] * Transpose_Rot[1][2] * G2[1][2] + J2V4[1][0] * Transpose_Rot[1][2] * G2[2][2] + J2V2[2][0] * Transpose_Rot[2][2] * G2[0][2] + J2V3[2][0] * Transpose_Rot[2][2] * G2[1][2] + J2V4[2][0] * Transpose_Rot[2][2] * G2[2][2];
	_G[1][1] = J2V2[0][1] * Transpose_Rot[0][2] * G2[0][2] + J2V3[0][1] * Transpose_Rot[0][2] * G2[1][2] + J2V4[0][1] * Transpose_Rot[0][2] * G2[2][2] + J2V2[1][1] * Transpose_Rot[1][2] * G2[0][2] + J2V3[1][1] * Transpose_Rot[1][2] * G2[1][2] + J2V4[1][1] * Transpose_Rot[1][2] * G2[2][2] + J2V2[2][1] * Transpose_Rot[2][2] * G2[0][2] + J2V3[2][1] * Transpose_Rot[2][2] * G2[1][2] + J2V4[2][1] * Transpose_Rot[2][2] * G2[2][2];
	_G[1][2] = J2V2[0][2] * Transpose_Rot[0][2] * G2[0][2] + J2V3[0][2] * Transpose_Rot[0][2] * G2[1][2] + J2V4[0][2] * Transpose_Rot[0][2] * G2[2][2] + J2V2[1][2] * Transpose_Rot[1][2] * G2[0][2] + J2V3[1][2] * Transpose_Rot[1][2] * G2[1][2] + J2V4[1][2] * Transpose_Rot[1][2] * G2[2][2] + J2V2[2][2] * Transpose_Rot[2][2] * G2[0][2] + J2V3[2][2] * Transpose_Rot[2][2] * G2[1][2] + J2V4[2][2] * Transpose_Rot[2][2] * G2[2][2];
	_G[1][3] = J2V2[0][3] * Transpose_Rot[0][2] * G2[0][2] + J2V3[0][3] * Transpose_Rot[0][2] * G2[1][2] + J2V4[0][3] * Transpose_Rot[0][2] * G2[2][2] + J2V2[1][3] * Transpose_Rot[1][2] * G2[0][2] + J2V3[1][3] * Transpose_Rot[1][2] * G2[1][2] + J2V4[1][3] * Transpose_Rot[1][2] * G2[2][2] + J2V2[2][3] * Transpose_Rot[2][2] * G2[0][2] + J2V3[2][3] * Transpose_Rot[2][2] * G2[1][2] + J2V4[2][3] * Transpose_Rot[2][2] * G2[2][2];

	/////////////////////////////// 검지의 중력보상 토크 계산 ///////////////////////////////////////
	double J1V2[3][4]; // 검지의 2번 조인트의 글로벌자코비안	
	double J1V3[3][4]; // 검지의 3번 조인트의 글로벌자코비안
	double J1V4[3][4]; // 검지의 4번 조인트의 글로벌자코비안

	J1V2[0][0] = -0.5 * length1[1] * S1_C[0] * S2_C[0];
	J1V2[0][1] = length1[1] * C1_C[0] * C2_C[0] / 2;
	J1V2[0][2] = 0.0;
	J1V2[0][3] = 0.0;
	J1V2[1][0] = 0.5 * length1[1] * C1_C[0] * S2_C[0];
	J1V2[1][1] = 0.5 * length1[1] * S1_C[0] * C2_C[0];
	J1V2[1][2] = 0.0;
	J1V2[1][3] = 0.0;
	J1V2[2][0] = 0.0;
	J1V2[2][1] = -length1[1] / 2 * S2_C[0];
	J1V2[2][2] = 0.0;
	J1V2[2][3] = 0.0;

	J1V3[0][0] = -length1[1] * S1_C[0] * S2_C[0] - length1[2] * S1_C[0] * S23_C[0] / 2;
	J1V3[0][1] = length1[1] * C1_C[0] * C2_C[0] + length1[2] * C1_C[0] * C23_C[0] / 2;
	J1V3[0][2] = length1[2] * C1_C[0] * C23_C[0] / 2;
	J1V3[0][3] = 0.0;
	J1V3[1][0] = length1[1] * C1_C[0] * S2_C[0] + 0.5 * length1[2] * C1_C[0] * S23_C[0];
	J1V3[1][1] = length1[1] * S1_C[0] * C2_C[0] + 0.5 * length1[2] * S1_C[0] * C23_C[0];
	J1V3[1][2] = 0.5 * length1[2] * S1_C[0] * C23_C[0];
	J1V3[1][3] = 0.0;
	J1V3[2][0] = 0.0;
	J1V3[2][1] = -length1[1] * S2_C[0] - length1[2] * S23_C[0] / 2;
	J1V3[2][2] = -length1[2] * S23_C[0] / 2;
	J1V3[2][3] = 0.0;

	J1V4[0][0] = -length1[1] * S1_C[0] * S2_C[0] - length1[2] * S1_C[0] * S23_C[0] - length1[3] * S1_C[0] * S234_C[0] * 0.5;
	J1V4[0][1] = length1[1] * C1_C[0] * C2_C[0] + length1[2] * C1_C[0] * C23_C[0] + length1[3] * C1_C[0] * C234_C[0] * 0.5;
	J1V4[0][2] = length1[2] * C1_C[0] * C23_C[0] + length1[3] * C1_C[0] * C234_C[0] * 0.5;
	J1V4[0][3] = length1[3] * C1_C[0] * C234_C[0] * 0.5;
	J1V4[1][0] = length1[1] * C1_C[0] * S2_C[0] + length1[2] * C1_C[0] * S23_C[0] + length1[3] * 0.5 * C1_C[0] * S234_C[0];
	J1V4[1][1] = length1[1] * S1_C[0] * C2_C[0] + length1[2] * S1_C[0] * C23_C[0] + length1[3] * S1_C[0] * C234_C[0] * 0.5;
	J1V4[1][2] = length1[2] * S1_C[0] * C23_C[0] + length1[3] * S1_C[0] * C234_C[0] * 0.5;
	J1V4[1][3] = length1[3] * S1_C[0] * C234_C[0] * 0.5;
	J1V4[2][0] = 0.0;
	J1V4[2][1] = -length1[1] * S2_C[0] - length1[2] * S23_C[0] - length1[3] * S234_C[0] * 0.5;
	J1V4[2][2] = -length1[2] * S23_C[0] - length1[3] * 0.5 * S234_C[0];
	J1V4[2][3] = -length1[3] * 0.5 * S234_C[0];


	double G1[3][3]; // 검지의 글로벌 중력
	G1[0][0] = 0.0;
	G1[0][1] = 0.0;
	G1[0][2] = mass1[1] * g; // 검지의 두 번째 조인트의 무게
	G1[1][0] = 0.0;
	G1[1][1] = 0.0;
	G1[1][2] = mass1[2] * g;
	G1[2][0] = 0.0;
	G1[2][1] = 0.0;
	G1[2][2] = mass1[3] * g;

	double tilted_angle = 5 * DEG2RAD; // 5도 비틀림
	double Transpose_Rot2[3][3];


	// 센서에서 로테이션 메트릭스를 바로 받는지, 각도값으로 받을지 결정해야함. 

	//double Transpose_Rot[3][3];

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++) {
			Transpose_Rot2[i][j] = Transpose_Rot3[i][j];
			Transpose_Rot[i][j] = Transpose_Rot3[i][j];
		}
	}


	Transpose_Rot[0][1] = Transpose_Rot2[0][1] * cos(tilted_angle) + Transpose_Rot2[0][2] * sin(tilted_angle);
	Transpose_Rot[0][2] = -1.0 * Transpose_Rot2[0][1] * sin(tilted_angle) + Transpose_Rot2[0][2] * cos(tilted_angle);
	Transpose_Rot[1][1] = Transpose_Rot2[1][1] * cos(tilted_angle) + Transpose_Rot2[1][2] * sin(tilted_angle);
	Transpose_Rot[1][2] = -1.0 * Transpose_Rot2[1][1] * sin(tilted_angle) + Transpose_Rot2[1][2] * cos(tilted_angle);
	Transpose_Rot[2][1] = Transpose_Rot2[2][1] * cos(tilted_angle) + Transpose_Rot2[2][2] * sin(tilted_angle);
	Transpose_Rot[2][2] = -1.0 * Transpose_Rot2[2][1] * sin(tilted_angle) + Transpose_Rot2[2][2] * cos(tilted_angle);


	_G[0][0] = J1V2[0][0] * Transpose_Rot[0][2] * G1[0][2] + J1V3[0][0] * Transpose_Rot[0][2] * G1[1][2] + J1V4[0][0] * Transpose_Rot[0][2] * G1[2][2] + J1V2[1][0] * Transpose_Rot[1][2] * G1[0][2] + J1V3[1][0] * Transpose_Rot[1][2] * G1[1][2] + J1V4[1][0] * Transpose_Rot[1][2] * G1[2][2] + J1V2[2][0] * Transpose_Rot[2][2] * G1[0][2] + J1V3[2][0] * Transpose_Rot[2][2] * G1[1][2] + J1V4[2][0] * Transpose_Rot[2][2] * G1[2][2];
	_G[0][1] = J1V2[0][1] * Transpose_Rot[0][2] * G1[0][2] + J1V3[0][1] * Transpose_Rot[0][2] * G1[1][2] + J1V4[0][1] * Transpose_Rot[0][2] * G1[2][2] + J1V2[1][1] * Transpose_Rot[1][2] * G1[0][2] + J1V3[1][1] * Transpose_Rot[1][2] * G1[1][2] + J1V4[1][1] * Transpose_Rot[1][2] * G1[2][2] + J1V2[2][1] * Transpose_Rot[2][2] * G1[0][2] + J1V3[2][1] * Transpose_Rot[2][2] * G1[1][2] + J1V4[2][1] * Transpose_Rot[2][2] * G1[2][2];
	_G[0][2] = J1V2[0][2] * Transpose_Rot[0][2] * G1[0][2] + J1V3[0][2] * Transpose_Rot[0][2] * G1[1][2] + J1V4[0][2] * Transpose_Rot[0][2] * G1[2][2] + J1V2[1][2] * Transpose_Rot[1][2] * G1[0][2] + J1V3[1][2] * Transpose_Rot[1][2] * G1[1][2] + J1V4[1][2] * Transpose_Rot[1][2] * G1[2][2] + J1V2[2][2] * Transpose_Rot[2][2] * G1[0][2] + J1V3[2][2] * Transpose_Rot[2][2] * G1[1][2] + J1V4[2][2] * Transpose_Rot[2][2] * G1[2][2];
	_G[0][3] = J1V2[0][3] * Transpose_Rot[0][2] * G1[0][2] + J1V3[0][3] * Transpose_Rot[0][2] * G1[1][2] + J1V4[0][3] * Transpose_Rot[0][2] * G1[2][2] + J1V2[1][3] * Transpose_Rot[1][2] * G1[0][2] + J1V3[1][3] * Transpose_Rot[1][2] * G1[1][2] + J1V4[1][3] * Transpose_Rot[1][2] * G1[2][2] + J1V2[2][3] * Transpose_Rot[2][2] * G1[0][2] + J1V3[2][3] * Transpose_Rot[2][2] * G1[1][2] + J1V4[2][3] * Transpose_Rot[2][2] * G1[2][2];

	/////////////////////////////// 약지의 중력보상 토크 계산 ///////////////////////////////////////
	double J3V2[3][4]; // 약지의 2번 조인트의 글로벌자코비안	
	double J3V3[3][4]; // 약지의 3번 조인트의 글로벌자코비안
	double J3V4[3][4]; // 약지의 4번 조인트의 글로벌자코비안

	J3V2[0][0] = -0.5 * length3[1] * S1_C[2] * S2_C[2];
	J3V2[0][1] = length3[1] * C1_C[2] * C2_C[2] / 2;
	J3V2[0][2] = 0.0;
	J3V2[0][3] = 0.0;
	J3V2[1][0] = 0.5 * length3[1] * C1_C[2] * S2_C[2];
	J3V2[1][1] = 0.5 * length3[1] * S1_C[2] * C2_C[2];
	J3V2[1][2] = 0.0;
	J3V2[1][3] = 0.0;
	J3V2[2][0] = 0.0;
	J3V2[2][1] = -length3[1] / 2 * S2_C[2];
	J3V2[2][2] = 0.0;
	J3V2[2][3] = 0.0;

	J3V3[0][0] = -length3[1] * S1_C[2] * S2_C[2] - length3[2] * S1_C[2] * S23_C[2] / 2;
	J3V3[0][1] = length3[1] * C1_C[2] * C2_C[2] + length3[2] * C1_C[2] * C23_C[2] / 2;
	J3V3[0][2] = length3[2] * C1_C[2] * C23_C[2] / 2;
	J3V3[0][3] = 0.0;
	J3V3[1][0] = length3[1] * C1_C[2] * S2_C[2] + 0.5 * length3[2] * C1_C[2] * S23_C[2];
	J3V3[1][1] = length3[1] * S1_C[2] * C2_C[2] + 0.5 * length3[2] * S1_C[2] * C23_C[2];
	J3V3[1][2] = 0.5 * length3[2] * S1_C[2] * C23_C[2];
	J3V3[1][3] = 0.0;
	J3V3[2][0] = 0.0;
	J3V3[2][1] = -length3[1] * S2_C[2] - length3[2] * S23_C[2] / 2;
	J3V3[2][2] = -length3[2] * S23_C[2] / 2;
	J3V3[2][3] = 0.0;

	J3V4[0][0] = -length3[1] * S1_C[2] * S2_C[2] - length3[2] * S1_C[2] * S23_C[2] - length3[3] * S1_C[2] * S234_C[2] * 0.5;
	J3V4[0][1] = length3[1] * C1_C[2] * C2_C[2] + length3[2] * C1_C[2] * C23_C[2] + length3[3] * C1_C[2] * C234_C[2] * 0.5;
	J3V4[0][2] = length3[2] * C1_C[2] * C23_C[2] + length3[3] * C1_C[2] * C234_C[2] * 0.5;
	J3V4[0][3] = length3[3] * C1_C[2] * C234_C[2] * 0.5;
	J3V4[1][0] = length3[1] * C1_C[2] * S2_C[2] + length3[2] * C1_C[2] * S23_C[2] + length3[3] * 0.5 * C1_C[2] * S234_C[2];
	J3V4[1][1] = length3[1] * S1_C[2] * C2_C[2] + length3[2] * S1_C[2] * C23_C[2] + length3[3] * S1_C[2] * C234_C[2] * 0.5;
	J3V4[1][2] = length3[2] * S1_C[2] * C23_C[2] + length3[3] * S1_C[2] * C234_C[2] * 0.5;
	J3V4[1][3] = length3[3] * S1_C[2] * C234_C[2] * 0.5;
	J3V4[2][0] = 0.0;
	J3V4[2][1] = -length3[1] * S2_C[2] - length3[2] * S23_C[2] - length3[3] * S234_C[2] * 0.5;
	J3V4[2][2] = -length3[2] * S23_C[2] - length3[3] * 0.5 * S234_C[2];
	J3V4[2][3] = -length3[3] * 0.5 * S234_C[2];


	double G3[3][3]; // 검지의 글로벌 중력
	G3[0][0] = 0.0;
	G3[0][1] = 0.0;
	G3[0][2] = mass3[1] * g; // 검지의 두 번째 조인트의 무게
	G3[1][0] = 0.0;
	G3[1][1] = 0.0;
	G3[1][2] = mass3[2] * g;
	G3[2][0] = 0.0;
	G3[2][1] = 0.0;
	G3[2][2] = mass3[3] * g;

	tilted_angle = -1.0 * 5 * DEG2RAD; // -5도 비틀림


	// 센서에서 로테이션 메트릭스를 바로 받는지, 각도값으로 받을지 결정해야함. 

	//double Transpose_Rot[3][3];
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++) {
			Transpose_Rot2[i][j] = Transpose_Rot3[i][j];
			Transpose_Rot[i][j] = Transpose_Rot3[i][j];
		}
	}

	Transpose_Rot[0][1] = Transpose_Rot2[0][1] * cos(tilted_angle) + Transpose_Rot2[0][2] * sin(tilted_angle);
	Transpose_Rot[0][2] = -1.0 * Transpose_Rot2[0][1] * sin(tilted_angle) + Transpose_Rot2[0][2] * cos(tilted_angle);
	Transpose_Rot[1][1] = Transpose_Rot2[1][1] * cos(tilted_angle) + Transpose_Rot2[1][2] * sin(tilted_angle);
	Transpose_Rot[1][2] = -1.0 * Transpose_Rot2[1][1] * sin(tilted_angle) + Transpose_Rot2[1][2] * cos(tilted_angle);
	Transpose_Rot[2][1] = Transpose_Rot2[2][1] * cos(tilted_angle) + Transpose_Rot2[2][2] * sin(tilted_angle);
	Transpose_Rot[2][2] = -1.0 * Transpose_Rot2[2][1] * sin(tilted_angle) + Transpose_Rot2[2][2] * cos(tilted_angle);

	_G[2][0] = J3V2[0][0] * Transpose_Rot[0][2] * G3[0][2] + J3V3[0][0] * Transpose_Rot[0][2] * G3[1][2] + J3V4[0][0] * Transpose_Rot[0][2] * G3[2][2] + J3V2[1][0] * Transpose_Rot[1][2] * G3[0][2] + J3V3[1][0] * Transpose_Rot[1][2] * G3[1][2] + J3V4[1][0] * Transpose_Rot[1][2] * G3[2][2] + J3V2[2][0] * Transpose_Rot[2][2] * G3[0][2] + J3V3[2][0] * Transpose_Rot[2][2] * G3[1][2] + J3V4[2][0] * Transpose_Rot[2][2] * G3[2][2];
	_G[2][1] = J3V2[0][1] * Transpose_Rot[0][2] * G3[0][2] + J3V3[0][1] * Transpose_Rot[0][2] * G3[1][2] + J3V4[0][1] * Transpose_Rot[0][2] * G3[2][2] + J3V2[1][1] * Transpose_Rot[1][2] * G3[0][2] + J3V3[1][1] * Transpose_Rot[1][2] * G3[1][2] + J3V4[1][1] * Transpose_Rot[1][2] * G3[2][2] + J3V2[2][1] * Transpose_Rot[2][2] * G3[0][2] + J3V3[2][1] * Transpose_Rot[2][2] * G3[1][2] + J3V4[2][1] * Transpose_Rot[2][2] * G3[2][2];
	_G[2][2] = J3V2[0][2] * Transpose_Rot[0][2] * G3[0][2] + J3V3[0][2] * Transpose_Rot[0][2] * G3[1][2] + J3V4[0][2] * Transpose_Rot[0][2] * G3[2][2] + J3V2[1][2] * Transpose_Rot[1][2] * G3[0][2] + J3V3[1][2] * Transpose_Rot[1][2] * G3[1][2] + J3V4[1][2] * Transpose_Rot[1][2] * G3[2][2] + J3V2[2][2] * Transpose_Rot[2][2] * G3[0][2] + J3V3[2][2] * Transpose_Rot[2][2] * G3[1][2] + J3V4[2][2] * Transpose_Rot[2][2] * G3[2][2];
	_G[2][3] = J3V2[0][3] * Transpose_Rot[0][2] * G3[0][2] + J3V3[0][3] * Transpose_Rot[0][2] * G3[1][2] + J3V4[0][3] * Transpose_Rot[0][2] * G3[2][2] + J3V2[1][3] * Transpose_Rot[1][2] * G3[0][2] + J3V3[1][3] * Transpose_Rot[1][2] * G3[1][2] + J3V4[1][3] * Transpose_Rot[1][2] * G3[2][2] + J3V2[2][3] * Transpose_Rot[2][2] * G3[0][2] + J3V3[2][3] * Transpose_Rot[2][2] * G3[1][2] + J3V4[2][3] * Transpose_Rot[2][2] * G3[2][2];

	/////////////////////////////// 엄지의 중력보상 토크 계산 ///////////////////////////////////////
	double J4V1[3][4]; // 엄지의 2번 조인트의 글로벌자코비안	
	double J4V3[3][4]; // 엄지의 3번 조인트의 글로벌자코비안
	double J4V4[3][4]; // 엄지의 4번 조인트의 글로벌자코비안

	J4V1[0][0] = 1.0 * (length4[0] * 0.5 * C1_C[3]);
	J4V1[0][1] = 0.0;
	J4V1[0][2] = 0.0;
	J4V1[0][3] = 0.0;
	J4V1[1][0] = -1.0 * (length4[0] * S1_C[3]) * 0.5;
	J4V1[1][1] = 0.0;
	J4V1[1][2] = 0.0;
	J4V1[1][3] = 0.0;
	J4V1[2][0] = 0.0;
	J4V1[2][1] = 0.0;
	J4V1[2][2] = 0.0;
	J4V1[2][3] = 0.0;

	J4V3[0][0] = length4[0] * C1_C[3] - 0.5 * length4[2] * S1_C[3] * S2_C[3] * S3_C[3] + 0.5 * length4[2] * C1_C[3] * C3_C[3];
	J4V3[0][1] = 0.5 * length4[2] * C1_C[3] * C2_C[3] * S3_C[3];
	J4V3[0][2] = 0.5 * length4[2] * C1_C[3] * S2_C[3] * C3_C[3] - 0.5 * length4[2] * S1_C[3] * S3_C[3];
	J4V3[0][3] = 0.0;
	J4V3[1][0] = -length4[0] * S1_C[3] - 0.5 * length4[2] * C1_C[3] * S2_C[3] * S3_C[3] - 0.5 * length4[2] * S1_C[3] * C3_C[3];
	J4V3[1][1] = -0.5 * length4[2] * S1_C[3] * C2_C[3] * S3_C[3];
	J4V3[1][2] = -0.5 * length4[2] * S1_C[3] * S2_C[3] * C3_C[3] - 0.5 * length4[2] * C1_C[3] * S3_C[3];
	J4V3[1][3] = 0.0;
	J4V3[2][0] = 0.0;
	J4V3[2][1] = -0.5 * length4[2] * S2_C[3] * S3_C[3];
	J4V3[2][2] = 0.5 * length4[2] * C2_C[3] * C3_C[3];
	J4V3[2][3] = 0.0;

	J4V4[0][0] = length4[0] * C1_C[3] - length4[2] * S1_C[3] * S2_C[3] * S3_C[3] + length4[2] * C1_C[3] * C3_C[3] - length4[3] * 0.5 * S1_C[3] * C2_C[3] * S34_C[3] + 0.5 * length4[3] * C1_C[3] * C34_C[3];
	J4V4[0][1] = length4[2] * C1_C[3] * C2_C[3] * S3_C[3] + 0.5 * length4[3] * C1_C[3] * C2_C[3] * S34_C[3];
	J4V4[0][2] = length4[2] * C1_C[3] * S2_C[3] * C3_C[3] - length4[2] * S1_C[3] * S3_C[3] + 0.5 * length4[3] * C1_C[3] * S2_C[3] * C34_C[3] - 0.5 * length4[3] * S1_C[3] * S34_C[3];
	J4V4[0][3] = 0.5 * length4[3] * C1_C[3] * S2_C[3] * C34_C[3] - 0.5 * length4[3] * S1_C[3] * S34_C[3];
	J4V4[1][0] = -length4[0] * S1_C[3] - length4[2] * C1_C[3] * S2_C[3] * S3_C[3] - length4[2] * S1_C[3] * C3_C[3] - 0.5 * length4[3] * C1_C[3] * S2_C[3] * C34_C[3] - 0.5 * length4[3] * S1_C[3] * C34_C[3];
	J4V4[1][1] = -length4[2] * S1_C[3] * C2_C[3] * S3_C[3] - 0.5 * length4[3] * S1_C[3] * C2_C[3] * S34_C[3];
	J4V4[1][2] = -length4[2] * S1_C[3] * S2_C[3] * C3_C[3] - length4[2] * C1_C[3] * S3_C[3] - 0.5 * length4[3] * S1_C[3] * S2_C[3] * C34_C[3] - 0.5 * length4[3] * C1_C[3] * S34_C[3];
	J4V4[1][3] = -0.5 * length4[3] * S1_C[3] * S2_C[3] * C34_C[3] - 0.5 * length4[3] * C1_C[3] * S34_C[3];
	J4V4[2][0] = 0.0;
	J4V4[2][1] = -length4[2] * S2_C[3] * S3_C[3] - 0.5 * length4[3] * S2_C[3] * S34_C[3];
	J4V4[2][2] = length4[2] * C2_C[3] * C3_C[3] + length4[3] * 0.5 * C2_C[3] * C34_C[3];
	J4V4[2][3] = length4[3] * 0.5 * C2_C[3] * C34_C[3];

	double G4[3][3]; // 엄지의 글로벌 중력
	G4[0][0] = 0.0;
	G4[0][1] = 0.0;
	G4[0][2] = mass4[0] * g; // 엄지의 첫 번째 조인트의 무게
	G4[1][0] = 0.0;
	G4[1][1] = 0.0;
	G4[1][2] = mass4[2] * g;
	G4[2][0] = 0.0;
	G4[2][1] = 0.0;
	G4[2][2] = mass4[3] * g;

	tilted_angle = 1.0 * 5.0 * DEG2RAD; // -5도 비틀림	

	// 센서에서 로테이션 메트릭스를 바로 받는지, 각도값으로 받을지 결정해야함. 

	//double Transpose_Rot[3][3];
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++) {
			Transpose_Rot2[i][j] = Transpose_Rot3[i][j];
			Transpose_Rot[i][j] = Transpose_Rot3[i][j];
		}
	}

	Transpose_Rot[0][1] = Transpose_Rot2[0][1] * cos(tilted_angle) + Transpose_Rot2[0][2] * sin(tilted_angle);
	Transpose_Rot[0][2] = -1.0 * Transpose_Rot2[0][1] * sin(tilted_angle) + Transpose_Rot2[0][2] * cos(tilted_angle);
	Transpose_Rot[1][1] = Transpose_Rot2[1][1] * cos(tilted_angle) + Transpose_Rot2[1][2] * sin(tilted_angle);
	Transpose_Rot[1][2] = -1.0 * Transpose_Rot2[1][1] * sin(tilted_angle) + Transpose_Rot2[1][2] * cos(tilted_angle);
	Transpose_Rot[2][1] = Transpose_Rot2[2][1] * cos(tilted_angle) + Transpose_Rot2[2][2] * sin(tilted_angle);
	Transpose_Rot[2][2] = -1.0 * Transpose_Rot2[2][1] * sin(tilted_angle) + Transpose_Rot2[2][2] * cos(tilted_angle);

	_G[3][0] = J4V1[0][0] * Transpose_Rot[0][2] * G4[0][2] + J4V3[0][0] * Transpose_Rot[0][2] * G4[1][2] + J4V4[0][0] * Transpose_Rot[0][2] * G4[2][2] + J4V1[1][0] * Transpose_Rot[1][2] * G4[0][2] + J4V3[1][0] * Transpose_Rot[1][2] * G4[1][2] + J4V4[1][0] * Transpose_Rot[1][2] * G4[2][2] + J4V1[2][0] * Transpose_Rot[2][2] * G4[0][2] + J4V3[2][0] * Transpose_Rot[2][2] * G4[1][2] + J4V4[2][0] * Transpose_Rot[2][2] * G4[2][2];
	_G[3][1] = J4V1[0][1] * Transpose_Rot[0][2] * G4[0][2] + J4V3[0][1] * Transpose_Rot[0][2] * G4[1][2] + J4V4[0][1] * Transpose_Rot[0][2] * G4[2][2] + J4V1[1][1] * Transpose_Rot[1][2] * G4[0][2] + J4V3[1][1] * Transpose_Rot[1][2] * G4[1][2] + J4V4[1][1] * Transpose_Rot[1][2] * G4[2][2] + J4V1[2][1] * Transpose_Rot[2][2] * G4[0][2] + J4V3[2][1] * Transpose_Rot[2][2] * G4[1][2] + J4V4[2][1] * Transpose_Rot[2][2] * G4[2][2];
	_G[3][2] = J4V1[0][2] * Transpose_Rot[0][2] * G4[0][2] + J4V3[0][2] * Transpose_Rot[0][2] * G4[1][2] + J4V4[0][2] * Transpose_Rot[0][2] * G4[2][2] + J4V1[1][2] * Transpose_Rot[1][2] * G4[0][2] + J4V3[1][2] * Transpose_Rot[1][2] * G4[1][2] + J4V4[1][2] * Transpose_Rot[1][2] * G4[2][2] + J4V1[2][2] * Transpose_Rot[2][2] * G4[0][2] + J4V3[2][2] * Transpose_Rot[2][2] * G4[1][2] + J4V4[2][2] * Transpose_Rot[2][2] * G4[2][2];
	_G[3][3] = J4V1[0][3] * Transpose_Rot[0][2] * G4[0][2] + J4V3[0][3] * Transpose_Rot[0][2] * G4[1][2] + J4V4[0][3] * Transpose_Rot[0][2] * G4[2][2] + J4V1[1][3] * Transpose_Rot[1][2] * G4[0][2] + J4V3[1][3] * Transpose_Rot[1][2] * G4[1][2] + J4V4[1][3] * Transpose_Rot[1][2] * G4[2][2] + J4V1[2][3] * Transpose_Rot[2][2] * G4[0][2] + J4V3[2][3] * Transpose_Rot[2][2] * G4[1][2] + J4V4[2][3] * Transpose_Rot[2][2] * G4[2][2];
}

void BHand::GetFKResult(double x[4], double y[4], double z[4])
{
	memcpy(x, _x, sizeof(_x[0]) * NOF);
	memcpy(y, _y, sizeof(_y[0]) * NOF);
	memcpy(z, _z, sizeof(_z[0]) * NOF);
}

void BHand::SetJointDesiredPosition(double* q)
{
	memcpy(_q_des, q, SIZEOF_VARRAY);
}

void BHand::SetGainsEx(double* kp, double* kd)
{
	memcpy(_kp, kp, SIZEOF_VARRAY);
	memcpy(_kd, kd, SIZEOF_VARRAY);
}

void BHand::SetGraspingForce(double f[4])
{
	memcpy(_f_des, f, SIZEOF_VARRAY);
}
void BHand::SetEnvelopTorqueScalar(double set_scalar)
{
	//set_scalar default is 1.0. Set in the header.
	_envelop_torque_scalar = set_scalar;
}




//UNUSED
void BHand::SetObjectDisp(double x_d_o[3])
{
	x_move = x_d_o[0];
	y_move = x_d_o[1];
	z_move = x_d_o[2];

	//printf("\n\nmade it in!\n\n");
}

void BHand::MoveFingerTip(double set_xyz_0[3], double set_xyz_1[3], double set_xyz_2[3], double set_xyz_3[3])
{
	//void BHand::Motion_Ready()
	//{
	_set_x_des[0] = set_xyz_0[0];
	_set_y_des[0] = set_xyz_0[1];
	_set_z_des[0] = set_xyz_0[2];

	_set_x_des[1] = set_xyz_1[0];
	_set_y_des[1] = set_xyz_1[1];
	_set_z_des[1] = set_xyz_1[2];

	_set_x_des[2] = set_xyz_2[0];
	_set_y_des[2] = set_xyz_2[1];
	_set_z_des[2] = set_xyz_2[2];

	_set_x_des[3] = set_xyz_3[0];
	_set_y_des[3] = set_xyz_3[1];
	_set_z_des[3] = set_xyz_3[2];


	/*
	_x_des[0] = _x[0];
	_y_des[0] = _y[0];
	_z_des[0] = _z[0];

	_x_des[1] = _x[1];
	_y_des[1] = _y[1];
	_z_des[1] = _z[1];

	_x_des[2] = _x[2];
	_y_des[2] = _y[2];
	_z_des[2] = _z[2];

	_x_des[3] = _x[3];
	_y_des[3] = _y[3];
	_z_des[3] = _z[3];
	*/
	//}
}

void BHand::GetGraspingForce(double fx[4], double fy[4], double fz[4])
{
	int i;
	double e_x[4];
	double e_y[4];
	double e_z[4];
	double e_norm = 1.0;

	for (i = 0; i < NOF; i++)
	{
		e_x[i] = (_x_des[i] - _x[i]);
		e_y[i] = (_y_des[i] - _y[i]);
		e_z[i] = (_z_des[i] - _z[i]);
		e_norm = sqrt(e_x[i] * e_x[i] + e_y[i] * e_y[i] + e_z[i] * e_z[i]);

		fx[i] = _f_des[i] * e_x[i] / e_norm;
		fy[i] = _f_des[i] * e_y[i] / e_norm;
		fz[i] = _f_des[i] * e_z[i] / e_norm;
	}
}

void BHand::SetOrientation(double roll, double pitch, double yaw)
{
	memset(_R, 0, sizeof(_R[0]) * 9);
	_R[0] = _R[4] = _R[8] = 1.0;
}

void BHand::SetOrientation(double R[9])
{
	memcpy(_R, R, sizeof(_R[0]) * 9);
}

BHAND_EXTERN_C_BEGIN

BHand* bhCreateLeftHand()
{
	return new BHand(eHandType_Left);
}

BHand* bhCreateRightHand()
{
	return new BHand(eHandType_Right);
}

BHAND_EXTERN_C_END
