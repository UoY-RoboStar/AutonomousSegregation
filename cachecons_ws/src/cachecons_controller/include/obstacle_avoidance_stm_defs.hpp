#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wpedantic"

#ifndef OBSTACLE_AVOIDANCE_STM_DEFS_H
#define OBSTACLE_AVOIDANCE_STM_DEFS_H

#define MAX_SEQ_SIZE 32
#define MAXQUEUE 32

typedef struct {
	float values[2];
} vector_float_2;

/* Representation of enum STATUS */

typedef enum {
	STATUS_ENTER_STATE,
	STATUS_ENTER_CHILDREN,
	STATUS_EXECUTE_STATE,
	STATUS_EXIT_CHILDREN,
	STATUS_EXIT_STATE,
	STATUS_INACTIVE,
} STATUS_Type;

typedef union {
} STATUS_Data;

typedef struct {
	STATUS_Type type;
	STATUS_Data data;
} STATUS_Enum;

STATUS_Enum create_STATUS_ENTER_STATE() {
	STATUS_Data data;

	STATUS_Type type = STATUS_ENTER_STATE;
	
	STATUS_Enum aux = (STATUS_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
STATUS_Enum create_STATUS_ENTER_CHILDREN() {
	STATUS_Data data;

	STATUS_Type type = STATUS_ENTER_CHILDREN;
	
	STATUS_Enum aux = (STATUS_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
STATUS_Enum create_STATUS_EXECUTE_STATE() {
	STATUS_Data data;

	STATUS_Type type = STATUS_EXECUTE_STATE;
	
	STATUS_Enum aux = (STATUS_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
STATUS_Enum create_STATUS_EXIT_CHILDREN() {
	STATUS_Data data;

	STATUS_Type type = STATUS_EXIT_CHILDREN;
	
	STATUS_Enum aux = (STATUS_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
STATUS_Enum create_STATUS_EXIT_STATE() {
	STATUS_Data data;

	STATUS_Type type = STATUS_EXIT_STATE;
	
	STATUS_Enum aux = (STATUS_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
STATUS_Enum create_STATUS_INACTIVE() {
	STATUS_Data data;

	STATUS_Type type = STATUS_INACTIVE;
	
	STATUS_Enum aux = (STATUS_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
/* Representation of enum RESULT */

typedef enum {
	RESULT_WAIT,
	RESULT_CONT,
} RESULT_Type;

typedef union {
} RESULT_Data;

typedef struct {
	RESULT_Type type;
	RESULT_Data data;
} RESULT_Enum;

RESULT_Enum create_RESULT_WAIT() {
	RESULT_Data data;

	RESULT_Type type = RESULT_WAIT;
	
	RESULT_Enum aux = (RESULT_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
RESULT_Enum create_RESULT_CONT() {
	RESULT_Data data;

	RESULT_Type type = RESULT_CONT;
	
	RESULT_Enum aux = (RESULT_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
/* Representation of enum STATES_stm_ref1 */

typedef enum {
	STATES_stm_ref1_NONE,
	STATES_stm_ref1_VHFEnabled,
	STATES_stm_ref1_VHFDisabled,
} STATES_stm_ref1_Type;

typedef union {
} STATES_stm_ref1_Data;

typedef struct {
	STATES_stm_ref1_Type type;
	STATES_stm_ref1_Data data;
} STATES_stm_ref1_Enum;

STATES_stm_ref1_Enum create_STATES_stm_ref1_NONE() {
	STATES_stm_ref1_Data data;

	STATES_stm_ref1_Type type = STATES_stm_ref1_NONE;
	
	STATES_stm_ref1_Enum aux = (STATES_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
STATES_stm_ref1_Enum create_STATES_stm_ref1_VHFEnabled() {
	STATES_stm_ref1_Data data;

	STATES_stm_ref1_Type type = STATES_stm_ref1_VHFEnabled;
	
	STATES_stm_ref1_Enum aux = (STATES_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
STATES_stm_ref1_Enum create_STATES_stm_ref1_VHFDisabled() {
	STATES_stm_ref1_Data data;

	STATES_stm_ref1_Type type = STATES_stm_ref1_VHFDisabled;
	
	STATES_stm_ref1_Enum aux = (STATES_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
/* Representation of enum TRANSITIONS_stm_ref1 */

typedef enum {
	TRANSITIONS_stm_ref1_NONE,
	TRANSITIONS_stm_ref1_stm_ref1_t11,
	TRANSITIONS_stm_ref1_stm_ref1_t3,
	TRANSITIONS_stm_ref1_stm_ref1_t8,
	TRANSITIONS_stm_ref1_stm_ref1_t10,
	TRANSITIONS_stm_ref1_stm_ref1_t1,
	TRANSITIONS_stm_ref1_stm_ref1_t2,
	TRANSITIONS_stm_ref1_stm_ref1_t5,
	TRANSITIONS_stm_ref1_stm_ref1_t13,
	TRANSITIONS_stm_ref1_stm_ref1_t0,
	TRANSITIONS_stm_ref1_stm_ref1_t6,
	TRANSITIONS_stm_ref1_stm_ref1_t9,
	TRANSITIONS_stm_ref1_stm_ref1_t4,
	TRANSITIONS_stm_ref1_stm_ref1_t12,
	TRANSITIONS_stm_ref1_stm_ref1_t7,
} TRANSITIONS_stm_ref1_Type;

typedef union {
} TRANSITIONS_stm_ref1_Data;

typedef struct {
	TRANSITIONS_stm_ref1_Type type;
	TRANSITIONS_stm_ref1_Data data;
} TRANSITIONS_stm_ref1_Enum;

TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_NONE() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_NONE;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t11() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t11;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t3() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t3;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t8() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t8;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t10() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t10;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t1() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t1;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t2() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t2;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t5() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t5;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t13() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t13;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t0() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t0;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t6() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t6;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t9() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t9;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t4() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t4;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t12() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t12;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
TRANSITIONS_stm_ref1_Enum create_TRANSITIONS_stm_ref1_stm_ref1_t7() {
	TRANSITIONS_stm_ref1_Data data;

	TRANSITIONS_stm_ref1_Type type = TRANSITIONS_stm_ref1_stm_ref1_t7;
	
	TRANSITIONS_stm_ref1_Enum aux = (TRANSITIONS_stm_ref1_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
/* Representation of enum M_CacheConsM_input */

typedef enum {
	M_CacheConsM_input_closestAngle,
	M_CacheConsM_input_EnableOA,
	M_CacheConsM_input_closestDistance,
	M_CacheConsM_input_DisableOA,
	M_CacheConsM_input_NOAMove,
	M_CacheConsM_input__done_,
	M_CacheConsM_input__terminate_,
} M_CacheConsM_input_Type;

typedef struct {
	float v1;
} M_CacheConsM_input_closestAngle_Data;

typedef struct {
	float v1;
} M_CacheConsM_input_closestDistance_Data;

typedef struct {
	vector_float_2 v1;
} M_CacheConsM_input_NOAMove_Data;

typedef union {	
	M_CacheConsM_input_closestAngle_Data closestAngle;
	M_CacheConsM_input_closestDistance_Data closestDistance;
	M_CacheConsM_input_NOAMove_Data NOAMove;
} M_CacheConsM_input_Data;

typedef struct {
	M_CacheConsM_input_Type type;
	M_CacheConsM_input_Data data;
} M_CacheConsM_input_Enum;

M_CacheConsM_input_Enum create_M_CacheConsM_input_closestAngle(float v1) {
	M_CacheConsM_input_Data data;
		
	data.closestAngle.v1 = v1;	

	M_CacheConsM_input_Type type = M_CacheConsM_input_closestAngle;
	
	M_CacheConsM_input_Enum aux = (M_CacheConsM_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
M_CacheConsM_input_Enum create_M_CacheConsM_input_EnableOA() {
	M_CacheConsM_input_Data data;

	M_CacheConsM_input_Type type = M_CacheConsM_input_EnableOA;
	
	M_CacheConsM_input_Enum aux = (M_CacheConsM_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
M_CacheConsM_input_Enum create_M_CacheConsM_input_closestDistance(float v1) {
	M_CacheConsM_input_Data data;
		
	data.closestDistance.v1 = v1;	

	M_CacheConsM_input_Type type = M_CacheConsM_input_closestDistance;
	
	M_CacheConsM_input_Enum aux = (M_CacheConsM_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
M_CacheConsM_input_Enum create_M_CacheConsM_input_DisableOA() {
	M_CacheConsM_input_Data data;

	M_CacheConsM_input_Type type = M_CacheConsM_input_DisableOA;
	
	M_CacheConsM_input_Enum aux = (M_CacheConsM_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
M_CacheConsM_input_Enum create_M_CacheConsM_input_NOAMove(vector_float_2 v1) {
	M_CacheConsM_input_Data data;
		
	data.NOAMove.v1.values[0] = v1.values[0];
	data.NOAMove.v1.values[1] = v1.values[1];

	M_CacheConsM_input_Type type = M_CacheConsM_input_NOAMove;
	
	M_CacheConsM_input_Enum aux = (M_CacheConsM_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
M_CacheConsM_input_Enum create_M_CacheConsM_input__done_() {
	M_CacheConsM_input_Data data;

	M_CacheConsM_input_Type type = M_CacheConsM_input__done_;
	
	M_CacheConsM_input_Enum aux = (M_CacheConsM_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
M_CacheConsM_input_Enum create_M_CacheConsM_input__terminate_() {
	M_CacheConsM_input_Data data;

	M_CacheConsM_input_Type type = M_CacheConsM_input__terminate_;
	
	M_CacheConsM_input_Enum aux = (M_CacheConsM_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
/* Representation of enum M_CacheConsM_output */

typedef enum {
	M_CacheConsM_output_OAMove,
	M_CacheConsM_output__done_,
} M_CacheConsM_output_Type;

typedef struct {
	vector_float_2 v1;
} M_CacheConsM_output_OAMove_Data;

typedef union {
	M_CacheConsM_output_OAMove_Data OAMove;
} M_CacheConsM_output_Data;

typedef struct {
	M_CacheConsM_output_Type type;
	M_CacheConsM_output_Data data;
} M_CacheConsM_output_Enum;

M_CacheConsM_output_Enum create_M_CacheConsM_output_OAMove(vector_float_2 v1) {
	M_CacheConsM_output_Data data;
		
	data.OAMove.v1.values[0] = v1.values[0];
	data.OAMove.v1.values[1] = v1.values[1];

	M_CacheConsM_output_Type type = M_CacheConsM_output_OAMove;
	
	M_CacheConsM_output_Enum aux = (M_CacheConsM_output_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
M_CacheConsM_output_Enum create_M_CacheConsM_output__done_() {
	M_CacheConsM_output_Data data;

	M_CacheConsM_output_Type type = M_CacheConsM_output__done_;
	
	M_CacheConsM_output_Enum aux = (M_CacheConsM_output_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
/* Representation of enum C_ctrl_ref0_input */

typedef enum {
	C_ctrl_ref0_input_NOAMove,
	C_ctrl_ref0_input_DisableOA,
	C_ctrl_ref0_input_closestDistance,
	C_ctrl_ref0_input_closestAngle,
	C_ctrl_ref0_input_EnableOA,
	C_ctrl_ref0_input__done_,
	C_ctrl_ref0_input__terminate_,
} C_ctrl_ref0_input_Type;

typedef struct {
	vector_float_2 v1;
} C_ctrl_ref0_input_NOAMove_Data;

typedef struct {
	float v1;
} C_ctrl_ref0_input_closestDistance_Data;

typedef struct {
	float v1;
} C_ctrl_ref0_input_closestAngle_Data;

typedef union {
	C_ctrl_ref0_input_NOAMove_Data NOAMove;
	C_ctrl_ref0_input_closestDistance_Data closestDistance;
	C_ctrl_ref0_input_closestAngle_Data closestAngle;
} C_ctrl_ref0_input_Data;

typedef struct {
	C_ctrl_ref0_input_Type type;
	C_ctrl_ref0_input_Data data;
} C_ctrl_ref0_input_Enum;

C_ctrl_ref0_input_Enum create_C_ctrl_ref0_input_NOAMove(vector_float_2 v1) {
	C_ctrl_ref0_input_Data data;
		
	data.NOAMove.v1.values[0] = v1.values[0];
	data.NOAMove.v1.values[1] = v1.values[1];

	C_ctrl_ref0_input_Type type = C_ctrl_ref0_input_NOAMove;
	
	C_ctrl_ref0_input_Enum aux = (C_ctrl_ref0_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
C_ctrl_ref0_input_Enum create_C_ctrl_ref0_input_DisableOA() {
	C_ctrl_ref0_input_Data data;

	C_ctrl_ref0_input_Type type = C_ctrl_ref0_input_DisableOA;
	
	C_ctrl_ref0_input_Enum aux = (C_ctrl_ref0_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
C_ctrl_ref0_input_Enum create_C_ctrl_ref0_input_closestDistance(float v1) {
	C_ctrl_ref0_input_Data data;
		
	data.closestDistance.v1 = v1;	

	C_ctrl_ref0_input_Type type = C_ctrl_ref0_input_closestDistance;
	
	C_ctrl_ref0_input_Enum aux = (C_ctrl_ref0_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
C_ctrl_ref0_input_Enum create_C_ctrl_ref0_input_closestAngle(float v1) {
	C_ctrl_ref0_input_Data data;
		
	data.closestAngle.v1 = v1;	

	C_ctrl_ref0_input_Type type = C_ctrl_ref0_input_closestAngle;
	
	C_ctrl_ref0_input_Enum aux = (C_ctrl_ref0_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
C_ctrl_ref0_input_Enum create_C_ctrl_ref0_input_EnableOA() {
	C_ctrl_ref0_input_Data data;

	C_ctrl_ref0_input_Type type = C_ctrl_ref0_input_EnableOA;
	
	C_ctrl_ref0_input_Enum aux = (C_ctrl_ref0_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
C_ctrl_ref0_input_Enum create_C_ctrl_ref0_input__done_() {
	C_ctrl_ref0_input_Data data;

	C_ctrl_ref0_input_Type type = C_ctrl_ref0_input__done_;
	
	C_ctrl_ref0_input_Enum aux = (C_ctrl_ref0_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
C_ctrl_ref0_input_Enum create_C_ctrl_ref0_input__terminate_() {
	C_ctrl_ref0_input_Data data;

	C_ctrl_ref0_input_Type type = C_ctrl_ref0_input__terminate_;
	
	C_ctrl_ref0_input_Enum aux = (C_ctrl_ref0_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
/* Representation of enum stm_ref1_input */

typedef enum {
	stm_ref1_input_DisableOA,
	stm_ref1_input_closestDistance,
	stm_ref1_input_EnableOA,
	stm_ref1_input_closestAngle,
	stm_ref1_input_NOAMove,
	stm_ref1_input__done_,
	stm_ref1_input__terminate_,
} stm_ref1_input_Type;

typedef struct {
	float v1;
} stm_ref1_input_closestDistance_Data;

typedef struct {
	float v1;
} stm_ref1_input_closestAngle_Data;

typedef struct {
	vector_float_2 v1;
} stm_ref1_input_NOAMove_Data;

typedef union {
	stm_ref1_input_closestDistance_Data closestDistance;
	stm_ref1_input_closestAngle_Data closestAngle;
	stm_ref1_input_NOAMove_Data NOAMove;
} stm_ref1_input_Data;

typedef struct {
	stm_ref1_input_Type type;
	stm_ref1_input_Data data;
} stm_ref1_input_Enum;

stm_ref1_input_Enum create_stm_ref1_input_DisableOA() {
	stm_ref1_input_Data data;

	stm_ref1_input_Type type = stm_ref1_input_DisableOA;
	
	stm_ref1_input_Enum aux = (stm_ref1_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
stm_ref1_input_Enum create_stm_ref1_input_closestDistance(float v1) {
	stm_ref1_input_Data data;
		
	data.closestDistance.v1 = v1;	

	stm_ref1_input_Type type = stm_ref1_input_closestDistance;
	
	stm_ref1_input_Enum aux = (stm_ref1_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
stm_ref1_input_Enum create_stm_ref1_input_EnableOA() {
	stm_ref1_input_Data data;

	stm_ref1_input_Type type = stm_ref1_input_EnableOA;
	
	stm_ref1_input_Enum aux = (stm_ref1_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
stm_ref1_input_Enum create_stm_ref1_input_closestAngle(float v1) {
	stm_ref1_input_Data data;
		
	data.closestAngle.v1 = v1;	

	stm_ref1_input_Type type = stm_ref1_input_closestAngle;
	
	stm_ref1_input_Enum aux = (stm_ref1_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
stm_ref1_input_Enum create_stm_ref1_input_NOAMove(vector_float_2 v1) {
	stm_ref1_input_Data data;
		
	data.NOAMove.v1.values[0] = v1.values[0];
	data.NOAMove.v1.values[1] = v1.values[1];

	stm_ref1_input_Type type = stm_ref1_input_NOAMove;
	
	stm_ref1_input_Enum aux = (stm_ref1_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
stm_ref1_input_Enum create_stm_ref1_input__done_() {
	stm_ref1_input_Data data;

	stm_ref1_input_Type type = stm_ref1_input__done_;
	
	stm_ref1_input_Enum aux = (stm_ref1_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
stm_ref1_input_Enum create_stm_ref1_input__terminate_() {
	stm_ref1_input_Data data;

	stm_ref1_input_Type type = stm_ref1_input__terminate_;
	
	stm_ref1_input_Enum aux = (stm_ref1_input_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
/* Representation of enum stm_ref1_output */

typedef enum {
	stm_ref1_output_OAMove,
	stm_ref1_output__done_,
} stm_ref1_output_Type;

typedef struct {
	vector_float_2 v1;
} stm_ref1_output_OAMove_Data;

typedef union {
	stm_ref1_output_OAMove_Data OAMove;
} stm_ref1_output_Data;

typedef struct {
	stm_ref1_output_Type type;
	stm_ref1_output_Data data;
} stm_ref1_output_Enum;

stm_ref1_output_Enum create_stm_ref1_output_OAMove(vector_float_2 v1) {
	stm_ref1_output_Data data;
		
	data.OAMove.v1.values[0] = v1.values[0];
	data.OAMove.v1.values[1] = v1.values[1];

	stm_ref1_output_Type type = stm_ref1_output_OAMove;
	
	stm_ref1_output_Enum aux = (stm_ref1_output_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
stm_ref1_output_Enum create_stm_ref1_output__done_() {
	stm_ref1_output_Data data;

	stm_ref1_output_Type type = stm_ref1_output__done_;
	
	stm_ref1_output_Enum aux = stm_ref1_output_Enum {
		.type = type,
		.data = data
	};
	
	return aux;	
}
/* Representation of enum C_ctrl_ref0_output */

typedef enum {
	C_ctrl_ref0_output_OAMove,
	C_ctrl_ref0_output__done_,
} C_ctrl_ref0_output_Type;

typedef struct {
	vector_float_2 v1;
} C_ctrl_ref0_output_OAMove_Data;

typedef union {
	C_ctrl_ref0_output_OAMove_Data OAMove;
} C_ctrl_ref0_output_Data;

typedef struct {
	C_ctrl_ref0_output_Type type;
	C_ctrl_ref0_output_Data data;
} C_ctrl_ref0_output_Enum;

C_ctrl_ref0_output_Enum create_C_ctrl_ref0_output_OAMove(vector_float_2 v1) {
	C_ctrl_ref0_output_Data data;
		
	data.OAMove.v1.values[0] = v1.values[0];
	data.OAMove.v1.values[1] = v1.values[1];

	C_ctrl_ref0_output_Type type = C_ctrl_ref0_output_OAMove;
	
	C_ctrl_ref0_output_Enum aux = (C_ctrl_ref0_output_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}
C_ctrl_ref0_output_Enum create_C_ctrl_ref0_output__done_() {
	C_ctrl_ref0_output_Data data;

	C_ctrl_ref0_output_Type type = C_ctrl_ref0_output__done_;
	
	C_ctrl_ref0_output_Enum aux = (C_ctrl_ref0_output_Enum) {
		.type = type,
		.data = data
	};
	
	return aux;	
}

/* Representation of record stm_ref1_inputstate */
struct stm_ref1_inputstate {
	bool DisableOA;
	bool closestDistance;
	float closestDistance_value;
	bool EnableOA;
	bool closestAngle;
	float closestAngle_value;
	bool NOAMove;
	vector_float_2 NOAMove_value;
	int _clock_T;
	TRANSITIONS_stm_ref1_Enum _transition_;
};
/* Representation of record stm_ref1_state */
struct stm_ref1_state {
	bool done;
	STATES_stm_ref1_Enum state;
	STATES_stm_ref1_Enum target_state;
	STATUS_Enum status;
	bool en_ObstacleAvoidance_VHFEnabled_1_done;
	int en_ObstacleAvoidance_VHFEnabled_1_counter;
	bool tr_ObstacleAvoidance_t11_done;
	int tr_ObstacleAvoidance_t11_counter;
	bool tr_ObstacleAvoidance_t3_done;
	int tr_ObstacleAvoidance_t3_counter;
	bool tr_ObstacleAvoidance_t13_done;
	int tr_ObstacleAvoidance_t13_counter;
	bool tr_ObstacleAvoidance_t10_done;
	int tr_ObstacleAvoidance_t10_counter;
	bool tr_ObstacleAvoidance_t0_done;
	int tr_ObstacleAvoidance_t0_counter;
	bool tr_ObstacleAvoidance_t6_done;
	int tr_ObstacleAvoidance_t6_counter;
	bool tr_ObstacleAvoidance_t4_done;
	int tr_ObstacleAvoidance_t4_counter;
	bool tr_ObstacleAvoidance_t12_done;
	int tr_ObstacleAvoidance_t12_counter;
	bool tr_ObstacleAvoidance_t1_done;
	int tr_ObstacleAvoidance_t1_counter;
};
/* Representation of record stm_ref1_memory */
struct stm_ref1_memory {
	float min_range;
	float max_range;
	float pi;
	float current_speed;
	vector_float_2 NOA_Move;
	float DISTANCE;
	float lv;
	float closest_angle;
	float av;
	float closest_distance;
};

#endif

#pragma GCC diagnostic pop