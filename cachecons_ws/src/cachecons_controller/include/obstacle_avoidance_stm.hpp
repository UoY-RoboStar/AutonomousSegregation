#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"

#ifndef OBSTACLE_AVOIDANCE_STM_H
#define OBSTACLE_AVOIDANCE_STM_H

#include "rclcpp/rclcpp.hpp"
#include "obstacle_avoidance_stm_defs.hpp"
#include "cachecons_interfaces/msg/vector_float2.hpp"
#include <math.h>

typedef struct {
	rclcpp::Publisher<cachecons_interfaces::msg::VectorFloat2>::SharedPtr oamove_publisher_;
} stm_ref1_output_Enum_Channel;

class ObstacleAvoidanceStm {
    public:
        ObstacleAvoidanceStm();
        ObstacleAvoidanceStm(stm_ref1_output_Enum_Channel _out);
        void ReadInputs(
            bool EnableOA,
            bool DisableOA,
            bool NOAMove,
            vector_float_2 NOAMove_value,
            bool closestDistance,
            float closestDistance_value,
            bool closestAngle,
            float closestAngle_value
        ) {
            inputstate.EnableOA = EnableOA;
            inputstate.DisableOA = DisableOA;
            inputstate.NOAMove = NOAMove;
            inputstate.NOAMove_value = NOAMove_value;
            inputstate.closestDistance = closestDistance;
            inputstate.closestDistance_value = closestDistance_value;
            inputstate.closestAngle  = closestAngle;
            inputstate.closestAngle_value = closestAngle_value;
        }

        void ResetInputs() {
            inputstate.EnableOA = false;
            inputstate.DisableOA = false;
            inputstate.NOAMove = false;
            inputstate.closestDistance = false;
            inputstate.closestAngle  = false;
        }

        void step();
        RESULT_Enum stm_stm_ref1_step(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum tr_ObstacleAvoidance_t13(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);
	    RESULT_Enum tr_ObstacleAvoidance_t4(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum ObstacleAvoidance_j1(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum tr_ObstacleAvoidance_t6(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum ObstacleAvoidance_j0(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum tr_ObstacleAvoidance_t0(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum ObstacleAvoidance_j3(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum ObstacleAvoidance_j2(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum tr_ObstacleAvoidance_t1(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum en_ObstacleAvoidance_VHFEnabled_1(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum tr_ObstacleAvoidance_t3(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum tr_ObstacleAvoidance_t11(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum tr_ObstacleAvoidance_t10(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);
        RESULT_Enum tr_ObstacleAvoidance_t12(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output);



    int random_sign() {
        int num = rand();
        return num/abs(num);
	}

	float randomcoef() {
        float numerator = (float)(rand());
        float denumerator = (float)(rand());
        if (denumerator < 0.000005 && denumerator > -0.000005)
            return numerator;
		return numerator / denumerator;
		
	}

	float absolute(float n) {
		return abs(n);		
	}

	int randomnat() {
		return abs(rand());
	}

    private:
        stm_ref1_inputstate inputstate;
        stm_ref1_state state;
        struct stm_ref1_memory memorystate;
        stm_ref1_output_Enum_Channel output;
};

#endif

#pragma GCC diagnostic pop