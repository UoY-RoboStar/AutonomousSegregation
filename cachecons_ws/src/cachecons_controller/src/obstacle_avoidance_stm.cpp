#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "obstacle_avoidance_stm.hpp"
#include "obstacle_avoidance_stm_defs.hpp"
#include <cstdio>
#include "rclcpp/rclcpp.hpp"

ObstacleAvoidanceStm::ObstacleAvoidanceStm(stm_ref1_output_Enum_Channel _out) {
    inputstate = stm_ref1_inputstate {
	                                        	.DisableOA = false,
	                                        	.closestDistance = false,
	                                        	.closestDistance_value = 0.0,
	                                        	.EnableOA = false,
	                                        	.closestAngle = false,
	                                        	.closestAngle_value = 0.0,
	                                        	.NOAMove = false,
	                                        	.NOAMove_value = {{0.0,0.0}},
	                                        	._clock_T = 0,
	                                        	._transition_ = create_TRANSITIONS_stm_ref1_NONE()
	                                        };
	state = stm_ref1_state {
	                              	.done = false,
	                              	.state = create_STATES_stm_ref1_NONE(),
	                              	.target_state = create_STATES_stm_ref1_NONE(),
	                              	.status = create_STATUS_ENTER_STATE(),
	                              	.en_ObstacleAvoidance_VHFEnabled_1_done = false,
	                              	.en_ObstacleAvoidance_VHFEnabled_1_counter = 0,
	                              	.tr_ObstacleAvoidance_t11_done = false,
	                              	.tr_ObstacleAvoidance_t11_counter = 0,
	                              	.tr_ObstacleAvoidance_t3_done = false,
	                              	.tr_ObstacleAvoidance_t3_counter = 0,
	                              	.tr_ObstacleAvoidance_t13_done = false,
	                              	.tr_ObstacleAvoidance_t13_counter = 0,
	                              	.tr_ObstacleAvoidance_t10_done = false,
	                              	.tr_ObstacleAvoidance_t10_counter = 0,
	                              	.tr_ObstacleAvoidance_t0_done = false,
	                              	.tr_ObstacleAvoidance_t0_counter = 0,
	                              	.tr_ObstacleAvoidance_t6_done = false,
	                              	.tr_ObstacleAvoidance_t6_counter = 0,
	                              	.tr_ObstacleAvoidance_t4_done = false,
	                              	.tr_ObstacleAvoidance_t4_counter = 0,
	                              	.tr_ObstacleAvoidance_t12_done = false,
	                              	.tr_ObstacleAvoidance_t12_counter = 0,
	                              	.tr_ObstacleAvoidance_t1_done = false,
	                              	.tr_ObstacleAvoidance_t1_counter = 0
	                              };
	memorystate = stm_ref1_memory {
	                                     	.min_range = 0.1,
	                                     	.max_range = 0.4,
	                                     	.pi = 3.14159,
	                                     	.current_speed = 0.0,
	                                     	.NOA_Move = {{0.0,0.0}},
	                                     	.DISTANCE = 0.4,
	                                     	.lv = 0.07,
	                                     	.closest_angle = 0,
	                                     	.av = 0.7,
	                                     	.closest_distance = 0
	                                     };
	output = _out;
}

ObstacleAvoidanceStm::ObstacleAvoidanceStm() {
    inputstate = stm_ref1_inputstate {
	                                        	.DisableOA = false,
	                                        	.closestDistance = false,
	                                        	.closestDistance_value = 0.0,
	                                        	.EnableOA = false,
	                                        	.closestAngle = false,
	                                        	.closestAngle_value = 0.0,
	                                        	.NOAMove = false,
	                                        	.NOAMove_value = {{0.0,0.0}},
	                                        	._clock_T = 0,
	                                        	._transition_ = create_TRANSITIONS_stm_ref1_NONE()
	                                        };
	state = stm_ref1_state {
	                              	.done = false,
	                              	.state = create_STATES_stm_ref1_NONE(),
	                              	.target_state = create_STATES_stm_ref1_NONE(),
	                              	.status = create_STATUS_ENTER_STATE(),
	                              	.en_ObstacleAvoidance_VHFEnabled_1_done = false,
	                              	.en_ObstacleAvoidance_VHFEnabled_1_counter = 0,
	                              	.tr_ObstacleAvoidance_t11_done = false,
	                              	.tr_ObstacleAvoidance_t11_counter = 0,
	                              	.tr_ObstacleAvoidance_t3_done = false,
	                              	.tr_ObstacleAvoidance_t3_counter = 0,
	                              	.tr_ObstacleAvoidance_t13_done = false,
	                              	.tr_ObstacleAvoidance_t13_counter = 0,
	                              	.tr_ObstacleAvoidance_t10_done = false,
	                              	.tr_ObstacleAvoidance_t10_counter = 0,
	                              	.tr_ObstacleAvoidance_t0_done = false,
	                              	.tr_ObstacleAvoidance_t0_counter = 0,
	                              	.tr_ObstacleAvoidance_t6_done = false,
	                              	.tr_ObstacleAvoidance_t6_counter = 0,
	                              	.tr_ObstacleAvoidance_t4_done = false,
	                              	.tr_ObstacleAvoidance_t4_counter = 0,
	                              	.tr_ObstacleAvoidance_t12_done = false,
	                              	.tr_ObstacleAvoidance_t12_counter = 0,
	                              	.tr_ObstacleAvoidance_t1_done = false,
	                              	.tr_ObstacleAvoidance_t1_counter = 0
	                              };
	memorystate = stm_ref1_memory {
	                                     	.min_range = 0.1,
	                                     	.max_range = 0.4,
	                                     	.pi = 3.14159,
	                                     	.current_speed = 0.0,
	                                     	.NOA_Move = {{0.0,0.0}},
	                                     	.DISTANCE = 0.4,
	                                     	.lv = 0.07,
	                                     	.closest_angle = 0,
	                                     	.av = 0.7,
	                                     	.closest_distance = 0
	                                     };
}

void ObstacleAvoidanceStm::step() {
	RESULT_Enum ret = create_RESULT_CONT();
	while (ret.type == create_RESULT_CONT().type) {
		printf("Running small step\n");
		ret = stm_stm_ref1_step(&state, &inputstate, &memorystate, &this->output);
	}
}

RESULT_Enum ObstacleAvoidanceStm::stm_stm_ref1_step(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output) {
		if ((*state).state.type == create_STATES_stm_ref1_NONE().type) {
			{
				(*state).state = create_STATES_stm_ref1_VHFEnabled();
			}
			return create_RESULT_CONT();
		}
		else if ((*state).state.type == create_STATES_stm_ref1_VHFEnabled().type) {
		     	if ((*state).status.type == create_STATUS_ENTER_STATE().type) {
					printf("Entering state VHFENabled\n");
		     		if (!(state)->en_ObstacleAvoidance_VHFEnabled_1_done) {
		     			RESULT_Enum _ret_;
		     			_ret_ = en_ObstacleAvoidance_VHFEnabled_1(state, inputstate, memorystate, output);
		     			return _ret_;
		     		} else {
		     			(*state).status = create_STATUS_ENTER_CHILDREN();
		     			(*state).en_ObstacleAvoidance_VHFEnabled_1_done = false;
		     			(*state).en_ObstacleAvoidance_VHFEnabled_1_counter = 0;
		     			return create_RESULT_CONT();
		     		}
		     	}
		     	else if ((*state).status.type == create_STATUS_ENTER_CHILDREN().type) {
		     	     	(*state).status = create_STATUS_EXECUTE_STATE();
		     	     	{
		     	     		(*inputstate)._transition_ = create_TRANSITIONS_stm_ref1_NONE();
		     	     	}
		     	     	return create_RESULT_CONT();
		     	     }
		     	else if ((*state).status.type == create_STATUS_EXECUTE_STATE().type) {
						printf("Executing state VHFENabled\n");
		     	     	if (	(*inputstate)._transition_.type == create_TRANSITIONS_stm_ref1_NONE().type
		     	     	     ) {
		     	     		if ((inputstate)->closestAngle && (inputstate)->closestDistance && (inputstate)->NOAMove) {
		     	     			(*memorystate).NOA_Move.values[0] = (inputstate)->NOAMove_value.values[0];
		     	     			(*memorystate).NOA_Move.values[1] = (inputstate)->NOAMove_value.values[1];
		     	     			(*inputstate)._transition_ = create_TRANSITIONS_stm_ref1_stm_ref1_t5();
		     	     			(*state).status = create_STATUS_EXIT_CHILDREN();
		     	     			return create_RESULT_WAIT();
		     	     		} else if ((inputstate)->DisableOA) {
		     	     			(*inputstate)._transition_ = create_TRANSITIONS_stm_ref1_stm_ref1_t7();
		     	     			(*state).status = create_STATUS_EXIT_CHILDREN();
		     	     			return create_RESULT_WAIT();
		     	     		} else {
		     	     			return create_RESULT_WAIT();
		     	     		}
		     	     	} else {
		     	     		return create_RESULT_CONT();
		     	     	}
		     	     }
		     	else if ((*state).status.type == create_STATUS_EXIT_CHILDREN().type) {
		     	     	(*state).status = create_STATUS_EXIT_STATE();
		     	     	return create_RESULT_CONT();
		     	     }
		     	else if ((*state).status.type == create_STATUS_EXIT_STATE().type) {
					printf("Exiting state VHFENabled\n");
		     	     	{
		     	     		if (	(*inputstate)._transition_.type == create_TRANSITIONS_stm_ref1_stm_ref1_t5().type
		     	     		     ) {
		     	     			RESULT_Enum _ret_;
		     	     			_ret_ = ObstacleAvoidance_j1(state, inputstate, memorystate, output);
		     	     			return _ret_;
		     	     		} else if (	(*inputstate)._transition_.type == create_TRANSITIONS_stm_ref1_stm_ref1_t7().type
		     	     		            ) {
		     	     			(*state).state = create_STATES_stm_ref1_VHFDisabled();
		     	     			(*state).status = create_STATUS_ENTER_STATE();
		     	     			return create_RESULT_CONT();
		     	     		} else {
		     	     			(*state).status = create_STATUS_INACTIVE();
		     	     			(*state).state = create_STATES_stm_ref1_NONE();
		     	     			return create_RESULT_CONT();
		     	     		}
		     	     	}
		     	     }
		     	else if ((*state).status.type == create_STATUS_INACTIVE().type) {
		     	     	return create_RESULT_CONT();
		     	     }
		     }
		else if ((*state).state.type == create_STATES_stm_ref1_VHFDisabled().type) {
			printf("Entering state VHFDisabled\n");
		     	if ((*state).status.type == create_STATUS_ENTER_STATE().type) {
		     		{
		     			(*state).status = create_STATUS_ENTER_CHILDREN();
		     			return create_RESULT_CONT();
		     		}
		     	}
		     	else if ((*state).status.type == create_STATUS_ENTER_CHILDREN().type) {
		     	     	(*state).status = create_STATUS_EXECUTE_STATE();
		     	     	{
		     	     		(*inputstate)._transition_ = create_TRANSITIONS_stm_ref1_NONE();
		     	     	}
		     	     	return create_RESULT_CONT();
		     	     }
		     	else if ((*state).status.type == create_STATUS_EXECUTE_STATE().type) {
					printf("Executing state VHFDisabled\n");
		     	     	if (	(*inputstate)._transition_.type == create_TRANSITIONS_stm_ref1_NONE().type
		     	     	     ) {
		     	     		if ((inputstate)->EnableOA) {
		     	     			(*inputstate)._transition_ = create_TRANSITIONS_stm_ref1_stm_ref1_t8();
		     	     			(*state).status = create_STATUS_EXIT_CHILDREN();
		     	     			return create_RESULT_WAIT();
		     	     		} else if (!(inputstate)->EnableOA) {
		     	     			(*inputstate)._transition_ = create_TRANSITIONS_stm_ref1_stm_ref1_t1();
		     	     			(*state).status = create_STATUS_EXIT_CHILDREN();
		     	     			return create_RESULT_CONT();
		     	     		} else {
		     	     			return create_RESULT_WAIT();
		     	     		}
		     	     	} else {
		     	     		return create_RESULT_CONT();
		     	     	}
		     	     }
		     	else if ((*state).status.type == create_STATUS_EXIT_CHILDREN().type) {
		     	     	(*state).status = create_STATUS_EXIT_STATE();
		     	     	return create_RESULT_CONT();
		     	     }
		     	else if ((*state).status.type == create_STATUS_EXIT_STATE().type) {
					printf("Exiting state VHFDisabled\n");
		     	     	{
		     	     		if (	(*inputstate)._transition_.type == create_TRANSITIONS_stm_ref1_stm_ref1_t8().type
		     	     		     ) {
		     	     			(*state).state = create_STATES_stm_ref1_VHFEnabled();
		     	     			(*state).status = create_STATUS_ENTER_STATE();
		     	     			return create_RESULT_CONT();
		     	     		} else if (	(*inputstate)._transition_.type == create_TRANSITIONS_stm_ref1_stm_ref1_t1().type
		     	     		            ) {
		     	     			if (!(state)->tr_ObstacleAvoidance_t1_done) {
		     	     				RESULT_Enum _ret_;
		     	     				_ret_ = tr_ObstacleAvoidance_t1(state, inputstate, memorystate, output);
		     	     				return _ret_;
		     	     			} else {
		     	     				(*state).state = create_STATES_stm_ref1_VHFDisabled();
		     	     				(*state).status = create_STATUS_ENTER_STATE();
		     	     				(*state).tr_ObstacleAvoidance_t1_done = false;
		     	     				(*state).tr_ObstacleAvoidance_t1_counter = 0;
		     	     				return create_RESULT_CONT();
		     	     			}
		     	     		} else {
		     	     			(*state).status = create_STATUS_INACTIVE();
		     	     			(*state).state = create_STATES_stm_ref1_NONE();
		     	     			return create_RESULT_CONT();
		     	     		}
		     	     	}
		     	     }
		     	else if ((*state).status.type == create_STATUS_INACTIVE().type) {
		     	     	return create_RESULT_CONT();
		     	     }
		     }
		return create_RESULT_WAIT();
	}
	RESULT_Enum ObstacleAvoidanceStm::tr_ObstacleAvoidance_t13(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		if (	(state)->tr_ObstacleAvoidance_t13_counter == 0 ) {
			(*memory).lv = 0;
			(*state).tr_ObstacleAvoidance_t13_counter = 1;
			return create_RESULT_CONT();
		} else {
			(state)->tr_ObstacleAvoidance_t13_done = true;
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::tr_ObstacleAvoidance_t4(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		if (	(state)->tr_ObstacleAvoidance_t4_counter == 0 ) {
			(*memory).av = (((((memory)->closest_angle + 100)) * (memory)->pi) / 180);
			(*state).tr_ObstacleAvoidance_t4_counter = 1;
			return create_RESULT_CONT();
		} else {
			(state)->tr_ObstacleAvoidance_t4_done = true;
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::ObstacleAvoidance_j1(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output) {
		if (((memorystate)->closest_distance >= (memorystate)->min_range) && ((memorystate)->closest_distance < (memorystate)->max_range) && (absolute((memorystate)->closest_angle) <= 90)) {
			if (!(state)->tr_ObstacleAvoidance_t0_done) {
				RESULT_Enum _ret_;
				_ret_ = tr_ObstacleAvoidance_t0(state, inputstate, memorystate, output);
				return _ret_;
			} else {
				RESULT_Enum _ret_;
				_ret_ = ObstacleAvoidance_j3(state, inputstate, memorystate, output);
				return _ret_;
			}
		} else if (!(((memorystate)->closest_distance >= (memorystate)->min_range) && ((memorystate)->closest_distance < (memorystate)->max_range) && (absolute((memorystate)->closest_angle) <= 90))) {
			if (!(state)->tr_ObstacleAvoidance_t6_done) {
				RESULT_Enum _ret_;
				_ret_ = tr_ObstacleAvoidance_t6(state, inputstate, memorystate, output);
				return _ret_;
			} else {
				(*state).state = create_STATES_stm_ref1_VHFEnabled();
				(*state).status = create_STATUS_ENTER_STATE();
				(*state).tr_ObstacleAvoidance_t6_done = false;
				(*state).tr_ObstacleAvoidance_t6_counter = 0;
				return create_RESULT_CONT();
			}
		} else {
			return create_RESULT_CONT();
		}
	}
	
	RESULT_Enum ObstacleAvoidanceStm::tr_ObstacleAvoidance_t6(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		if (	(state)->tr_ObstacleAvoidance_t6_counter == 0 ) {
			(*memory).lv = ((memory)->NOA_Move.values)[2];
			(*state).tr_ObstacleAvoidance_t6_counter = 1;
			return create_RESULT_CONT();
		} else if (	(state)->tr_ObstacleAvoidance_t6_counter == 1 ) {
			(*memory).av = ((memory)->NOA_Move.values)[1];
			(*state).tr_ObstacleAvoidance_t6_counter = 2;
			return create_RESULT_CONT();
		} else {
			(state)->tr_ObstacleAvoidance_t6_done = true;
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::ObstacleAvoidance_j0(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output) {
		if (((memorystate)->closest_angle > 0)) {
			if (!(state)->tr_ObstacleAvoidance_t3_done) {
				RESULT_Enum _ret_;
				_ret_ = tr_ObstacleAvoidance_t3(state, inputstate, memorystate, output);
				return _ret_;
			} else {
				(*state).state = create_STATES_stm_ref1_VHFEnabled();
				(*state).status = create_STATUS_ENTER_STATE();
				(*state).tr_ObstacleAvoidance_t3_done = false;
				(*state).tr_ObstacleAvoidance_t3_counter = 0;
				return create_RESULT_CONT();
			}
		} else if (((memorystate)->closest_angle < 30)) {
			RESULT_Enum _ret_;
			_ret_ = ObstacleAvoidance_j2(state, inputstate, memorystate, output);
			return _ret_;
		} else if (((memorystate)->closest_angle <= 0)) {
			if (!(state)->tr_ObstacleAvoidance_t4_done) {
				RESULT_Enum _ret_;
				_ret_ = tr_ObstacleAvoidance_t4(state, inputstate, memorystate, output);
				return _ret_;
			} else {
				(*state).state = create_STATES_stm_ref1_VHFEnabled();
				(*state).status = create_STATUS_ENTER_STATE();
				(*state).tr_ObstacleAvoidance_t4_done = false;
				(*state).tr_ObstacleAvoidance_t4_counter = 0;
				return create_RESULT_CONT();
			}
		} else {
			return create_RESULT_CONT();
		}
	}
	
	RESULT_Enum ObstacleAvoidanceStm::tr_ObstacleAvoidance_t0(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		if (	(state)->tr_ObstacleAvoidance_t0_counter == 0 ) {
			(*memory).current_speed = ((memory)->NOA_Move.values)[2];
			(*state).tr_ObstacleAvoidance_t0_counter = 1;
			return create_RESULT_CONT();
		} else {
			(state)->tr_ObstacleAvoidance_t0_done = true;
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::ObstacleAvoidance_j3(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output) {
		if (((memorystate)->closest_distance <= (memorystate)->DISTANCE)) {
			if (!(state)->tr_ObstacleAvoidance_t13_done) {
				RESULT_Enum _ret_;
				_ret_ = tr_ObstacleAvoidance_t13(state, inputstate, memorystate, output);
				return _ret_;
			} else {
				RESULT_Enum _ret_;
				_ret_ = ObstacleAvoidance_j0(state, inputstate, memorystate, output);
				return _ret_;
			}
		} else if (((memorystate)->closest_distance > (memorystate)->DISTANCE)) {
			if (!(state)->tr_ObstacleAvoidance_t12_done) {
				RESULT_Enum _ret_;
				_ret_ = tr_ObstacleAvoidance_t12(state, inputstate, memorystate, output);
				return _ret_;
			} else {
				RESULT_Enum _ret_;
				_ret_ = ObstacleAvoidance_j0(state, inputstate, memorystate, output);
				return _ret_;
			}
		} else {
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::ObstacleAvoidance_j2(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memorystate, stm_ref1_output_Enum_Channel* output) {
		if (((memorystate)->closest_distance >= (memorystate)->DISTANCE)) {
			if (!(state)->tr_ObstacleAvoidance_t11_done) {
				RESULT_Enum _ret_;
				_ret_ = tr_ObstacleAvoidance_t11(state, inputstate, memorystate, output);
				return _ret_;
			} else {
				(*state).state = create_STATES_stm_ref1_VHFEnabled();
				(*state).status = create_STATUS_ENTER_STATE();
				(*state).tr_ObstacleAvoidance_t11_done = false;
				(*state).tr_ObstacleAvoidance_t11_counter = 0;
				return create_RESULT_CONT();
			}
		} else if (((memorystate)->closest_distance < (memorystate)->DISTANCE)) {
			if (!(state)->tr_ObstacleAvoidance_t10_done) {
				RESULT_Enum _ret_;
				_ret_ = tr_ObstacleAvoidance_t10(state, inputstate, memorystate, output);
				return _ret_;
			} else {
				(*state).state = create_STATES_stm_ref1_VHFEnabled();
				(*state).status = create_STATUS_ENTER_STATE();
				(*state).tr_ObstacleAvoidance_t10_done = false;
				(*state).tr_ObstacleAvoidance_t10_counter = 0;
				return create_RESULT_CONT();
			}
		} else {
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::tr_ObstacleAvoidance_t1(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		if (	(state)->tr_ObstacleAvoidance_t1_counter == 0 ) {
			(*state).tr_ObstacleAvoidance_t1_counter = 1;
			return create_RESULT_WAIT();
		} else {
			(state)->tr_ObstacleAvoidance_t1_done = true;
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::en_ObstacleAvoidance_VHFEnabled_1(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		printf("Running entry action of VHFDisabled\n");
		if (	(state)->en_ObstacleAvoidance_VHFEnabled_1_counter == 0 ) {
			{
				auto message = cachecons_interfaces::msg::VectorFloat2();
    			message.values[0] = (memory)->av;
    			message.values[1] = (memory)->lv;
				printf("Trying to publish oamove\n");
    			output->oamove_publisher_->publish(message);
				printf("Published oamove\n");
			}
			(*state).en_ObstacleAvoidance_VHFEnabled_1_counter = 1;
			return create_RESULT_CONT();
		} else {
			printf("Finished running entry action of VHFDisabled\n");
			(state)->en_ObstacleAvoidance_VHFEnabled_1_done = true;
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::tr_ObstacleAvoidance_t3(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		if (	(state)->tr_ObstacleAvoidance_t3_counter == 0 ) {
			(*memory).av = (((((memory)->closest_angle - 100)) * (memory)->pi) / 180);
			(*state).tr_ObstacleAvoidance_t3_counter = 1;
			return create_RESULT_CONT();
		} else {
			(state)->tr_ObstacleAvoidance_t3_done = true;
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::tr_ObstacleAvoidance_t11(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		if (	(state)->tr_ObstacleAvoidance_t11_counter == 0 ) {
			(*memory).lv = 0;
			(*state).tr_ObstacleAvoidance_t11_counter = 1;
			return create_RESULT_CONT();
		} else {
			(state)->tr_ObstacleAvoidance_t11_done = true;
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::tr_ObstacleAvoidance_t10(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		if (	(state)->tr_ObstacleAvoidance_t10_counter == 0 ) {
			(*memory).lv = -(memory)->DISTANCE;
			(*state).tr_ObstacleAvoidance_t10_counter = 1;
			return create_RESULT_CONT();
		} else {
			(state)->tr_ObstacleAvoidance_t10_done = true;
			return create_RESULT_CONT();
		}
	}
	RESULT_Enum ObstacleAvoidanceStm::tr_ObstacleAvoidance_t12(struct stm_ref1_state* state, struct stm_ref1_inputstate* inputstate, struct stm_ref1_memory* memory, stm_ref1_output_Enum_Channel* output) {
		if (	(state)->tr_ObstacleAvoidance_t12_counter == 0 ) {
			(*memory).lv = ((memory)->current_speed / 2);
			(*state).tr_ObstacleAvoidance_t12_counter = 1;
			return create_RESULT_CONT();
		} else {
			(state)->tr_ObstacleAvoidance_t12_done = true;
			return create_RESULT_CONT();
		}
	}

#pragma GCC diagnostic pop