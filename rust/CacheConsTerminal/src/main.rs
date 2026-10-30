#![allow(unused_parens)]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]

use std::thread;
use std::sync::mpsc;
use std::sync::mpsc::{SyncSender, Receiver};
use std::sync::Barrier;
use std::sync::Arc;
use std::time::Duration;
use rand::prelude::*;
// These are only used for the interaction with the user and should be encapsulated in
// whatever extension mechanism that will be developed to allow integration with different
// simulators.
use std::io::Write;
use std::io::Read;
use std::rc::Rc;
use std::cell::RefCell;

use log::debug;
use simplelog::*;
use std::fs::File;

use serde::{Deserialize, Serialize};
use serde_json::Result;
use clap::{App,Arg};

#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum M_CacheConsMod_input {
	HomeSeen,
	HomeReached,
	PuckCarried,
	ClusterSeen(i32),
	TargetSeen(f32),
	_done_,
	_terminate_,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum M_CacheConsMod_output {
	EnableTargetWatch,
	stop,
	EnableClusterWatch,
	search,
	stopsearch,
	DisableTargetWatch,
	DepositPuck,
	EnableHomeWatch,
	DisableClusterWatch,
	obstacle_turn(f32),
	DisableHomeWatch,
	r#move(f32,f32),
	_done_,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATUS {
	ENTER_STATE,
	ENTER_CHILDREN,
	EXECUTE_STATE,
	EXIT_CHILDREN,
	EXIT_STATE,
	INACTIVE,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum RESULT {
	WAIT,
	CONT,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_PU_Scan_Wander {
	NONE,
	Turn,
	Move_Forward,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_Back_Up_Moving {
	NONE,
	Move,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_Exile {
	NONE,
	Move,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_Turning_To_Target {
	NONE,
	Watch,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_PU_Scan {
	NONE,
	Wander,
	CalculateProb,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum S_input {
	HomeReached,
	ClusterSeen(i32),
	TargetSeen(f32),
	PuckCarried,
	HomeSeen,
	_done_,
	_terminate_,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_Look_For_Target {
	NONE,
	Watch,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_Back_Up_Turning {
	NONE,
	Move,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum S_output {
	DepositPuck,
	obstacle_turn(f32),
	stop,
	EnableClusterWatch,
	DisableHomeWatch,
	stopsearch,
	EnableTargetWatch,
	r#move(f32,f32),
	search,
	DisableTargetWatch,
	EnableHomeWatch,
	DisableClusterWatch,
	_done_,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S {
	NONE,
	PU_Scan,
	Turning_To_Target,
	Look_For_Target,
	Moving_To_Target,
	Look_For_Home,
	Turning_Home,
	Back_Up_Moving,
	Exile,
	DepositPuck,
	Move_To_Home,
	Back_Up_Turning,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum C_C_input {
	HomeSeen,
	PuckCarried,
	HomeReached,
	TargetSeen(f32),
	ClusterSeen(i32),
	_done_,
	_terminate_,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum TRANSITIONS_S {
	NONE,
	S_t2,
	S_t18,
	S_t9,
	S_PU_Scan_t1,
	S_Look_For_Home_t1,
	S_t4,
	S_t12,
	S_Moving_To_Target_t1,
	S_Back_Up_Moving_t1,
	S_t8,
	S_Move_To_Home_t1,
	S_PU_Scan_Wander_t4,
	S_t5,
	S_t14,
	S_Look_For_Target_t2,
	S_Move_To_Home_t0,
	S_Turning_To_Target_t5,
	S_PU_Scan_Wander_t0,
	S_Back_Up_Turning_t1,
	S_PU_Scan_t2,
	S_Back_Up_Turning_t0,
	S_t11,
	S_PU_Scan_Wander_t3,
	S_Look_For_Home_t0,
	S_t15,
	S_Back_Up_Moving_t0,
	S_Exile_t1,
	S_t16,
	S_t0,
	S_t6,
	S_Look_For_Target_t0,
	S_t7,
	S_Turning_To_Target_t4,
	S_t17,
	S_PU_Scan_t0,
	S_PU_Scan_Wander_t2,
	S_Exile_t0,
	S_t1,
	S_Moving_To_Target_t0,
	S_Turning_Home_t0,
	S_Turning_Home_t1,
	S_t13,
	S_t3,
	S_PU_Scan_Wander_t1,
	S_t10,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_Turning_Home {
	NONE,
	Watch,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_Look_For_Home {
	NONE,
	Watch,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_Move_To_Home {
	NONE,
	Watch,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum STATES_S_Moving_To_Target {
	NONE,
	Watch,
}
#[derive(PartialEq,Debug,Clone,Copy,Serialize,Deserialize)]
enum C_C_output {
	DisableClusterWatch,
	EnableHomeWatch,
	DepositPuck,
	DisableTargetWatch,
	stop,
	search,
	EnableClusterWatch,
	EnableTargetWatch,
	stopsearch,
	obstacle_turn(f32),
	r#move(f32,f32),
	DisableHomeWatch,
	_done_,
}

#[derive(Debug,Serialize,Deserialize)]
struct S_Exile_state {
	done:bool,
	state:STATES_S_Exile,
	target_state:STATES_S_Exile,
	status:STATUS,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_Moving_To_Target_state {
	done:bool,
	state:STATES_S_Moving_To_Target,
	target_state:STATES_S_Moving_To_Target,
	status:STATUS,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_inputstate {
	HomeReached:bool,
	ClusterSeen:bool,
	ClusterSeen_value:i32,
	TargetSeen:bool,
	TargetSeen_value:f32,
	PuckCarried:bool,
	HomeSeen:bool,
	_clock_T:i32,
	_transition_:TRANSITIONS_S,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_Turning_To_Target_state {
	done:bool,
	state:STATES_S_Turning_To_Target,
	target_state:STATES_S_Turning_To_Target,
	status:STATUS,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_PU_Scan_Wander_state {
	done:bool,
	state:STATES_S_PU_Scan_Wander,
	target_state:STATES_S_PU_Scan_Wander,
	status:STATUS,
	en_Wander_Move_Forward_1_done:bool,
	en_Wander_Move_Forward_1_counter:i32,
	en_Wander_Turn_1_done:bool,
	en_Wander_Turn_1_counter:i32,
	tr_SimCacheCons_PU_Scan_Wander_t2_done:bool,
	tr_SimCacheCons_PU_Scan_Wander_t2_counter:i32,
	tr_SimCacheCons_PU_Scan_Wander_t1_done:bool,
	tr_SimCacheCons_PU_Scan_Wander_t1_counter:i32,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_memory {
	randnat:i32,
	homeangle:f32,
	av:f32,
	distance:i32,
	k1:i32,
	timeout:f32,
	pi:f32,
	angle:f32,
	clustersize:i32,
	prob:f32,
	randcoef:f32,
	fv:f32,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_Back_Up_Turning_state {
	done:bool,
	state:STATES_S_Back_Up_Turning,
	target_state:STATES_S_Back_Up_Turning,
	status:STATUS,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_Back_Up_Moving_state {
	done:bool,
	state:STATES_S_Back_Up_Moving,
	target_state:STATES_S_Back_Up_Moving,
	status:STATUS,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_Look_For_Target_state {
	done:bool,
	state:STATES_S_Look_For_Target,
	target_state:STATES_S_Look_For_Target,
	status:STATUS,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_Look_For_Home_state {
	done:bool,
	state:STATES_S_Look_For_Home,
	target_state:STATES_S_Look_For_Home,
	status:STATUS,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_Move_To_Home_state {
	done:bool,
	state:STATES_S_Move_To_Home,
	target_state:STATES_S_Move_To_Home,
	status:STATUS,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_state {
	done:bool,
	state:STATES_S,
	target_state:STATES_S,
	status:STATUS,
	s_PU_Scan:S_PU_Scan_state,
	s_Turning_To_Target:S_Turning_To_Target_state,
	s_Look_For_Target:S_Look_For_Target_state,
	s_Moving_To_Target:S_Moving_To_Target_state,
	s_Look_For_Home:S_Look_For_Home_state,
	s_Turning_Home:S_Turning_Home_state,
	s_Back_Up_Moving:S_Back_Up_Moving_state,
	s_Exile:S_Exile_state,
	s_Move_To_Home:S_Move_To_Home_state,
	s_Back_Up_Turning:S_Back_Up_Turning_state,
	en_SimCacheCons_Moving_To_Target_1_done:bool,
	en_SimCacheCons_Moving_To_Target_1_counter:i32,
	en_SimCacheCons_PU_Scan_1_done:bool,
	en_SimCacheCons_PU_Scan_1_counter:i32,
	en_SimCacheCons_Back_Up_Turning_1_done:bool,
	en_SimCacheCons_Back_Up_Turning_1_counter:i32,
	en_SimCacheCons_Look_For_Target_1_done:bool,
	en_SimCacheCons_Look_For_Target_1_counter:i32,
	en_SimCacheCons_Exile_1_done:bool,
	en_SimCacheCons_Exile_1_counter:i32,
	en_SimCacheCons_DepositPuck_1_done:bool,
	en_SimCacheCons_DepositPuck_1_counter:i32,
	en_SimCacheCons_Back_Up_Moving_1_done:bool,
	en_SimCacheCons_Back_Up_Moving_1_counter:i32,
	en_SimCacheCons_Turning_Home_1_done:bool,
	en_SimCacheCons_Turning_Home_1_counter:i32,
	en_SimCacheCons_Look_For_Home_1_done:bool,
	en_SimCacheCons_Look_For_Home_1_counter:i32,
	en_SimCacheCons_Turning_To_Target_1_done:bool,
	en_SimCacheCons_Turning_To_Target_1_counter:i32,
	en_SimCacheCons_Move_To_Home_1_done:bool,
	en_SimCacheCons_Move_To_Home_1_counter:i32,
	tr_SimCacheCons_t11_done:bool,
	tr_SimCacheCons_t11_counter:i32,
	tr_SimCacheCons_t2_done:bool,
	tr_SimCacheCons_t2_counter:i32,
	tr_SimCacheCons_t18_done:bool,
	tr_SimCacheCons_t18_counter:i32,
	tr_SimCacheCons_t9_done:bool,
	tr_SimCacheCons_t9_counter:i32,
	tr_SimCacheCons_t15_done:bool,
	tr_SimCacheCons_t15_counter:i32,
	tr_SimCacheCons_t4_done:bool,
	tr_SimCacheCons_t4_counter:i32,
	tr_SimCacheCons_t12_done:bool,
	tr_SimCacheCons_t12_counter:i32,
	tr_SimCacheCons_t16_done:bool,
	tr_SimCacheCons_t16_counter:i32,
	tr_SimCacheCons_t6_done:bool,
	tr_SimCacheCons_t6_counter:i32,
	tr_SimCacheCons_t7_done:bool,
	tr_SimCacheCons_t7_counter:i32,
	tr_SimCacheCons_t8_done:bool,
	tr_SimCacheCons_t8_counter:i32,
	tr_SimCacheCons_t17_done:bool,
	tr_SimCacheCons_t17_counter:i32,
	tr_SimCacheCons_t5_done:bool,
	tr_SimCacheCons_t5_counter:i32,
	tr_SimCacheCons_t1_done:bool,
	tr_SimCacheCons_t1_counter:i32,
	tr_SimCacheCons_t14_done:bool,
	tr_SimCacheCons_t14_counter:i32,
	tr_SimCacheCons_t13_done:bool,
	tr_SimCacheCons_t13_counter:i32,
	tr_SimCacheCons_t3_done:bool,
	tr_SimCacheCons_t3_counter:i32,
	tr_SimCacheCons_t10_done:bool,
	tr_SimCacheCons_t10_counter:i32,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_Turning_Home_state {
	done:bool,
	state:STATES_S_Turning_Home,
	target_state:STATES_S_Turning_Home,
	status:STATUS,
}
#[derive(Debug,Serialize,Deserialize)]
struct S_PU_Scan_state {
	done:bool,
	state:STATES_S_PU_Scan,
	target_state:STATES_S_PU_Scan,
	status:STATUS,
	s_Wander:S_PU_Scan_Wander_state,
	en_PU_Scan_CalculateProb_1_done:bool,
	en_PU_Scan_CalculateProb_1_counter:i32,
	tr_SimCacheCons_PU_Scan_t1_done:bool,
	tr_SimCacheCons_PU_Scan_t1_counter:i32,
}

fn main() -> std::result::Result<(), String> {
	let matches = App::new("SimCacheCons")
						.version("1.0.0")
						.about("Simulation of RoboSim model SimCacheCons")
						.arg(Arg::with_name("record")
							.short('r')
							.long("record")
							.value_name("FILE")
							.help("Records the state of a simulation in a file.")
							.takes_value(true)
						).arg(Arg::with_name("replay")
							.short('p')
							.long("replay")
							.value_name("FILE")
							.help("Replays the simulation with the contents of a file")
							.takes_value(true)
						).get_matches();
	
	let _ = WriteLogger::init(
		LevelFilter::Trace, Config::default(), File::create("test.log").unwrap());
	
	let _args: Vec<String> = std::env::args().collect();
    if _args.len() <= 0 {
        eprintln!("error: Not enough arguments.");
            std::process::exit(1);
    }
    
    // TODO: This record and replay mechanism needs to be generalised for multiple
    // statemachine and controllers. Currently, only the recording functionality
    // is implemented and only records data for one state machine.
    
    let replay_value = matches.value_of("replay").and_then(|s|Some(s.to_string()));
	let record_value = matches.value_of("record").and_then(|s|Some(s.to_string()));

	let replayFile = if let Some(s) = replay_value {
		let file = std::fs::File::create(s);
		match file {
			Ok(f) => Some(f),
			Err(e) => { eprintln!("{}", e); std::process::exit(1) }
		}
	} else {
		None
	};

	let (send_data, recv_data)= mpsc::channel::<Option<String>>();
	
	let recording = record_value.is_some();

	let recorder = {
		let mut code = 0;
		let mut first = true;
		thread::spawn(move || {
			let mut recordFile = if let Some(s) = record_value {
				let mut file = std::fs::OpenOptions::new()
					.write(true)
					.append(false)
					.create(true)
					.truncate(true)
					.open(s);
				 match file {
					Ok(mut f) => { Some(f) },
					Err(e) => { eprintln!("{}", e); std::process::exit(1) }
				}
			} else {
				None
			};
			
			// TODO: Uncomment some of the commented sections below to 
			// record the state at each cycle. Also, comment the final
			// treatment of lastState.
			// This was changed to avoid writing to a file too much.
			// For replaying, the final state is all that is needed.
			// Adding extra information about the inputs and output
			// would be beneficial to reproduce the interactions
			// that led to the final state.
			
			// if let Some(file) = &mut recordFile {
			// 	file.write_all(b"[\n").expect("Writing to record file");
			// }
			let mut terminate = false;
			let mut lastState = None;
			while !terminate {
				let value = recv_data.recv();
				match value {
					Ok(v) => {
						match v {
							Some(state) => {
								lastState = Some(state);
								// if let Some(file) = &mut recordFile {
								// 	if !first {
								// 		file.write_all(b",\n").expect("Failed to write to record file");
								// 	} else {
								// 		first = false;
								// 	}
								// 	file.write_all(state.as_bytes()).expect("Failed to write to record file");
								// }
							},
							None => {
								terminate = true;
							}
						}

					},
					Err(e) => {
						terminate = true;
						code = 1;
						eprintln!("Failed to receive debug value.");
					}
				}
			}

			// if let Some(file) = &mut recordFile {
			// 	file.write_all(b"\n]\n").expect("Writing to record file");
			// }

			println!("\nTerminating application.");

			if let Some(file) = &mut recordFile {
				if let Some(state) = lastState {
					println!("Saving last state");
					file.write_all(state.as_bytes()).expect("Failed to write to record file");
				} else {
					println!("No state to save");
				}
				file.sync_all().expect("Failed to synchronise recorded file.");
			}

			std::process::exit(code);
		})
	};
	
	let ctrlc_send = send_data.clone();
	ctrlc::set_handler(move || {
		ctrlc_send.send(None).expect("Failed to send termination signal.");
	}).expect("Failed to setup ctrl-c handler.");
	
	// Module channel declarations;
	let (send_start_CacheConsMod, recv_start_CacheConsMod) = mpsc::sync_channel(0);
	let (send_end_CacheConsMod, recv_end_CacheConsMod) = mpsc::sync_channel(0);
	let (send_start_C, recv_start_C) = mpsc::sync_channel(0);
	let (send_end_C, recv_end_C) = mpsc::sync_channel(0);
	let (send_start_S, recv_start_S) = mpsc::sync_channel(0);
	let (send_end_S, recv_end_S) = mpsc::sync_channel(0);
	// Instantiate threads;
	let control_thread = {
	                       	thread::spawn(move || {
	                     		let mut terminate__: bool = false;
	                     		while !terminate__ {
	                     			{
	                     				let mut inputdone: bool = false;
	                     				while !inputdone {
	                     					{
	                     						let _s0: String;
	                     						_s0 = format!("{}", "Enter an event:");
	                     						print!("{}", _s0);
	                     						std::io::stdout().flush().unwrap();
	                     					}
	                     					let mut _event_: String;
	                     					let mut _temp_ = String::new();
	                     					std::io::stdin().read_line(&mut _temp_).expect("Failed to read input.");
	                     					_event_ = _temp_.as_str().trim().to_string();
	                     					match (_event_).as_str() {
	                     						"HomeReached" => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Found event HomeReached");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     							send_start_CacheConsMod.send(M_CacheConsMod_input::HomeReached).unwrap();
	                     						},
	                     						"" => {
	                     							send_start_CacheConsMod.send(M_CacheConsMod_input::_done_).unwrap();
	                     							inputdone = true;
	                     						},
	                     						"ClusterSeen" => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Found event ClusterSeen");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Enter value for event ClusterSeen of type nat: ");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							let mut _svalue_: String;
	                     							let mut _temp_ = String::new();
	                     							std::io::stdin().read_line(&mut _temp_).expect("Failed to read input.");
	                     							_svalue_ = _temp_.as_str().trim().to_string();
	                     							let mut _value_: i32;
	                     							_value_ = (_svalue_).parse::<i32>().unwrap();
	                     							send_start_CacheConsMod.send(M_CacheConsMod_input::ClusterSeen(_value_)).unwrap();
	                     						},
	                     						"PuckCarried" => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Found event PuckCarried");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     							send_start_CacheConsMod.send(M_CacheConsMod_input::PuckCarried).unwrap();
	                     						},
	                     						"TargetSeen" => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Found event TargetSeen");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Enter value for event TargetSeen of type real: ");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							let mut _svalue_: String;
	                     							let mut _temp_ = String::new();
	                     							std::io::stdin().read_line(&mut _temp_).expect("Failed to read input.");
	                     							_svalue_ = _temp_.as_str().trim().to_string();
	                     							let mut _value_: f32;
	                     							_value_ = (_svalue_).parse::<f32>().unwrap();
	                     							send_start_CacheConsMod.send(M_CacheConsMod_input::TargetSeen(_value_)).unwrap();
	                     						},
	                     						"HomeSeen" => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Found event HomeSeen");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     							send_start_CacheConsMod.send(M_CacheConsMod_input::HomeSeen).unwrap();
	                     						},
	                     						"$end" => {
	                     							send_start_CacheConsMod.send(M_CacheConsMod_input::_terminate_).unwrap();
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "Terminating system. One more cycle will run before this program ends.");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     							terminate__ = true;
	                     							inputdone = true;
	                     						},
	                     						_ => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", format!("{}{}","Unknown event: ", _event_));
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						}
	                     					}
	                     				}
	                     			}
	                     			{
	                     				let mut outputdone: bool = false;
	                     				while !outputdone {
	                     					let mut _output_: M_CacheConsMod_output;
	                     					_output_ = recv_end_CacheConsMod.recv().unwrap();
	                     					match _output_ {
	                     						M_CacheConsMod_output::DepositPuck => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output DepositPuck");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::obstacle_turn(_aux1_) => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output obstacle_turn");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "(");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", _aux1_);
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", ")");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::stop => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output stop");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::EnableClusterWatch => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output EnableClusterWatch");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::DisableHomeWatch => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output DisableHomeWatch");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::stopsearch => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output stopsearch");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::EnableTargetWatch => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output EnableTargetWatch");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::search => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output search");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::r#move(_aux1_, _aux2_) => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output move");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "(");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", _aux1_);
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", ",");
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", _aux2_);
	                     								print!("{}", _s0);
	                     								std::io::stdout().flush().unwrap();
	                     							}
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", ")");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::DisableTargetWatch => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output DisableTargetWatch");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::EnableHomeWatch => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output EnableHomeWatch");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::DisableClusterWatch => {
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "output DisableClusterWatch");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     						M_CacheConsMod_output::_done_ => {
	                     							outputdone = true;
	                     							{
	                     								let _s0: String;
	                     								_s0 = format!("{}", "---------------------");
	                     								println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                     							}
	                     						},
	                     					}
	                     				}
	                     			}
	                     		}
	                     	})
	                     };		
	let mod_CacheConsMod_thread_thread = {
	                                       	thread::spawn(move || {
	                                     		let mut terminate__: bool = false;
	                                     		while !terminate__ {
	                                     			{
	                                     				let mut inputDone: bool = false;
	                                     				while !inputDone {
	                                     					{
	                                     						let _s0: String;
	                                     						_s0 = format!("{}", "- Waiting for input on channel start_CacheConsMod");
	                                     						debug!("{}", _s0);
	                                     					}
	                                     					let mut _input_: M_CacheConsMod_input;
	                                     					_input_ = recv_start_CacheConsMod.recv().unwrap();
	                                     					{
	                                     						let _s0: String;
	                                     						_s0 = format!("{}", "- Read input on channel start_CacheConsMod");
	                                     						debug!("{}", _s0);
	                                     					}
	                                     					match _input_ {
	                                     						M_CacheConsMod_input::HomeSeen => {
	                                     							send_start_C.send(C_C_input::HomeSeen).unwrap();
	                                     						},
	                                     						M_CacheConsMod_input::HomeReached => {
	                                     							send_start_C.send(C_C_input::HomeReached).unwrap();
	                                     						},
	                                     						M_CacheConsMod_input::PuckCarried => {
	                                     							send_start_C.send(C_C_input::PuckCarried).unwrap();
	                                     						},
	                                     						M_CacheConsMod_input::ClusterSeen(_aux1_) => {
	                                     							send_start_C.send(C_C_input::ClusterSeen(_aux1_)).unwrap();
	                                     						},
	                                     						M_CacheConsMod_input::TargetSeen(_aux1_) => {
	                                     							send_start_C.send(C_C_input::TargetSeen(_aux1_)).unwrap();
	                                     						},
	                                     						M_CacheConsMod_input::_done_ => {
	                                     							send_start_C.send(C_C_input::_done_).unwrap();
	                                     							inputDone = true;
	                                     						},
	                                     						M_CacheConsMod_input::_terminate_ => {
	                                     							send_start_C.send(C_C_input::_terminate_).unwrap();
	                                     							terminate__ = true;
	                                     						},
	                                     					}
	                                     				}
	                                     			}
	                                     			{
	                                     				let _s0: String;
	                                     				_s0 = format!("{}", "Finished reading inputs of module CacheConsMod");
	                                     				debug!("{}", _s0);
	                                     			}
	                                     			mod_CacheConsMod_step(&send_start_C
	                                     			                      , &recv_end_C);
	                                     			{
	                                     				let mut outputDone: bool = false;
	                                     				while !outputDone {
	                                     					let mut _output_: C_C_output;
	                                     					_output_ = recv_end_C.recv().unwrap();
	                                     					match _output_ {
	                                     						C_C_output::EnableTargetWatch => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::EnableTargetWatch).unwrap();
	                                     						},
	                                     						C_C_output::stop => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::stop).unwrap();
	                                     						},
	                                     						C_C_output::EnableClusterWatch => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::EnableClusterWatch).unwrap();
	                                     						},
	                                     						C_C_output::search => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::search).unwrap();
	                                     						},
	                                     						C_C_output::stopsearch => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::stopsearch).unwrap();
	                                     						},
	                                     						C_C_output::DisableTargetWatch => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::DisableTargetWatch).unwrap();
	                                     						},
	                                     						C_C_output::DepositPuck => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::DepositPuck).unwrap();
	                                     						},
	                                     						C_C_output::EnableHomeWatch => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::EnableHomeWatch).unwrap();
	                                     						},
	                                     						C_C_output::DisableClusterWatch => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::DisableClusterWatch).unwrap();
	                                     						},
	                                     						C_C_output::obstacle_turn(_aux1_) => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::obstacle_turn(_aux1_)).unwrap();
	                                     						},
	                                     						C_C_output::DisableHomeWatch => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::DisableHomeWatch).unwrap();
	                                     						},
	                                     						C_C_output::r#move(_aux1_, _aux2_) => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::r#move(_aux1_,_aux2_)).unwrap();
	                                     						},
	                                     						C_C_output::_done_ => {
	                                     							send_end_CacheConsMod.send(M_CacheConsMod_output::_done_).unwrap();
	                                     							outputDone = true;
	                                     						},
	                                     					}
	                                     				}
	                                     			}
	                                     		}
	                                     	})
	                                     };		
	let stm_S_thread = {
	                   	let stm_S_record = send_data.clone();
	                     	thread::spawn(move || {
	                   		// state machine variable declarations;
	                   		let mut inputstate: S_inputstate = S_inputstate {
	                   		                                   	HomeReached: false,
	                   		                                   	ClusterSeen: false,
	                   		                                   	ClusterSeen_value: 0_i32,
	                   		                                   	TargetSeen: false,
	                   		                                   	TargetSeen_value: 0.0_f32,
	                   		                                   	PuckCarried: false,
	                   		                                   	HomeSeen: false,
	                   		                                   	_clock_T: 0_i32,
	                   		                                   	_transition_: TRANSITIONS_S::NONE
	                   		                                   };
	                   		let mut state: S_state = S_state {
	                   		                         	done: false,
	                   		                         	state: STATES_S::NONE,
	                   		                         	target_state: STATES_S::NONE,
	                   		                         	status: STATUS::ENTER_STATE,
	                   		                         	s_PU_Scan: S_PU_Scan_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_PU_Scan::NONE,
	                   		                         		target_state: STATES_S_PU_Scan::NONE,
	                   		                         		status: STATUS::ENTER_STATE,
	                   		                         		s_Wander: S_PU_Scan_Wander_state {
	                   		                         			done: false,
	                   		                         			state: STATES_S_PU_Scan_Wander::NONE,
	                   		                         			target_state: STATES_S_PU_Scan_Wander::NONE,
	                   		                         			status: STATUS::ENTER_STATE,
	                   		                         			en_Wander_Move_Forward_1_done: false,
	                   		                         			en_Wander_Move_Forward_1_counter: 0_i32,
	                   		                         			en_Wander_Turn_1_done: false,
	                   		                         			en_Wander_Turn_1_counter: 0_i32,
	                   		                         			tr_SimCacheCons_PU_Scan_Wander_t2_done: false,
	                   		                         			tr_SimCacheCons_PU_Scan_Wander_t2_counter: 0_i32,
	                   		                         			tr_SimCacheCons_PU_Scan_Wander_t1_done: false,
	                   		                         			tr_SimCacheCons_PU_Scan_Wander_t1_counter: 0_i32
	                   		                         		},
	                   		                         		en_PU_Scan_CalculateProb_1_done: false,
	                   		                         		en_PU_Scan_CalculateProb_1_counter: 0_i32,
	                   		                         		tr_SimCacheCons_PU_Scan_t1_done: false,
	                   		                         		tr_SimCacheCons_PU_Scan_t1_counter: 0_i32
	                   		                         	},
	                   		                         	s_Turning_To_Target: S_Turning_To_Target_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_Turning_To_Target::NONE,
	                   		                         		target_state: STATES_S_Turning_To_Target::NONE,
	                   		                         		status: STATUS::ENTER_STATE
	                   		                         	},
	                   		                         	s_Look_For_Target: S_Look_For_Target_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_Look_For_Target::NONE,
	                   		                         		target_state: STATES_S_Look_For_Target::NONE,
	                   		                         		status: STATUS::ENTER_STATE
	                   		                         	},
	                   		                         	s_Moving_To_Target: S_Moving_To_Target_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_Moving_To_Target::NONE,
	                   		                         		target_state: STATES_S_Moving_To_Target::NONE,
	                   		                         		status: STATUS::ENTER_STATE
	                   		                         	},
	                   		                         	s_Look_For_Home: S_Look_For_Home_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_Look_For_Home::NONE,
	                   		                         		target_state: STATES_S_Look_For_Home::NONE,
	                   		                         		status: STATUS::ENTER_STATE
	                   		                         	},
	                   		                         	s_Turning_Home: S_Turning_Home_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_Turning_Home::NONE,
	                   		                         		target_state: STATES_S_Turning_Home::NONE,
	                   		                         		status: STATUS::ENTER_STATE
	                   		                         	},
	                   		                         	s_Back_Up_Moving: S_Back_Up_Moving_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_Back_Up_Moving::NONE,
	                   		                         		target_state: STATES_S_Back_Up_Moving::NONE,
	                   		                         		status: STATUS::ENTER_STATE
	                   		                         	},
	                   		                         	s_Exile: S_Exile_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_Exile::NONE,
	                   		                         		target_state: STATES_S_Exile::NONE,
	                   		                         		status: STATUS::ENTER_STATE
	                   		                         	},
	                   		                         	s_Move_To_Home: S_Move_To_Home_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_Move_To_Home::NONE,
	                   		                         		target_state: STATES_S_Move_To_Home::NONE,
	                   		                         		status: STATUS::ENTER_STATE
	                   		                         	},
	                   		                         	s_Back_Up_Turning: S_Back_Up_Turning_state {
	                   		                         		done: false,
	                   		                         		state: STATES_S_Back_Up_Turning::NONE,
	                   		                         		target_state: STATES_S_Back_Up_Turning::NONE,
	                   		                         		status: STATUS::ENTER_STATE
	                   		                         	},
	                   		                         	en_SimCacheCons_Moving_To_Target_1_done: false,
	                   		                         	en_SimCacheCons_Moving_To_Target_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_PU_Scan_1_done: false,
	                   		                         	en_SimCacheCons_PU_Scan_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_Back_Up_Turning_1_done: false,
	                   		                         	en_SimCacheCons_Back_Up_Turning_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_Look_For_Target_1_done: false,
	                   		                         	en_SimCacheCons_Look_For_Target_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_Exile_1_done: false,
	                   		                         	en_SimCacheCons_Exile_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_DepositPuck_1_done: false,
	                   		                         	en_SimCacheCons_DepositPuck_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_Back_Up_Moving_1_done: false,
	                   		                         	en_SimCacheCons_Back_Up_Moving_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_Turning_Home_1_done: false,
	                   		                         	en_SimCacheCons_Turning_Home_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_Look_For_Home_1_done: false,
	                   		                         	en_SimCacheCons_Look_For_Home_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_Turning_To_Target_1_done: false,
	                   		                         	en_SimCacheCons_Turning_To_Target_1_counter: 0_i32,
	                   		                         	en_SimCacheCons_Move_To_Home_1_done: false,
	                   		                         	en_SimCacheCons_Move_To_Home_1_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t11_done: false,
	                   		                         	tr_SimCacheCons_t11_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t2_done: false,
	                   		                         	tr_SimCacheCons_t2_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t18_done: false,
	                   		                         	tr_SimCacheCons_t18_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t9_done: false,
	                   		                         	tr_SimCacheCons_t9_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t15_done: false,
	                   		                         	tr_SimCacheCons_t15_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t4_done: false,
	                   		                         	tr_SimCacheCons_t4_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t12_done: false,
	                   		                         	tr_SimCacheCons_t12_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t16_done: false,
	                   		                         	tr_SimCacheCons_t16_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t6_done: false,
	                   		                         	tr_SimCacheCons_t6_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t7_done: false,
	                   		                         	tr_SimCacheCons_t7_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t8_done: false,
	                   		                         	tr_SimCacheCons_t8_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t17_done: false,
	                   		                         	tr_SimCacheCons_t17_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t5_done: false,
	                   		                         	tr_SimCacheCons_t5_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t1_done: false,
	                   		                         	tr_SimCacheCons_t1_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t14_done: false,
	                   		                         	tr_SimCacheCons_t14_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t13_done: false,
	                   		                         	tr_SimCacheCons_t13_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t3_done: false,
	                   		                         	tr_SimCacheCons_t3_counter: 0_i32,
	                   		                         	tr_SimCacheCons_t10_done: false,
	                   		                         	tr_SimCacheCons_t10_counter: 0_i32
	                   		                         };
	                   		let mut memorystate: S_memory = S_memory {
	                   		                                	randnat: 1_i32,
	                   		                                	homeangle: 0.0_f32,
	                   		                                	av: 1.0_f32,
	                   		                                	distance: 0_i32,
	                   		                                	k1: 1_i32,
	                   		                                	timeout: 5.0_f32,
	                   		                                	pi: 3.14_f32,
	                   		                                	angle: 0.0_f32,
	                   		                                	clustersize: 0_i32,
	                   		                                	prob: 0.0_f32,
	                   		                                	randcoef: 0.5_f32,
	                   		                                	fv: 1.0_f32
	                   		                                };
	                   		// state machine loop;
	                   		while !(state).done {
	                   			{
	                   				{
	                   					let _s0: String;
	                   					_s0 = format!("{}", "- Waiting for input on channel start_S");
	                   					debug!("{}", _s0);
	                   				}
	                   				let mut inputDone: bool = false;
	                   				while !inputDone {
	                   					let mut _input_: S_input;
	                   					_input_ = recv_start_S.recv().unwrap();
	                   					{
	                   						let _s0: String;
	                   						_s0 = format!("{}", "- Read input on channel start_S");
	                   						debug!("{}", _s0);
	                   					}
	                   					match _input_ {
	                   						S_input::HomeReached => {
	                   							(inputstate).HomeReached = true;
	                   						},
	                   						S_input::ClusterSeen(_aux_) => {
	                   							(inputstate).ClusterSeen = true;
	                   							(inputstate).ClusterSeen_value = _aux_;
	                   						},
	                   						S_input::TargetSeen(_aux_) => {
	                   							(inputstate).TargetSeen = true;
	                   							(inputstate).TargetSeen_value = _aux_;
	                   						},
	                   						S_input::PuckCarried => {
	                   							(inputstate).PuckCarried = true;
	                   						},
	                   						S_input::HomeSeen => {
	                   							(inputstate).HomeSeen = true;
	                   						},
	                   						S_input::_done_ => {
	                   							inputDone = true;
	                   						},
	                   						S_input::_terminate_ => {
	                   							inputDone = true;
	                   						},
	                   					}
	                   				}
	                   			}
	                   			let mut ret: RESULT = RESULT::CONT;
	                   			while ret == RESULT::CONT {
	                   				let mut temp_: String;
	                   				temp_ = print_S_state(&mut state);
	                   				{
	                   					let _s0: String;
	                   					_s0 = format!("{}", "	");
	                   					print!("{}", _s0);
	                   					std::io::stdout().flush().unwrap();
	                   				}
	                   				{
	                   					let _s0: String;
	                   					_s0 = format!("{}", temp_);
	                   					println!("{}", _s0);	std::io::stdout().flush().unwrap();
	                   				}
	                   				ret = stm_S_step(&mut state, &mut inputstate, &mut memorystate, &send_end_S);
	                   			}
	                   			send_end_S.send(S_output::_done_).unwrap();
	                   			// update clocks;
	                   			(inputstate)._clock_T = ((inputstate)._clock_T + 1_i32);
	                   			// reset input events;
	                   			(inputstate).HomeSeen = false;
	                   			(inputstate).HomeReached = false;
	                   			(inputstate).PuckCarried = false;
	                   			(inputstate).TargetSeen = false;
	                   			(inputstate).ClusterSeen = false;
	                   			{
	                   				let _s0: String;
	                   				_s0 = format!("{}", "		Sent output _done_ on channel end_S");
	                   				debug!("{}", _s0);
	                   			}
	                   			{
	                   					let state_str = serde_json::to_string(&state).expect("Failed to serialise state of state machine stm_S");
	                   					let memory_str = serde_json::to_string(&memorystate).expect("Failed to serialise memorystate of state machine stm_S");
	                   				let snapshot = format!("{{\"state\": {},\"memory\": {}}}",state_str,memory_str);
	                   				stm_S_record.send(Some(snapshot)).expect("Failed to send data to recorder."); 
	                   			}
	                   		}
	                   	})
	                   };		
	let ctrl_C_thread_thread = {
	                             	thread::spawn(move || {
	                           		let mut terminate__: bool = false;
	                           		while !terminate__ {
	                           			{
	                           				let mut inputDone: bool = false;
	                           				while !inputDone {
	                           					{
	                           						let _s0: String;
	                           						_s0 = format!("{}", "- Waiting for input on channel start_C");
	                           						debug!("{}", _s0);
	                           					}
	                           					let mut _input_: C_C_input;
	                           					_input_ = recv_start_C.recv().unwrap();
	                           					{
	                           						let _s0: String;
	                           						_s0 = format!("{}", "- Read input on channel start_C");
	                           						debug!("{}", _s0);
	                           					}
	                           					match _input_ {
	                           						C_C_input::HomeSeen => {
	                           							send_start_S.send(S_input::HomeSeen).unwrap();
	                           						},
	                           						C_C_input::PuckCarried => {
	                           							send_start_S.send(S_input::PuckCarried).unwrap();
	                           						},
	                           						C_C_input::HomeReached => {
	                           							send_start_S.send(S_input::HomeReached).unwrap();
	                           						},
	                           						C_C_input::TargetSeen(_aux1_) => {
	                           							send_start_S.send(S_input::TargetSeen(_aux1_)).unwrap();
	                           						},
	                           						C_C_input::ClusterSeen(_aux1_) => {
	                           							send_start_S.send(S_input::ClusterSeen(_aux1_)).unwrap();
	                           						},
	                           						C_C_input::_done_ => {
	                           							send_start_S.send(S_input::_done_).unwrap();
	                           							inputDone = true;
	                           						},
	                           						C_C_input::_terminate_ => {
	                           							send_start_S.send(S_input::_terminate_).unwrap();
	                           							terminate__ = true;
	                           						},
	                           					}
	                           				}
	                           			}
	                           			{
	                           				let _s0: String;
	                           				_s0 = format!("{}", "	Finished reading inputs of controller C");
	                           				debug!("{}", _s0);
	                           			}
	                           			ctrl_C_step(&send_start_S
	                           			            , &recv_end_S);
	                           			{
	                           				let mut outputDone: bool = false;
	                           				while !outputDone {
	                           					let mut _output_: S_output;
	                           					_output_ = recv_end_S.recv().unwrap();
	                           					match _output_ {
	                           						S_output::DisableClusterWatch => {
	                           							send_end_C.send(C_C_output::DisableClusterWatch).unwrap();
	                           						},
	                           						S_output::EnableHomeWatch => {
	                           							send_end_C.send(C_C_output::EnableHomeWatch).unwrap();
	                           						},
	                           						S_output::DepositPuck => {
	                           							send_end_C.send(C_C_output::DepositPuck).unwrap();
	                           						},
	                           						S_output::DisableTargetWatch => {
	                           							send_end_C.send(C_C_output::DisableTargetWatch).unwrap();
	                           						},
	                           						S_output::stop => {
	                           							send_end_C.send(C_C_output::stop).unwrap();
	                           						},
	                           						S_output::search => {
	                           							send_end_C.send(C_C_output::search).unwrap();
	                           						},
	                           						S_output::EnableClusterWatch => {
	                           							send_end_C.send(C_C_output::EnableClusterWatch).unwrap();
	                           						},
	                           						S_output::EnableTargetWatch => {
	                           							send_end_C.send(C_C_output::EnableTargetWatch).unwrap();
	                           						},
	                           						S_output::stopsearch => {
	                           							send_end_C.send(C_C_output::stopsearch).unwrap();
	                           						},
	                           						S_output::obstacle_turn(_aux1_) => {
	                           							send_end_C.send(C_C_output::obstacle_turn(_aux1_)).unwrap();
	                           						},
	                           						S_output::r#move(_aux1_, _aux2_) => {
	                           							send_end_C.send(C_C_output::r#move(_aux1_,_aux2_)).unwrap();
	                           						},
	                           						S_output::DisableHomeWatch => {
	                           							send_end_C.send(C_C_output::DisableHomeWatch).unwrap();
	                           						},
	                           						S_output::_done_ => {
	                           							send_end_C.send(C_C_output::_done_).unwrap();
	                           							outputDone = true;
	                           						},
	                           					}
	                           				}
	                           			}
	                           		}
	                           	})
	                           };		
	control_thread.join().unwrap();
	mod_CacheConsMod_thread_thread.join().unwrap();
	stm_S_thread.join().unwrap();
	ctrl_C_thread_thread.join().unwrap();
	
	println!("System successfully terminated.");
	return Ok(())
}

	fn print_STATUS(value:STATUS)  -> String {
		match value {
			STATUS::ENTER_STATE => {
				return ("ENTER_STATE").to_string();
			},
			STATUS::ENTER_CHILDREN => {
				return ("ENTER_CHILDREN").to_string();
			},
			STATUS::EXECUTE_STATE => {
				return ("EXECUTE_STATE").to_string();
			},
			STATUS::EXIT_CHILDREN => {
				return ("EXIT_CHILDREN").to_string();
			},
			STATUS::EXIT_STATE => {
				return ("EXIT_STATE").to_string();
			},
			STATUS::INACTIVE => {
				return ("INACTIVE").to_string();
			},
		}
	}
	fn mod_CacheConsMod_step(send_start_C: &SyncSender<C_C_input>, recv_end_C: &Receiver<C_C_output>)  {
		{
			let _s0: String;
			_s0 = format!("{}", "Started step of module CacheConsMod");
			debug!("{}", _s0);
		}
		{
			let _s0: String;
			_s0 = format!("{}", "Finished step of module CacheConsMod");
			debug!("{}", _s0);
		}
	}
	fn print_S_Turning_To_Target_state(state:&mut S_Turning_To_Target_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_Turning_To_Target((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn en_Wander_Turn_1(state:&mut S_PU_Scan_Wander_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state Wander_Turn.");
			debug!("{}", _s0);
		}
		if (state).en_Wander_Turn_1_counter == 0_i32 {
			(*memory).randcoef = {
			                     	let mut rng = rand::thread_rng();
			                     	rng.gen_range(-128.0_f32..128.0_f32)
			                     };
			(*state).en_Wander_Turn_1_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).en_Wander_Turn_1_counter == 1_i32 {
			send_output.send(S_output::r#move((memory).av,0.0_f32)).unwrap();
			(*state).en_Wander_Turn_1_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).en_Wander_Turn_1_done = true;
			return RESULT::CONT;
		}
	}
	fn stm_S_Back_Up_Turning_step(state:&mut S_Back_Up_Turning_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Back_Up_Turning");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_Back_Up_Turning::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Back_Up_Turning");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_Back_Up_Turning::Move;
				}
				return RESULT::CONT;
			},
			STATES_S_Back_Up_Turning::Move => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Move");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Move");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Move");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (((inputstate)._clock_T) as f32) < (((memorystate).timeout) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_Back_Up_Turning_t0;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Move");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Move");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_Back_Up_Turning_t0 {
								(*state).state = STATES_S_Back_Up_Turning::Move;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_Back_Up_Turning::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Move is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn print_STATES_S_PU_Scan(value:STATES_S_PU_Scan)  -> String {
		match value {
			STATES_S_PU_Scan::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_PU_Scan::Wander => {
				return ("Wander").to_string();
			},
			STATES_S_PU_Scan::CalculateProb => {
				return ("CalculateProb").to_string();
			},
		}
	}
	fn tr_SimCacheCons_t2(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t2.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t2_counter == 0_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_t2_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t2_counter == 1_i32 {
			send_output.send(S_output::stopsearch).unwrap();
			(*state).tr_SimCacheCons_t2_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t2_done = true;
			return RESULT::CONT;
		}
	}
	fn en_SimCacheCons_PU_Scan_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_PU_Scan.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_PU_Scan_1_counter == 0_i32 {
			send_output.send(S_output::EnableClusterWatch).unwrap();
			(*state).en_SimCacheCons_PU_Scan_1_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).en_SimCacheCons_PU_Scan_1_counter == 1_i32 {
			send_output.send(S_output::search).unwrap();
			(*state).en_SimCacheCons_PU_Scan_1_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_PU_Scan_1_done = true;
			return RESULT::CONT;
		}
	}
	fn print_S_Look_For_Target_state(state:&mut S_Look_For_Target_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_Look_For_Target((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn tr_SimCacheCons_t13(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t13.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t13_counter == 0_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t13_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t13_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_t11(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t11.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t11_counter == 0_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t11_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t11_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_PU_Scan_t1(state:&mut S_PU_Scan_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_PU_Scan_t1.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_PU_Scan_t1_counter == 0_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_PU_Scan_t1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_PU_Scan_t1_done = true;
			return RESULT::CONT;
		}
	}
	fn print_STATES_S_Turning_Home(value:STATES_S_Turning_Home)  -> String {
		match value {
			STATES_S_Turning_Home::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_Turning_Home::Watch => {
				return ("Watch").to_string();
			},
		}
	}
	fn en_SimCacheCons_Exile_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_Exile.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_Exile_1_counter == 0_i32 {
			send_output.send(S_output::r#move(0.0_f32,(memory).fv)).unwrap();
			(*state).en_SimCacheCons_Exile_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_Exile_1_done = true;
			return RESULT::CONT;
		}
	}
	fn print_STATES_S(value:STATES_S)  -> String {
		match value {
			STATES_S::NONE => {
				return ("NONE").to_string();
			},
			STATES_S::PU_Scan => {
				return ("PU_Scan").to_string();
			},
			STATES_S::Turning_To_Target => {
				return ("Turning_To_Target").to_string();
			},
			STATES_S::Look_For_Target => {
				return ("Look_For_Target").to_string();
			},
			STATES_S::Moving_To_Target => {
				return ("Moving_To_Target").to_string();
			},
			STATES_S::Look_For_Home => {
				return ("Look_For_Home").to_string();
			},
			STATES_S::Turning_Home => {
				return ("Turning_Home").to_string();
			},
			STATES_S::Back_Up_Moving => {
				return ("Back_Up_Moving").to_string();
			},
			STATES_S::Exile => {
				return ("Exile").to_string();
			},
			STATES_S::DepositPuck => {
				return ("DepositPuck").to_string();
			},
			STATES_S::Move_To_Home => {
				return ("Move_To_Home").to_string();
			},
			STATES_S::Back_Up_Turning => {
				return ("Back_Up_Turning").to_string();
			},
		}
	}
	fn print_S_Back_Up_Turning_state(state:&mut S_Back_Up_Turning_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_Back_Up_Turning((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn stm_S_Exile_step(state:&mut S_Exile_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Exile");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_Exile::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Exile");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_Exile::Move;
				}
				return RESULT::CONT;
			},
			STATES_S_Exile::Move => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Move");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Move");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Move");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (((inputstate)._clock_T) as f32) < (((memorystate).timeout) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_Exile_t1;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Move");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Move");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_Exile_t1 {
								(*state).state = STATES_S_Exile::Move;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_Exile::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Move is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn tr_SimCacheCons_t5(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t5.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t5_counter == 0_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_t5_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t5_counter == 1_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t5_counter = 2_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t5_counter == 2_i32 {
			send_output.send(S_output::DisableTargetWatch).unwrap();
			(*state).tr_SimCacheCons_t5_counter = 3_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t5_done = true;
			return RESULT::CONT;
		}
	}
	fn en_SimCacheCons_Look_For_Target_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_Look_For_Target.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_Look_For_Target_1_counter == 0_i32 {
			send_output.send(S_output::EnableTargetWatch).unwrap();
			(*state).en_SimCacheCons_Look_For_Target_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_Look_For_Target_1_done = true;
			return RESULT::CONT;
		}
	}
	fn stm_S_step(state:&mut S_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine SimCacheCons");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of SimCacheCons");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S::PU_Scan;
				}
				return RESULT::CONT;
			},
			STATES_S::PU_Scan => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state PU_Scan");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_PU_Scan_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_PU_Scan_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_PU_Scan_1_done = false;
							(*state).en_SimCacheCons_PU_Scan_1_counter = 0_i32;
							((*state).s_PU_Scan).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state PU_Scan");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_PU_Scan_step(&mut (state).s_PU_Scan, inputstate, memorystate, &send_output);
						if ((state).s_PU_Scan).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state PU_Scan");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (((memorystate).randcoef) as f32) <= (((memorystate).prob) as f32) && (((memorystate).clustersize) as f32) > ((0_i32) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t1;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_PU_Scan_step(&mut (state).s_PU_Scan, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_PU_Scan_step(&mut (state).s_PU_Scan, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state PU_Scan");
							debug!("{}", _s0);
						}
						if ((*state).s_PU_Scan).status == STATUS::EXECUTE_STATE {
							((*state).s_PU_Scan).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_PU_Scan).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_PU_Scan_step(&mut (state).s_PU_Scan, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state PU_Scan");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t1 {
								if !(state).tr_SimCacheCons_t1_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t1(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Look_For_Target;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t1_done = false;
									(*state).tr_SimCacheCons_t1_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State PU_Scan is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::Turning_To_Target => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Turning_To_Target");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_Turning_To_Target_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_Turning_To_Target_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_Turning_To_Target_1_done = false;
							(*state).en_SimCacheCons_Turning_To_Target_1_counter = 0_i32;
							((*state).s_Turning_To_Target).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Turning_To_Target");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_Turning_To_Target_step(&mut (state).s_Turning_To_Target, inputstate, memorystate, &send_output);
						if ((state).s_Turning_To_Target).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Turning_To_Target");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (((inputstate)._clock_T) as f32) >= ((((((memorystate).angle * (memorystate).pi)) / (memorystate).av)) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t4;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if !(inputstate).TargetSeen {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t3;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_Turning_To_Target_step(&mut (state).s_Turning_To_Target, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Turning_To_Target_step(&mut (state).s_Turning_To_Target, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Turning_To_Target");
							debug!("{}", _s0);
						}
						if ((*state).s_Turning_To_Target).status == STATUS::EXECUTE_STATE {
							((*state).s_Turning_To_Target).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Turning_To_Target).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Turning_To_Target_step(&mut (state).s_Turning_To_Target, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Turning_To_Target");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t4 {
								if !(state).tr_SimCacheCons_t4_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t4(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Moving_To_Target;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t4_done = false;
									(*state).tr_SimCacheCons_t4_counter = 0_i32;
									return RESULT::CONT;
								}
							} else if (*inputstate)._transition_ == TRANSITIONS_S::S_t3 {
								if !(state).tr_SimCacheCons_t3_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t3(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::PU_Scan;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t3_done = false;
									(*state).tr_SimCacheCons_t3_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Turning_To_Target is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::Look_For_Target => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Look_For_Target");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_Look_For_Target_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_Look_For_Target_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_Look_For_Target_1_done = false;
							(*state).en_SimCacheCons_Look_For_Target_1_counter = 0_i32;
							((*state).s_Look_For_Target).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Look_For_Target");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_Look_For_Target_step(&mut (state).s_Look_For_Target, inputstate, memorystate, &send_output);
						if ((state).s_Look_For_Target).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Look_For_Target");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (inputstate).TargetSeen {
								(*memorystate).angle = (inputstate).TargetSeen_value;
								(*inputstate)._transition_ = TRANSITIONS_S::S_t2;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_Look_For_Target_step(&mut (state).s_Look_For_Target, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Look_For_Target_step(&mut (state).s_Look_For_Target, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Look_For_Target");
							debug!("{}", _s0);
						}
						if ((*state).s_Look_For_Target).status == STATUS::EXECUTE_STATE {
							((*state).s_Look_For_Target).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Look_For_Target).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Look_For_Target_step(&mut (state).s_Look_For_Target, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Look_For_Target");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t2 {
								if !(state).tr_SimCacheCons_t2_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t2(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Turning_To_Target;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t2_done = false;
									(*state).tr_SimCacheCons_t2_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Look_For_Target is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::Moving_To_Target => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Moving_To_Target");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_Moving_To_Target_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_Moving_To_Target_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_Moving_To_Target_1_done = false;
							(*state).en_SimCacheCons_Moving_To_Target_1_counter = 0_i32;
							((*state).s_Moving_To_Target).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Moving_To_Target");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_Moving_To_Target_step(&mut (state).s_Moving_To_Target, inputstate, memorystate, &send_output);
						if ((state).s_Moving_To_Target).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Moving_To_Target");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if !(inputstate).TargetSeen && !(inputstate).PuckCarried {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t6;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if (inputstate).PuckCarried {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t5;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_Moving_To_Target_step(&mut (state).s_Moving_To_Target, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Moving_To_Target_step(&mut (state).s_Moving_To_Target, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Moving_To_Target");
							debug!("{}", _s0);
						}
						if ((*state).s_Moving_To_Target).status == STATUS::EXECUTE_STATE {
							((*state).s_Moving_To_Target).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Moving_To_Target).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Moving_To_Target_step(&mut (state).s_Moving_To_Target, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Moving_To_Target");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t6 {
								if !(state).tr_SimCacheCons_t6_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t6(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::PU_Scan;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t6_done = false;
									(*state).tr_SimCacheCons_t6_counter = 0_i32;
									return RESULT::CONT;
								}
							} else if (*inputstate)._transition_ == TRANSITIONS_S::S_t5 {
								if !(state).tr_SimCacheCons_t5_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t5(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Look_For_Home;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t5_done = false;
									(*state).tr_SimCacheCons_t5_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Moving_To_Target is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::Look_For_Home => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Look_For_Home");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_Look_For_Home_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_Look_For_Home_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_Look_For_Home_1_done = false;
							(*state).en_SimCacheCons_Look_For_Home_1_counter = 0_i32;
							((*state).s_Look_For_Home).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Look_For_Home");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_Look_For_Home_step(&mut (state).s_Look_For_Home, inputstate, memorystate, &send_output);
						if ((state).s_Look_For_Home).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Look_For_Home");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (inputstate).HomeSeen {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t7;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if !(inputstate).PuckCarried {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t9;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_Look_For_Home_step(&mut (state).s_Look_For_Home, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Look_For_Home_step(&mut (state).s_Look_For_Home, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Look_For_Home");
							debug!("{}", _s0);
						}
						if ((*state).s_Look_For_Home).status == STATUS::EXECUTE_STATE {
							((*state).s_Look_For_Home).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Look_For_Home).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Look_For_Home_step(&mut (state).s_Look_For_Home, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Look_For_Home");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t7 {
								if !(state).tr_SimCacheCons_t7_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t7(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Turning_Home;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t7_done = false;
									(*state).tr_SimCacheCons_t7_counter = 0_i32;
									return RESULT::CONT;
								}
							} else if (*inputstate)._transition_ == TRANSITIONS_S::S_t9 {
								if !(state).tr_SimCacheCons_t9_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t9(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::PU_Scan;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t9_done = false;
									(*state).tr_SimCacheCons_t9_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Look_For_Home is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::Turning_Home => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Turning_Home");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_Turning_Home_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_Turning_Home_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_Turning_Home_1_done = false;
							(*state).en_SimCacheCons_Turning_Home_1_counter = 0_i32;
							((*state).s_Turning_Home).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Turning_Home");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_Turning_Home_step(&mut (state).s_Turning_Home, inputstate, memorystate, &send_output);
						if ((state).s_Turning_Home).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Turning_Home");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if !(inputstate).HomeSeen {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t8;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if (((inputstate)._clock_T) as f32) >= ((((((memorystate).angle * (memorystate).pi)) / (memorystate).av)) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t11;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if !(inputstate).PuckCarried {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t10;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_Turning_Home_step(&mut (state).s_Turning_Home, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Turning_Home_step(&mut (state).s_Turning_Home, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Turning_Home");
							debug!("{}", _s0);
						}
						if ((*state).s_Turning_Home).status == STATUS::EXECUTE_STATE {
							((*state).s_Turning_Home).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Turning_Home).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Turning_Home_step(&mut (state).s_Turning_Home, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Turning_Home");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t8 {
								if !(state).tr_SimCacheCons_t8_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t8(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Look_For_Home;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t8_done = false;
									(*state).tr_SimCacheCons_t8_counter = 0_i32;
									return RESULT::CONT;
								}
							} else if (*inputstate)._transition_ == TRANSITIONS_S::S_t11 {
								if !(state).tr_SimCacheCons_t11_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t11(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Move_To_Home;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t11_done = false;
									(*state).tr_SimCacheCons_t11_counter = 0_i32;
									return RESULT::CONT;
								}
							} else if (*inputstate)._transition_ == TRANSITIONS_S::S_t10 {
								if !(state).tr_SimCacheCons_t10_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t10(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::PU_Scan;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t10_done = false;
									(*state).tr_SimCacheCons_t10_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Turning_Home is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::Back_Up_Moving => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Back_Up_Moving");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_Back_Up_Moving_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_Back_Up_Moving_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_Back_Up_Moving_1_done = false;
							(*state).en_SimCacheCons_Back_Up_Moving_1_counter = 0_i32;
							((*state).s_Back_Up_Moving).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Back_Up_Moving");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_Back_Up_Moving_step(&mut (state).s_Back_Up_Moving, inputstate, memorystate, &send_output);
						if ((state).s_Back_Up_Moving).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Back_Up_Moving");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (((inputstate)._clock_T) as f32) >= (((memorystate).timeout) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t16;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_Back_Up_Moving_step(&mut (state).s_Back_Up_Moving, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Back_Up_Moving_step(&mut (state).s_Back_Up_Moving, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Back_Up_Moving");
							debug!("{}", _s0);
						}
						if ((*state).s_Back_Up_Moving).status == STATUS::EXECUTE_STATE {
							((*state).s_Back_Up_Moving).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Back_Up_Moving).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Back_Up_Moving_step(&mut (state).s_Back_Up_Moving, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Back_Up_Moving");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t16 {
								if !(state).tr_SimCacheCons_t16_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t16(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Back_Up_Turning;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t16_done = false;
									(*state).tr_SimCacheCons_t16_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Back_Up_Moving is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::Exile => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Exile");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_Exile_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_Exile_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_Exile_1_done = false;
							(*state).en_SimCacheCons_Exile_1_counter = 0_i32;
							((*state).s_Exile).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Exile");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_Exile_step(&mut (state).s_Exile, inputstate, memorystate, &send_output);
						if ((state).s_Exile).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Exile");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (((inputstate)._clock_T) as f32) >= (((memorystate).timeout) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t18;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_Exile_step(&mut (state).s_Exile, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Exile_step(&mut (state).s_Exile, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Exile");
							debug!("{}", _s0);
						}
						if ((*state).s_Exile).status == STATUS::EXECUTE_STATE {
							((*state).s_Exile).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Exile).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Exile_step(&mut (state).s_Exile, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Exile");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t18 {
								if !(state).tr_SimCacheCons_t18_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t18(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::PU_Scan;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t18_done = false;
									(*state).tr_SimCacheCons_t18_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Exile is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::DepositPuck => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state DepositPuck");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_DepositPuck_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_DepositPuck_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_DepositPuck_1_done = false;
							(*state).en_SimCacheCons_DepositPuck_1_counter = 0_i32;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state DepositPuck");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state DepositPuck");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if true {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t15;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state DepositPuck");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state DepositPuck");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t15 {
								if !(state).tr_SimCacheCons_t15_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t15(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Back_Up_Moving;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t15_done = false;
									(*state).tr_SimCacheCons_t15_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State DepositPuck is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::Move_To_Home => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Move_To_Home");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_Move_To_Home_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_Move_To_Home_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_Move_To_Home_1_done = false;
							(*state).en_SimCacheCons_Move_To_Home_1_counter = 0_i32;
							((*state).s_Move_To_Home).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Move_To_Home");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_Move_To_Home_step(&mut (state).s_Move_To_Home, inputstate, memorystate, &send_output);
						if ((state).s_Move_To_Home).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Move_To_Home");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (inputstate).HomeReached {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t14;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if !(inputstate).HomeSeen {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t13;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if !(inputstate).PuckCarried {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t12;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_Move_To_Home_step(&mut (state).s_Move_To_Home, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Move_To_Home_step(&mut (state).s_Move_To_Home, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Move_To_Home");
							debug!("{}", _s0);
						}
						if ((*state).s_Move_To_Home).status == STATUS::EXECUTE_STATE {
							((*state).s_Move_To_Home).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Move_To_Home).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Move_To_Home_step(&mut (state).s_Move_To_Home, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Move_To_Home");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t14 {
								if !(state).tr_SimCacheCons_t14_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t14(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::DepositPuck;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t14_done = false;
									(*state).tr_SimCacheCons_t14_counter = 0_i32;
									return RESULT::CONT;
								}
							} else if (*inputstate)._transition_ == TRANSITIONS_S::S_t13 {
								if !(state).tr_SimCacheCons_t13_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t13(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Look_For_Home;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t13_done = false;
									(*state).tr_SimCacheCons_t13_counter = 0_i32;
									return RESULT::CONT;
								}
							} else if (*inputstate)._transition_ == TRANSITIONS_S::S_t12 {
								if !(state).tr_SimCacheCons_t12_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t12(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::PU_Scan;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t12_done = false;
									(*state).tr_SimCacheCons_t12_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Move_To_Home is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S::Back_Up_Turning => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Back_Up_Turning");
							debug!("{}", _s0);
						}
						if !(state).en_SimCacheCons_Back_Up_Turning_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_SimCacheCons_Back_Up_Turning_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_SimCacheCons_Back_Up_Turning_1_done = false;
							(*state).en_SimCacheCons_Back_Up_Turning_1_counter = 0_i32;
							((*state).s_Back_Up_Turning).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Back_Up_Turning");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_Back_Up_Turning_step(&mut (state).s_Back_Up_Turning, inputstate, memorystate, &send_output);
						if ((state).s_Back_Up_Turning).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Back_Up_Turning");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (((inputstate)._clock_T) as f32) >= (((memorystate).timeout) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_t17;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_Back_Up_Turning_step(&mut (state).s_Back_Up_Turning, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Back_Up_Turning_step(&mut (state).s_Back_Up_Turning, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Back_Up_Turning");
							debug!("{}", _s0);
						}
						if ((*state).s_Back_Up_Turning).status == STATUS::EXECUTE_STATE {
							((*state).s_Back_Up_Turning).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Back_Up_Turning).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_Back_Up_Turning_step(&mut (state).s_Back_Up_Turning, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Back_Up_Turning");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_t17 {
								if !(state).tr_SimCacheCons_t17_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_t17(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S::Exile;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_t17_done = false;
									(*state).tr_SimCacheCons_t17_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Back_Up_Turning is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn stm_S_Move_To_Home_step(state:&mut S_Move_To_Home_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Move_To_Home");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_Move_To_Home::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Move_To_Home");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_Move_To_Home::Watch;
				}
				return RESULT::CONT;
			},
			STATES_S_Move_To_Home::Watch => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Watch");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Watch");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (inputstate).HomeSeen && (inputstate).PuckCarried && !(inputstate).HomeReached {
								(*inputstate)._transition_ = TRANSITIONS_S::S_Move_To_Home_t1;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Watch");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_Move_To_Home_t1 {
								(*state).state = STATES_S_Move_To_Home::Watch;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_Move_To_Home::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Watch is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn print_STATES_S_Back_Up_Turning(value:STATES_S_Back_Up_Turning)  -> String {
		match value {
			STATES_S_Back_Up_Turning::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_Back_Up_Turning::Move => {
				return ("Move").to_string();
			},
		}
	}
	fn en_SimCacheCons_Look_For_Home_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_Look_For_Home.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_Look_For_Home_1_counter == 0_i32 {
			send_output.send(S_output::EnableHomeWatch).unwrap();
			(*state).en_SimCacheCons_Look_For_Home_1_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).en_SimCacheCons_Look_For_Home_1_counter == 1_i32 {
			send_output.send(S_output::r#move((memory).av,0.0_f32)).unwrap();
			(*state).en_SimCacheCons_Look_For_Home_1_counter = 2_i32;
			return RESULT::CONT;
		} else if (state).en_SimCacheCons_Look_For_Home_1_counter == 2_i32 {
			send_output.send(S_output::search).unwrap();
			(*state).en_SimCacheCons_Look_For_Home_1_counter = 3_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_Look_For_Home_1_done = true;
			return RESULT::CONT;
		}
	}
	fn stm_S_Turning_Home_step(state:&mut S_Turning_Home_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Turning_Home");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_Turning_Home::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Turning_Home");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_Turning_Home::Watch;
				}
				return RESULT::CONT;
			},
			STATES_S_Turning_Home::Watch => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Watch");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Watch");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (inputstate).HomeSeen && (inputstate).PuckCarried && (((inputstate)._clock_T) as f32) < ((((((memorystate).homeangle * (memorystate).pi)) / (memorystate).av)) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_Turning_Home_t1;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Watch");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_Turning_Home_t1 {
								(*state).state = STATES_S_Turning_Home::Watch;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_Turning_Home::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Watch is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn print_STATES_S_PU_Scan_Wander(value:STATES_S_PU_Scan_Wander)  -> String {
		match value {
			STATES_S_PU_Scan_Wander::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_PU_Scan_Wander::Turn => {
				return ("Turn").to_string();
			},
			STATES_S_PU_Scan_Wander::Move_Forward => {
				return ("Move_Forward").to_string();
			},
		}
	}
	fn print_S_Moving_To_Target_state(state:&mut S_Moving_To_Target_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_Moving_To_Target((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn ctrl_C_step(send_start_S: &SyncSender<S_input>, recv_end_S: &Receiver<S_output>)  {
		{
			let _s0: String;
			_s0 = format!("{}", "	Started step of controller C");
			debug!("{}", _s0);
		}
	}
	fn print_STATES_S_Exile(value:STATES_S_Exile)  -> String {
		match value {
			STATES_S_Exile::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_Exile::Move => {
				return ("Move").to_string();
			},
		}
	}
	fn print_S_Exile_state(state:&mut S_Exile_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_Exile((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn print_STATES_S_Back_Up_Moving(value:STATES_S_Back_Up_Moving)  -> String {
		match value {
			STATES_S_Back_Up_Moving::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_Back_Up_Moving::Move => {
				return ("Move").to_string();
			},
		}
	}
	fn print_STATES_S_Move_To_Home(value:STATES_S_Move_To_Home)  -> String {
		match value {
			STATES_S_Move_To_Home::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_Move_To_Home::Watch => {
				return ("Watch").to_string();
			},
		}
	}
	fn tr_SimCacheCons_t17(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t17.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t17_counter == 0_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t17_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t17_counter == 1_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_t17_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t17_done = true;
			return RESULT::CONT;
		}
	}
	fn print_S_PU_Scan_state(state:&mut S_PU_Scan_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_PU_Scan((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		match (state).state {
			STATES_S_PU_Scan::Wander => {
				let mut temp_: String;
				temp_ = print_S_PU_Scan_Wander_state(&mut (state).s_Wander);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			_ => {
				return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
			}
		}
	}
	fn print_S_Back_Up_Moving_state(state:&mut S_Back_Up_Moving_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_Back_Up_Moving((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn tr_SimCacheCons_t6(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t6.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t6_counter == 0_i32 {
			send_output.send(S_output::DisableTargetWatch).unwrap();
			(*state).tr_SimCacheCons_t6_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t6_counter == 1_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t6_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t6_done = true;
			return RESULT::CONT;
		}
	}
	fn stm_S_Look_For_Home_step(state:&mut S_Look_For_Home_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Look_For_Home");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_Look_For_Home::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Look_For_Home");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_Look_For_Home::Watch;
				}
				return RESULT::CONT;
			},
			STATES_S_Look_For_Home::Watch => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Watch");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Watch");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if !(inputstate).HomeSeen {
								(*inputstate)._transition_ = TRANSITIONS_S::S_Look_For_Home_t1;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Watch");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_Look_For_Home_t1 {
								(*state).state = STATES_S_Look_For_Home::Watch;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_Look_For_Home::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Watch is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn en_SimCacheCons_Move_To_Home_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_Move_To_Home.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_Move_To_Home_1_counter == 0_i32 {
			send_output.send(S_output::r#move(0.0_f32,(memory).fv)).unwrap();
			(*state).en_SimCacheCons_Move_To_Home_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_Move_To_Home_1_done = true;
			return RESULT::CONT;
		}
	}
	fn stm_S_PU_Scan_step(state:&mut S_PU_Scan_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine PU_Scan");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_PU_Scan::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of PU_Scan");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_PU_Scan::Wander;
				}
				return RESULT::CONT;
			},
			STATES_S_PU_Scan::Wander => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Wander");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							((*state).s_Wander).status = STATUS::ENTER_STATE;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Wander");
							debug!("{}", _s0);
						}
						let mut _ret_: RESULT;
						_ret_ = stm_S_PU_Scan_Wander_step(&mut (state).s_Wander, inputstate, memorystate, &send_output);
						if ((state).s_Wander).status == STATUS::EXECUTE_STATE {
							(*state).status = STATUS::EXECUTE_STATE;
							{
								(*inputstate)._transition_ = TRANSITIONS_S::NONE;
							}
						} 
						return _ret_;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Wander");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (inputstate).ClusterSeen {
								(*memorystate).clustersize = (inputstate).ClusterSeen_value;
								(*inputstate)._transition_ = TRANSITIONS_S::S_PU_Scan_t1;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::CONT;
							} else {
								let mut _ret_: RESULT;
								_ret_ = stm_S_PU_Scan_Wander_step(&mut (state).s_Wander, inputstate, memorystate, &send_output);
								return _ret_;
							}
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_PU_Scan_Wander_step(&mut (state).s_Wander, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Wander");
							debug!("{}", _s0);
						}
						if ((*state).s_Wander).status == STATUS::EXECUTE_STATE {
							((*state).s_Wander).status = STATUS::EXIT_CHILDREN;
							return RESULT::CONT;
						} else if ((*state).s_Wander).status == STATUS::INACTIVE {
							(*state).status = STATUS::EXIT_STATE;
							return RESULT::CONT;
						} else {
							let mut _ret_: RESULT;
							_ret_ = stm_S_PU_Scan_Wander_step(&mut (state).s_Wander, inputstate, memorystate, &send_output);
							return _ret_;
						}
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Wander");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_PU_Scan_t1 {
								if !(state).tr_SimCacheCons_PU_Scan_t1_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_PU_Scan_t1(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S_PU_Scan::CalculateProb;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_PU_Scan_t1_done = false;
									(*state).tr_SimCacheCons_PU_Scan_t1_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_PU_Scan::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Wander is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S_PU_Scan::CalculateProb => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state CalculateProb");
							debug!("{}", _s0);
						}
						if !(state).en_PU_Scan_CalculateProb_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_PU_Scan_CalculateProb_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_PU_Scan_CalculateProb_1_done = false;
							(*state).en_PU_Scan_CalculateProb_1_counter = 0_i32;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state CalculateProb");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state CalculateProb");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (((memorystate).randcoef) as f32) > (((memorystate).prob) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_PU_Scan_t2;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state CalculateProb");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state CalculateProb");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_PU_Scan_t2 {
								(*state).state = STATES_S_PU_Scan::Wander;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_PU_Scan::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State CalculateProb is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn en_PU_Scan_CalculateProb_1(state:&mut S_PU_Scan_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state PU_Scan_CalculateProb.");
			debug!("{}", _s0);
		}
		if (state).en_PU_Scan_CalculateProb_1_counter == 0_i32 {
			(*memory).randcoef = {
			                     	let mut rng = rand::thread_rng();
			                     	rng.gen_range(-128.0_f32..128.0_f32)
			                     };
			(*state).en_PU_Scan_CalculateProb_1_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).en_PU_Scan_CalculateProb_1_counter == 1_i32 {
			(*memory).prob = ((((memory).k1 as f32 / (((memory).k1 as f32 + (memory).clustersize as f32)))) * (((memory).k1 as f32 / (((memory).k1 as f32 + (memory).clustersize as f32)))));
			(*state).en_PU_Scan_CalculateProb_1_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).en_PU_Scan_CalculateProb_1_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_t7(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t7.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t7_counter == 0_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_t7_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t7_counter == 1_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t7_counter = 2_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t7_counter == 2_i32 {
			send_output.send(S_output::stopsearch).unwrap();
			(*state).tr_SimCacheCons_t7_counter = 3_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t7_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_t10(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t10.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t10_counter == 0_i32 {
			send_output.send(S_output::DisableHomeWatch).unwrap();
			(*state).tr_SimCacheCons_t10_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t10_counter == 1_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t10_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t10_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_t1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t1.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t1_counter == 0_i32 {
			send_output.send(S_output::DisableClusterWatch).unwrap();
			(*state).tr_SimCacheCons_t1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t1_done = true;
			return RESULT::CONT;
		}
	}
	fn stm_S_Turning_To_Target_step(state:&mut S_Turning_To_Target_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Turning_To_Target");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_Turning_To_Target::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Turning_To_Target");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_Turning_To_Target::Watch;
				}
				return RESULT::CONT;
			},
			STATES_S_Turning_To_Target::Watch => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Watch");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Watch");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (inputstate).TargetSeen && (((inputstate)._clock_T) as f32) < ((((((memorystate).angle * (memorystate).pi)) / (memorystate).av)) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_Turning_To_Target_t5;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Watch");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_Turning_To_Target_t5 {
								(*state).state = STATES_S_Turning_To_Target::Watch;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_Turning_To_Target::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Watch is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn stm_S_PU_Scan_Wander_step(state:&mut S_PU_Scan_Wander_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Wander");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_PU_Scan_Wander::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Wander");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_PU_Scan_Wander::Turn;
				}
				return RESULT::CONT;
			},
			STATES_S_PU_Scan_Wander::Turn => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Turn");
							debug!("{}", _s0);
						}
						if !(state).en_Wander_Turn_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_Wander_Turn_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_Wander_Turn_1_done = false;
							(*state).en_Wander_Turn_1_counter = 0_i32;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Turn");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Turn");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if !(inputstate).ClusterSeen && (((inputstate)._clock_T) as f32) < ((((memorystate).randcoef * (memorystate).pi)) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_PU_Scan_Wander_t4;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if !(inputstate).ClusterSeen && (((inputstate)._clock_T) as f32) >= ((((memorystate).randcoef * (memorystate).pi)) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_PU_Scan_Wander_t1;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Turn");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Turn");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_PU_Scan_Wander_t4 {
								(*state).state = STATES_S_PU_Scan_Wander::Turn;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else if (*inputstate)._transition_ == TRANSITIONS_S::S_PU_Scan_Wander_t1 {
								if !(state).tr_SimCacheCons_PU_Scan_Wander_t1_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_PU_Scan_Wander_t1(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S_PU_Scan_Wander::Move_Forward;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_PU_Scan_Wander_t1_done = false;
									(*state).tr_SimCacheCons_PU_Scan_Wander_t1_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_PU_Scan_Wander::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Turn is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
			STATES_S_PU_Scan_Wander::Move_Forward => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Move_Forward");
							debug!("{}", _s0);
						}
						if !(state).en_Wander_Move_Forward_1_done {
							let mut _ret_: RESULT;
							_ret_ = en_Wander_Move_Forward_1(state, inputstate, memorystate, &send_output);
							return _ret_;
						} else {
							(*state).status = STATUS::ENTER_CHILDREN;
							(*state).en_Wander_Move_Forward_1_done = false;
							(*state).en_Wander_Move_Forward_1_counter = 0_i32;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Move_Forward");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Move_Forward");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if !(inputstate).ClusterSeen && (((inputstate)._clock_T) as f32) < (((memorystate).randnat) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_PU_Scan_Wander_t3;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else if !(inputstate).ClusterSeen && (((inputstate)._clock_T) as f32) >= (((memorystate).randnat) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_PU_Scan_Wander_t2;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Move_Forward");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Move_Forward");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_PU_Scan_Wander_t3 {
								(*state).state = STATES_S_PU_Scan_Wander::Move_Forward;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else if (*inputstate)._transition_ == TRANSITIONS_S::S_PU_Scan_Wander_t2 {
								if !(state).tr_SimCacheCons_PU_Scan_Wander_t2_done {
									let mut _ret_: RESULT;
									_ret_ = tr_SimCacheCons_PU_Scan_Wander_t2(state, inputstate, memorystate, &send_output);
									return _ret_;
								} else {
									(*state).state = STATES_S_PU_Scan_Wander::Turn;
									(*state).status = STATUS::ENTER_STATE;
									(*state).tr_SimCacheCons_PU_Scan_Wander_t2_done = false;
									(*state).tr_SimCacheCons_PU_Scan_Wander_t2_counter = 0_i32;
									return RESULT::CONT;
								}
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_PU_Scan_Wander::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Move_Forward is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn stm_S_Back_Up_Moving_step(state:&mut S_Back_Up_Moving_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Back_Up_Moving");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_Back_Up_Moving::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Back_Up_Moving");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_Back_Up_Moving::Move;
				}
				return RESULT::CONT;
			},
			STATES_S_Back_Up_Moving::Move => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Move");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Move");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Move");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (((inputstate)._clock_T) as f32) < (((memorystate).timeout) as f32) {
								(*inputstate)._transition_ = TRANSITIONS_S::S_Back_Up_Moving_t0;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Move");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Move");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_Back_Up_Moving_t0 {
								(*state).state = STATES_S_Back_Up_Moving::Move;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_Back_Up_Moving::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Move is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn stm_S_Look_For_Target_step(state:&mut S_Look_For_Target_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Look_For_Target");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_Look_For_Target::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Look_For_Target");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_Look_For_Target::Watch;
				}
				return RESULT::CONT;
			},
			STATES_S_Look_For_Target::Watch => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Watch");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Watch");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if !(inputstate).TargetSeen {
								(*inputstate)._transition_ = TRANSITIONS_S::S_Look_For_Target_t2;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Watch");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_Look_For_Target_t2 {
								(*state).state = STATES_S_Look_For_Target::Watch;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_Look_For_Target::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Watch is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn print_S_state(state:&mut S_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		match (state).state {
			STATES_S::PU_Scan => {
				let mut temp_: String;
				temp_ = print_S_PU_Scan_state(&mut (state).s_PU_Scan);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			STATES_S::Back_Up_Moving => {
				let mut temp_: String;
				temp_ = print_S_Back_Up_Moving_state(&mut (state).s_Back_Up_Moving);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			STATES_S::Look_For_Target => {
				let mut temp_: String;
				temp_ = print_S_Look_For_Target_state(&mut (state).s_Look_For_Target);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			STATES_S::Exile => {
				let mut temp_: String;
				temp_ = print_S_Exile_state(&mut (state).s_Exile);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			STATES_S::Turning_To_Target => {
				let mut temp_: String;
				temp_ = print_S_Turning_To_Target_state(&mut (state).s_Turning_To_Target);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			STATES_S::Moving_To_Target => {
				let mut temp_: String;
				temp_ = print_S_Moving_To_Target_state(&mut (state).s_Moving_To_Target);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			STATES_S::Move_To_Home => {
				let mut temp_: String;
				temp_ = print_S_Move_To_Home_state(&mut (state).s_Move_To_Home);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			STATES_S::Look_For_Home => {
				let mut temp_: String;
				temp_ = print_S_Look_For_Home_state(&mut (state).s_Look_For_Home);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			STATES_S::Turning_Home => {
				let mut temp_: String;
				temp_ = print_S_Turning_Home_state(&mut (state).s_Turning_Home);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			STATES_S::Back_Up_Turning => {
				let mut temp_: String;
				temp_ = print_S_Back_Up_Turning_state(&mut (state).s_Back_Up_Turning);
				return format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")"), " > "), temp_);
			},
			_ => {
				return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
			}
		}
	}
	fn en_Wander_Move_Forward_1(state:&mut S_PU_Scan_Wander_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state Wander_Move_Forward.");
			debug!("{}", _s0);
		}
		if (state).en_Wander_Move_Forward_1_counter == 0_i32 {
			(*memory).randnat = {
			                    	let mut rng = rand::thread_rng();
			                    	rng.gen_range(0_i32..128_i32)
			                    };
			(*state).en_Wander_Move_Forward_1_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).en_Wander_Move_Forward_1_counter == 1_i32 {
			send_output.send(S_output::r#move(0.0_f32,(memory).fv)).unwrap();
			(*state).en_Wander_Move_Forward_1_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).en_Wander_Move_Forward_1_done = true;
			return RESULT::CONT;
		}
	}
	fn en_SimCacheCons_Back_Up_Moving_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_Back_Up_Moving.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_Back_Up_Moving_1_counter == 0_i32 {
			send_output.send(S_output::r#move(0.0_f32,-(memory).fv)).unwrap();
			(*state).en_SimCacheCons_Back_Up_Moving_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_Back_Up_Moving_1_done = true;
			return RESULT::CONT;
		}
	}
	fn print_S_Move_To_Home_state(state:&mut S_Move_To_Home_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_Move_To_Home((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn print_S_Turning_Home_state(state:&mut S_Turning_Home_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_Turning_Home((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn en_SimCacheCons_Back_Up_Turning_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_Back_Up_Turning.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_Back_Up_Turning_1_counter == 0_i32 {
			send_output.send(S_output::r#move((memory).av,0.0_f32)).unwrap();
			(*state).en_SimCacheCons_Back_Up_Turning_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_Back_Up_Turning_1_done = true;
			return RESULT::CONT;
		}
	}
	fn print_STATES_S_Moving_To_Target(value:STATES_S_Moving_To_Target)  -> String {
		match value {
			STATES_S_Moving_To_Target::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_Moving_To_Target::Watch => {
				return ("Watch").to_string();
			},
		}
	}
	fn tr_SimCacheCons_t9(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t9.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t9_counter == 0_i32 {
			send_output.send(S_output::DisableHomeWatch).unwrap();
			(*state).tr_SimCacheCons_t9_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t9_counter == 1_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t9_counter = 2_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t9_counter == 2_i32 {
			send_output.send(S_output::stopsearch).unwrap();
			(*state).tr_SimCacheCons_t9_counter = 3_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t9_done = true;
			return RESULT::CONT;
		}
	}
	fn print_STATES_S_Look_For_Target(value:STATES_S_Look_For_Target)  -> String {
		match value {
			STATES_S_Look_For_Target::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_Look_For_Target::Watch => {
				return ("Watch").to_string();
			},
		}
	}
	fn en_SimCacheCons_Turning_To_Target_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_Turning_To_Target.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_Turning_To_Target_1_counter == 0_i32 {
			send_output.send(S_output::r#move((memory).av,0.0_f32)).unwrap();
			(*state).en_SimCacheCons_Turning_To_Target_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_Turning_To_Target_1_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_PU_Scan_Wander_t2(state:&mut S_PU_Scan_Wander_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_PU_Scan_Wander_t2.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_PU_Scan_Wander_t2_counter == 0_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_PU_Scan_Wander_t2_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_PU_Scan_Wander_t2_counter == 1_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_PU_Scan_Wander_t2_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_PU_Scan_Wander_t2_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_t3(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t3.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t3_counter == 0_i32 {
			send_output.send(S_output::DisableTargetWatch).unwrap();
			(*state).tr_SimCacheCons_t3_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t3_counter == 1_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t3_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t3_done = true;
			return RESULT::CONT;
		}
	}
	fn print_S_PU_Scan_Wander_state(state:&mut S_PU_Scan_Wander_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_PU_Scan_Wander((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn tr_SimCacheCons_t8(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t8.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t8_counter == 0_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t8_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t8_done = true;
			return RESULT::CONT;
		}
	}
	fn print_S_Look_For_Home_state(state:&mut S_Look_For_Home_state)  -> String {
		let mut temp1_: String;
		temp1_ = print_STATES_S_Look_For_Home((state).state);
		let mut temp2_: String;
		temp2_ = print_STATUS((state).status);
		return format!("{}{}",format!("{}{}",format!("{}{}",temp1_, " ("), temp2_), ")");
	}
	fn tr_SimCacheCons_t18(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t18.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t18_counter == 0_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t18_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t18_counter == 1_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_t18_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t18_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_t16(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t16.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t16_counter == 0_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t16_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t16_counter == 1_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_t16_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t16_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_t15(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t15.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t15_counter == 0_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_t15_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t15_done = true;
			return RESULT::CONT;
		}
	}
	fn print_STATES_S_Look_For_Home(value:STATES_S_Look_For_Home)  -> String {
		match value {
			STATES_S_Look_For_Home::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_Look_For_Home::Watch => {
				return ("Watch").to_string();
			},
		}
	}
	fn tr_SimCacheCons_t14(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t14.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t14_counter == 0_i32 {
			send_output.send(S_output::DisableHomeWatch).unwrap();
			(*state).tr_SimCacheCons_t14_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t14_counter == 1_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t14_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t14_done = true;
			return RESULT::CONT;
		}
	}
	fn en_SimCacheCons_Moving_To_Target_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_Moving_To_Target.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_Moving_To_Target_1_counter == 0_i32 {
			send_output.send(S_output::r#move(0.0_f32,(memory).fv)).unwrap();
			(*state).en_SimCacheCons_Moving_To_Target_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_Moving_To_Target_1_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_PU_Scan_Wander_t1(state:&mut S_PU_Scan_Wander_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_PU_Scan_Wander_t1.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_PU_Scan_Wander_t1_counter == 0_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_PU_Scan_Wander_t1_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_PU_Scan_Wander_t1_counter == 1_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_PU_Scan_Wander_t1_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_PU_Scan_Wander_t1_done = true;
			return RESULT::CONT;
		}
	}
	fn stm_S_Moving_To_Target_step(state:&mut S_Moving_To_Target_state, inputstate:&mut S_inputstate, memorystate:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "		Running step of state machine Moving_To_Target");
			debug!("{}", _s0);
		}
		match (*state).state {
			STATES_S_Moving_To_Target::NONE => {
				{
					let _s0: String;
					_s0 = format!("{}", "		Executing initial junction of Moving_To_Target");
					debug!("{}", _s0);
				}
				{
					(*state).state = STATES_S_Moving_To_Target::Watch;
				}
				return RESULT::CONT;
			},
			STATES_S_Moving_To_Target::Watch => {
				match (*state).status {
					STATUS::ENTER_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering state Watch");
							debug!("{}", _s0);
						}
						{
							(*state).status = STATUS::ENTER_CHILDREN;
							return RESULT::CONT;
						}
					},
					STATUS::ENTER_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Entering children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXECUTE_STATE;
						{
							(*inputstate)._transition_ = TRANSITIONS_S::NONE;
						}
						return RESULT::CONT;
					},
					STATUS::EXECUTE_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Executing state Watch");
							debug!("{}", _s0);
						}
						if (*inputstate)._transition_ == TRANSITIONS_S::NONE {
							if (inputstate).TargetSeen && !(inputstate).PuckCarried {
								(*inputstate)._transition_ = TRANSITIONS_S::S_Moving_To_Target_t1;
								(*state).status = STATUS::EXIT_CHILDREN;
								return RESULT::WAIT;
							} else {
								return RESULT::CONT;
							}
						} else {
							return RESULT::CONT;
						}
					},
					STATUS::EXIT_CHILDREN => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting children of state Watch");
							debug!("{}", _s0);
						}
						(*state).status = STATUS::EXIT_STATE;
						return RESULT::CONT;
					},
					STATUS::EXIT_STATE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		Exiting state Watch");
							debug!("{}", _s0);
						}
						{
							if (*inputstate)._transition_ == TRANSITIONS_S::S_Moving_To_Target_t1 {
								(*state).state = STATES_S_Moving_To_Target::Watch;
								(*state).status = STATUS::ENTER_STATE;
								return RESULT::CONT;
							} else {
								(*state).status = STATUS::INACTIVE;
								(*state).state = STATES_S_Moving_To_Target::NONE;
								return RESULT::CONT;
							}
						}
					},
					STATUS::INACTIVE => {
						{
							let _s0: String;
							_s0 = format!("{}", "		State Watch is inactive");
							debug!("{}", _s0);
						}
						return RESULT::CONT;
					},
				}
			},
		}
	}
	fn en_SimCacheCons_DepositPuck_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_DepositPuck.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_DepositPuck_1_counter == 0_i32 {
			send_output.send(S_output::DepositPuck).unwrap();
			(*state).en_SimCacheCons_DepositPuck_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_DepositPuck_1_done = true;
			return RESULT::CONT;
		}
	}
	fn print_STATES_S_Turning_To_Target(value:STATES_S_Turning_To_Target)  -> String {
		match value {
			STATES_S_Turning_To_Target::NONE => {
				return ("NONE").to_string();
			},
			STATES_S_Turning_To_Target::Watch => {
				return ("Watch").to_string();
			},
		}
	}
	fn en_SimCacheCons_Turning_Home_1(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running entry action 1 of state SimCacheCons_Turning_Home.");
			debug!("{}", _s0);
		}
		if (state).en_SimCacheCons_Turning_Home_1_counter == 0_i32 {
			send_output.send(S_output::r#move((memory).av,0.0_f32)).unwrap();
			(*state).en_SimCacheCons_Turning_Home_1_counter = 1_i32;
			return RESULT::CONT;
		} else {
			(state).en_SimCacheCons_Turning_Home_1_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_t12(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t12.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t12_counter == 0_i32 {
			send_output.send(S_output::DisableHomeWatch).unwrap();
			(*state).tr_SimCacheCons_t12_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t12_counter == 1_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t12_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t12_done = true;
			return RESULT::CONT;
		}
	}
	fn tr_SimCacheCons_t4(state:&mut S_state, inputstate:&mut S_inputstate, memory:&mut S_memory, send_output: &SyncSender<S_output>)  -> RESULT {
		{
			let _s0: String;
			_s0 = format!("{}", "Running transition action of transition SimCacheCons_t4.");
			debug!("{}", _s0);
		}
		if (state).tr_SimCacheCons_t4_counter == 0_i32 {
			(*inputstate)._clock_T = 0_i32;
			(*state).tr_SimCacheCons_t4_counter = 1_i32;
			return RESULT::CONT;
		} else if (state).tr_SimCacheCons_t4_counter == 1_i32 {
			send_output.send(S_output::stop).unwrap();
			(*state).tr_SimCacheCons_t4_counter = 2_i32;
			return RESULT::CONT;
		} else {
			(state).tr_SimCacheCons_t4_done = true;
			return RESULT::CONT;
		}
	}



