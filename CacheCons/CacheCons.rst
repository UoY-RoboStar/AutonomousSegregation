import Mathematics::*  

interface IVisibleClustersCC {
	event VisibleClustersCC: nat*Seq(ClusterData)*Seq(ObjectData)
}

interface IVisibleClustersCA {
	event VisibleClustersCA: nat*Seq(ClusterData)*Seq(ObjectData)
}

interface IVisibleClustersTW {
	event VisibleClustersTW: nat*Seq(ClusterData)*Seq(ObjectData)
}

interface IClusterWatch {
	event EnableClusterWatch
	event DisableClusterWatch
}

interface ITargetWatchFromCC {
	event EnableTargetWatch: vector(real,2)
	event DisableTargetWatch
}

interface ITargetWatchToCC {
	event InvalidTarget
	event TargetObject: vector(real,2)
}

interface IPose{
	event Pose_O: vector(real,2)
}

interface ICachePointsCC{
	event CachePointsCC: Seq(ClusterData) 
}

interface ICachePointsTW{
	event CachePointsTW: Seq(ClusterData) 
}


interface IStatus {
	event ObjectCarried
}


interface IObjectOps {
	DepositObject ( )
}

interface ObstacleEvents {
	event closestDistance : real
	event closestAngle : real
}

interface Move {
	move( cmd: vector(real,2) )
}

interface CCMove {
	event CCMove : vector(real,2)
}

interface RWMove {
	event RWMove : vector ( real , 2 )
}

interface OAMove {
	event OAMove : vector ( real , 2 )
}

interface NOAMove {
	event NOAMove : vector ( real , 2 )
}

interface IVHF {
	event EnableOA
	event DisableOA
}

interface ICurrentTypeCA{
	event CurrentTypeCA: nat
}

interface ICurrentTypeTW{
	event CurrentTypeTW: nat
}

//type O_ID = nat

datatype ObjectData {
	objectID: nat
	clusterID: nat
	graphOrder: nat
	position: vector(real,2)
	positionx: real
	positiony: real
}

datatype  ClusterData {
       clusterID: nat
       clusterSize: nat
       clusterType: nat
       centroid: vector(real,2)
} 


controller CacheConsC {
	uses ObstacleEvents  uses IStatus uses IVisibleClustersCC uses IVisibleClustersCA uses IVisibleClustersTW uses IPose requires Move requires IObjectOps  sref stm_ref0 = CacheConsS
	sref stm_ref5 = RandomWalk
	sref stm_ref4 = TargetWatch
	sref stm_ref3 = CachePointAssignment
	sref stm_ref2 = ObstacleAvoidance
	sref stm_ref1 = MoveManager

	connection CacheConsC on ObjectCarried to stm_ref0 on ObjectCarried

	connection stm_ref0 on CCMove to stm_ref1 on CCMove
	connection stm_ref0 on EnableClusterWatch to stm_ref5 on EnableClusterWatch
	connection stm_ref0 on DisableClusterWatch to stm_ref5 on DisableClusterWatch


	connection stm_ref0 on EnableOA to stm_ref2 on EnableOA
connection stm_ref0 on EnableTargetWatch to stm_ref4 on EnableTargetWatch
	connection stm_ref0 on DisableOA to stm_ref2 on DisableOA
connection CacheConsC on VisibleClustersCC to stm_ref0 on VisibleClustersCC
	connection stm_ref0 on CurrentTypeCA to stm_ref3 on CurrentTypeCA
	connection stm_ref3 on CachePointsCC to stm_ref0 on CachePointsCC
	connection stm_ref0 on DisableTargetWatch to stm_ref4 on DisableTargetWatch
	connection stm_ref4 on TargetObject to stm_ref0 on TargetObject
	connection stm_ref4 on InvalidTarget to stm_ref0 on InvalidTarget

	connection stm_ref3 on CachePointsTW to stm_ref4 on CachePointsTW
	connection CacheConsC on VisibleClustersCA to stm_ref3 on VisibleClustersCA
	connection stm_ref0 on CurrentTypeTW to stm_ref4 on CurrentTypeTW
connection CacheConsC on Pose_O to stm_ref0 on Pose_O
	connection CacheConsC on VisibleClustersTW to stm_ref4 on VisibleClustersTW

connection stm_ref5 on RWMove to stm_ref1 on RWMove

	connection stm_ref2 on OAMove to stm_ref1 on OAMove
connection CacheConsC on closestAngle to stm_ref2 on closestAngle
	connection CacheConsC on closestDistance to stm_ref2 on closestDistance
	cycleDef cycle == 1
connection stm_ref1 on NOAMove to stm_ref2 on NOAMove
}

stm CacheConsS {
	//Constants
	const lv : real = 1 // 0.25
	const av : real = 1 // 1.0
	const pi : real = 3 // 3.14159
	var randcoef : real = 100 // const randcoef : real = 100 // 100.0
	const k1 : real = 1 // 0.1
	const timeout : real = 6 // 6.0
	const timeout_PUSH: real = 1 // 0.5
	const timeout_BACKUP: real = 5 // 5.0
	const timeout_EXILE: real = 5 //5.0
	const PUSH_LV : real = 1 //0.2
	const TARGET_AV : real = 1 //0.3
	const ANGLE_DIFF : real = 1 //0.1
	const ANGLE_DIFF_TOLERANCE : real = 1 // 0.01
	const LINEAR_HOME : real = 1 // 0.8
	
	//Variables
	var done : boolean = false
	clock T	
	//var distance : real = 0.0
	var angle : real = 10 // 10.0
	var prob : real = 0 // 0.0
	var counter : nat = 0
	var data : nat*Seq(ClusterData)*Seq(ObjectData)
	var SmallestVisibleCluster: ClusterData
	var m: Seq(ClusterData)
	var leftObject : nat = 0
	var leftObjectx : real = 0 // 0.0
	var leftObjecty: real = 0 // 0.0
	var rightObject : nat = 0
	var rightObjectx : real = 0 // 0.0
	var rightObjecty: real = 0 // 0.0
	var targetObjectID : nat = 0
	var targetObjectx : real = 0 // 0.0
	var targetObjecty : real = 0 // 0.0
	var targetPosition: vector(real,2)
	var targetObject: vector(real, 2)
	var j: nat = 0
	var pose : vector ( real , 2)
	input context { uses IStatus uses IVisibleClustersCC uses ITargetWatchToCC uses IPose uses ICachePointsCC}
	output context {  requires IObjectOps uses CCMove uses IVHF uses IClusterWatch uses ITargetWatchFromCC uses ICurrentTypeCA uses ICurrentTypeTW}
	cycleDef cycle == 1
	state PU_SCAN {
		entry $ EnableClusterWatch ; $ EnableOA
	initial i0
		state ClusterSeen {
			entry j = SmallestVisibleCluster.clusterType
			state CalculateProb {
			entry randcoef = randomcoef ( ) ; prob = ( k1 / ( k1 + SmallestVisibleCluster.clusterSize ) ) * ( k1 / ( k1 + SmallestVisibleCluster.clusterSize ) )
		}
			state CalcSmallestVisibleCluster {
				entry if (data[2])[counter].clusterSize > SmallestVisibleCluster.clusterSize then SmallestVisibleCluster = (data[2])[counter]
		end
			}
			initial i0
			transition t0 {
				from CalcSmallestVisibleCluster
				to CalcSmallestVisibleCluster
				condition counter < data[1]
				action counter = counter + 1
			}
		transition t1 {
				from CalcSmallestVisibleCluster
				to CalculateProb
				condition counter == data[1]
			}
		transition t2 {
				from i0
				to CalcSmallestVisibleCluster
			}
		}
		state ChooseTargetPosition {
		initial i0
			state CalcLeftObject {
				entry if (data[3])[counter].clusterID == SmallestVisibleCluster.clusterID /\ (data[3])[counter].positionx < leftObjectx then leftObjectx = (data[3])[counter].positionx ; leftObjecty = (data[3])[counter].positiony; leftObject = (data[3])[counter].objectID end
			}
			state CalcRightObject {
				entry if (data[3])[counter].clusterID == SmallestVisibleCluster.clusterID /\ (data[3])[counter].positionx > rightObjectx then rightObjectx = (data[3])[counter].positionx; rightObjecty = (data[3])[counter].positiony ; rightObject = (data[3])[counter].objectID end
			}
			state ChooseLeftObject {
				entry targetPosition[1] = leftObjectx; targetPosition[2] = leftObjecty; done = true
			}
			state ChooseRightObject {
				entry targetPosition[1] = rightObjectx; targetPosition[2] = rightObjecty; done = true
			}

			transition t0 {
				from CalcLeftObject
				to CalcLeftObject
				condition counter < data[1]
				action counter = counter + 1
			}
		transition t1 {
				from i0
				to CalcLeftObject
			}
		transition t2 {
				from CalcLeftObject
				to CalcRightObject
				condition counter == data[1]
				action counter = 0
			}
		transition t3 {
				from CalcRightObject
				to CalcRightObject
				condition counter < data[1]
				action counter = counter +1
			}
		transition t4 {
				from CalcRightObject
				to ChooseLeftObject
				condition (data[3])[leftObject].graphOrder < (data[3])[rightObject].graphOrder
			}
			transition t5 {
				from CalcRightObject
				to ChooseRightObject
				condition (data[3])[rightObject].graphOrder < (data[3])[leftObject].graphOrder
			}
		}
		state WaitForCluster {
		}
		transition t0 {
			from i0
			to WaitForCluster
		}
	transition t1 {
			from WaitForCluster
			to ClusterSeen
			condition 

			$     VisibleClustersCC ? data
			action 
		counter = 1 ; SmallestVisibleCluster . clusterSize = 0 ; $ DisableClusterWatch ; $ CCMove  ! [| 0 , 0 |] ; exec
		}
		transition t2 {
			from ClusterSeen
			to WaitForCluster
			exec
			condition not ( randcoef <= prob /\ SmallestVisibleCluster . clusterSize > 0 ) 
		}
	
	transition t3 {
			from ClusterSeen
			to ChooseTargetPosition
			condition randcoef <= prob /\ SmallestVisibleCluster . clusterSize > 0
			action 
		counter = 1
		}
	transition t4 {
			from WaitForCluster
			to WaitForCluster
			condition not $ VisibleClustersCC
			action exec
		}
		transition t5 {
			from ClusterSeen
			to ClusterSeen
			condition SmallestVisibleCluster . clusterSize > 0 /\ randcoef > prob
			action exec
		}
	}
	initial i0

	state DE_BACKUP {
	}
	state EXILE {
	state EXILE_Turn {
		}
		state EXILE_LinearMove {
		}
		initial i0
		transition t0 {
			from i0
			to EXILE_Turn
		}
		transition t1 {
			from EXILE_Turn
			to EXILE_LinearMove
			exec
			condition since(T) > pi / (2 * av)
			action 
		$ CCMove ! [| 
			0 //0.0 
			, lv
		|]
		}
		transition t2 {
			from EXILE_Turn
			to EXILE_Turn
			exec
			condition since(T) <= pi / (2 * av)
			action 
		$  CCMove ! [| av , 
			0 // 0.0
		|]
		}
	transition t3 {
			from EXILE_LinearMove
			to EXILE_LinearMove
			exec
			condition since ( T ) < timeout_EXILE
			action $ CCMove ! [| 0 // 0.0 
				, lv
			|]
		}
	}
	state DE_PUSH {
		entry $ CCMove ! [| 0 //0.0 
			, PUSH_LV * lv //0.2 * lv -> replaced by PUSH_LV
		|]
	}

	state PU_TARGET {
		state MoveToTarget {
			state TurnToTarget {
				entry $ DisableOA ; if angle > 0 then $ CCMove ! [| 
					TARGET_AV * av, // 0.3 * av , 
					0
				|] end ; if angle < 0 then $ CCMove ! [|
					- TARGET_AV * av, //- 0.3 * av , 
					0
				|] end
			}
			state LinearMoveToTarget {
				entry $ CCMove ! [| 0, lv |] // (0.0 , lv )
			}
			initial i0
			transition t0 {
				from TurnToTarget
				to TurnToTarget
				condition not $ TargetObject
				action 
			exec
			}
			transition t1 {
				from TurnToTarget
				to LinearMoveToTarget
				exec
				condition abs ( angle - 1 ) <= ANGLE_DIFF_TOLERANCE //0.01
				action 
			
			$ EnableOA ; # T ; $ CCMove ! [|0,0|] //( 0.0 , 0.0 )
			}
			transition t2 {
				from LinearMoveToTarget
				to LinearMoveToTarget
				exec
			}
		transition t3 {
				from TurnToTarget
				to TurnToTarget
				exec
				condition $ Pose_O ? pose /\ abs ( angle - ANGLE_DIFF ) > ANGLE_DIFF_TOLERANCE // abs ( angle - 0.1 ) > 0.01
				action targetPosition = targetObject + pose ; angle = calculate_turn_angle ( pose , targetPosition )
				}
		transition t4 {
				from TurnToTarget
				to TurnToTarget
				condition 
				not ( abs ( angle - ANGLE_DIFF ) <= ANGLE_DIFF_TOLERANCE /\ not ( abs ( angle - ANGLE_DIFF ) > ANGLE_DIFF_TOLERANCE ) ) // not ( abs ( angle - 0.1 ) <= 0.01 /\ not ( abs ( angle - 0.1 ) > 0.01 ) )
				action 
			exec
			}
			transition t5 {
				from LinearMoveToTarget
				to TurnToTarget
				exec
				condition not $ InvalidTarget /\ since ( T ) >= timeout /\ $ TargetObject ? targetObject /\ $ Pose_O ? pose
				action targetPosition = targetObject + pose
			}
		transition t6 {
				from i0
				to LinearMoveToTarget
			}
		}
		initial i0
		transition t0 {
			from i0
			to MoveToTarget
		}
	}
	state HOMING {
		//entry distance = distance ( m [ j ] , ( pose [ 1 ] , pose [ 2 ] ) )
	state TurnToHome {
			entry if angle > 0 
				then $ CCMove ! [| TARGET_AV * av , 0 |] // ( 0.3 * av , 0.0 ) 
				else $ CCMove ! [| - TARGET_AV * av , 0 |] // ( - 0.3 * av , 0.0 ) 
				end
		}
		initial i0
		state LinearMoveToHome {
		}
		state WaitForPose {
		}
		transition t0 {
			from i0
			to WaitForPose
		}
		transition t1 {
			from TurnToHome
			to LinearMoveToHome
			exec
			condition $ Pose_O ? pose /\ abs ( angle - ANGLE_DIFF ) <= ANGLE_DIFF_TOLERANCE // abs ( angle - 0.1 ) <= 0.01
			action $ CCMove ! [| 0 , 0 |] // ( 0.0 , 0.0 ) 
				; # T ; $ EnableOA
		}
		transition t2 {
			from TurnToHome
			to TurnToHome
			exec
			condition abs ( angle - ANGLE_DIFF ) > ANGLE_DIFF_TOLERANCE // abs ( angle - 0.1 ) > 0.01
				/\ $ Pose_O ? pose
				action angle = calculate_turn_angle ( pose , m [ j ] . centroid )
		}
		transition t3 {
			from LinearMoveToHome
			to LinearMoveToHome
			exec
			condition since ( T ) < timeout
			action $ CCMove ! [| 0 , LINEAR_HOME * lv |] //( 0.0 , 0.8 * lv )
		}
		transition t4 {
			from WaitForPose
			to TurnToHome
			exec
			condition $ Pose_O ? pose
			action angle = calculate_turn_angle ( pose , targetPosition )
		}
		transition t5 {
			from WaitForPose
			to WaitForPose
			exec
			condition not $ Pose_O
		}
		transition t6 {
			from LinearMoveToHome
			to TurnToHome
			exec
			condition since ( T ) >= timeout
			action $ CCMove ! [| 0, 0 |] ; // ( 0.0 , 0.0 )
				angle = calculate_turn_angle ( pose , m [ j ] . centroid ) ; $ DisableOA
		}
	}
	transition t0 {
		from i0
		to PU_SCAN
	}
	transition t5 {
		from PU_TARGET
		to HOMING
		condition $ ObjectCarried /\ $ CachePointsCC ? m
		action 
	$ CCMove ! [| 0 , 0 |] ; // ( 0.0 , 0.0 ) ;
	$ DisableTargetWatch ; exec
	}
	transition t9 {
		from HOMING
		to PU_SCAN
		exec
		condition 
	
		
		
		not $ ObjectCarried
		action 
	
	 
	$  CCMove ! [| 0, 0 |] // ( 0.0 , 0.0 )
	}
	transition t1 {
		from PU_SCAN
		to PU_TARGET
		condition done == true
		action 
	
	$ CurrentTypeCA ! j ; $ DisableClusterWatch ;
	$ CCMove ! [| 0 , 0 |] ; // ( 0.0 , 0.0 )
	# T ; done = false ; angle = 10 ; $ EnableTargetWatch ! targetPosition ; exec
	}
	transition t15 {
		from DE_PUSH
		to DE_BACKUP
		exec
		condition since ( T ) >= timeout_PUSH
		action 
	$ CCMove ! [| 0 , 0 |] ; // ( 0.0 , 0.0 )
	$ DepositObject ( ) ; # T
	}
	transition t18 {
		from EXILE
		to PU_SCAN
		exec
		condition since ( T ) >= timeout_EXILE
		action 
	
	$ CCMove ! [| 0 , 0 |] // ( 0.0 , 0.0 )
	}
transition t19 {
		from PU_TARGET
		to PU_SCAN
		condition $ InvalidTarget \/ since ( T ) > timeout 
		action 
	
	
	
	
	$ CCMove ! [| 0 , 0 |] ; // ( 0.0 , 0.0 ) 
	$ EnableOA ; $ DisableTargetWatch ; exec
	}
transition t2 {
		from DE_BACKUP
		to EXILE
		exec
		condition ( not $ ObjectCarried ) /\ since ( T ) >= timeout_BACKUP
		action $ CCMove ! [| 0 , 0 |] ; // ( 0.0 , 0.0 )
		# T
	}
transition t3 {
		from DE_PUSH
		to DE_PUSH
		exec
		condition since ( T ) < timeout_PUSH
	}
	transition t6 {
		from PU_SCAN
		to HOMING
		condition $ ObjectCarried /\ $ CachePointsCC ? m
		action $ CCMove ! [| 0 , 0 |] ; // ( 0.0 , 0.0 )
		$ DisableTargetWatch ; exec
	}
transition t7 {
		from HOMING
		to DE_PUSH
		exec
		condition distance ( m [ j ] . centroid , [| pose [ 1 ] , pose [ 2 ] |] ) < LINEAR_HOME // 0.8
		action # T ; $ CCMove ! [| 0 , 0 |] // ( 0.0 , 0.0 )
	}
transition t4 {
		from DE_BACKUP
		to DE_BACKUP
		exec
		condition $ ObjectCarried
		action $ DepositObject ( )
	}
	transition t8 {
		from DE_BACKUP
		to DE_BACKUP
		exec
		condition not $ ObjectCarried /\ since ( T ) < timeout_BACKUP
		action $ CCMove ! [| 0 , - lv |] // ( 0 , - lv )
	}
}

stm ObstacleAvoidance {
	const pi : real  //3.14159
	const DISTANCE : real // 0.4
	const min_range : real //0.1
	const max_range : real //0.4
	var closest_angle : real = 0
	var closest_distance : real = 0
	var av : real = 1 // 0.7
	var lv : real = 1 // 0.07
	var NOA_Move : vector ( real , 2 )
	var current_speed : real
	clock T
	input context { uses ObstacleEvents uses IVHF uses NOAMove}
	output context { uses OAMove }
	cycleDef cycle == 1
	state OAEnabled {
		entry $OAMove![|av,lv|]
	}
	state OADisabled {
	}
	initial i1
	junction j0
	junction j1
	junction j2
	junction j3
	transition t7 {
		from OAEnabled
		to OADisabled
	exec
		condition  $ 
		DisableOA
	}
	transition t8 {
		from OADisabled
		to OAEnabled
	exec
		condition  $ 
		EnableOA
	}
	transition t1 {
		from OADisabled
		to OADisabled
		condition not $ EnableOA
		action exec
	}
transition t2 {
		from i1
		to OAEnabled
	}
	transition t0 {
		from j1
		to j3
		condition ( closest_distance >= min_range ) /\ ( closest_distance < max_range ) /\ (abs ( closest_angle) <= 90 )
		action current_speed = NOA_Move [ 2 ]
	}
	transition t3 {
		from j0
		to OAEnabled
		condition ( closest_angle > 0 )
		action av = ( closest_angle - 100 ) * pi / 180
	}
	transition t4 {
		from j0
		to OAEnabled
	condition ( closest_angle <= 0 )
		action av = ( closest_angle + 100 ) * pi / 180
	}
	transition t5 {
		from OAEnabled
		to j1
		exec
		condition $ closestAngle ? closest_angle /\ $ closestDistance ? closest_distance /\ $ NOAMove ? NOA_Move
	}
transition t6 {
		from j1
		to OAEnabled
		condition not ( ( closest_distance >= min_range ) /\ ( closest_distance < max_range ) /\ (abs ( closest_angle) <= 90 ) )
		action lv = NOA_Move [ 2 ] ; av = NOA_Move [ 1 ]
	}
	transition t9 {
		from j0
		to j2
		condition ( closest_angle < 30 )
	}
	transition t10 {
		from j2
		to OAEnabled
		condition ( closest_distance < DISTANCE ) // 0.4
		action lv = - DISTANCE // 0.4
	}
	transition t11 {
		from j2
		to OAEnabled
		condition ( closest_distance >= DISTANCE ) // 0.4
		action lv = 0 // 0.0
	}
	transition t12 {
		from j3
		to j0
		condition ( closest_distance > DISTANCE ) // 0.4
		action lv = current_speed / 2
	}
	transition t13 {
		from j3
		to j0
		condition ( closest_distance <= DISTANCE ) // 0.4
		action lv = 0 // 0.0
	}
}

stm MoveManager {
	var NOAcmd : vector ( real , 2 ) = [|0,0|] // const NOAcmd : vector ( real , 2 )
	var cmd: vector(real, 2) = [|0,0|]
	input context { uses CCMove uses OAMove uses RWMove }
	output context { requires Move uses NOAMove }
	cycleDef cycle == 1
	initial i0
state MoveHandler {
		entry $ move ( cmd ) ; $ NOAMove ! NOAcmd
	}
	transition t0 {
		from MoveHandler
		to MoveHandler
		exec
		condition $ RWMove ? cmd /\ $ RWMove ? NOAcmd /\ not $ OAMove /\ not $ CCMove
	}
	transition t1 {
		from i0
		to MoveHandler
		//exec
	}
	transition t2 {
		from MoveHandler
		to MoveHandler
		exec
		condition $ CCMove ? cmd /\ $ CCMove ? NOAcmd /\ not $ OAMove
	}
transition t3 {
		from MoveHandler
		to MoveHandler
	exec
		condition $ OAMove ? cmd /\ $ RWMove ? NOAcmd /\ not $ CCMove
	}
	transition t4 {
		from MoveHandler
		to MoveHandler
		exec
		condition $ OAMove ? cmd /\ $ CCMove ? NOAcmd /\ not $ RWMove
	}
}

stm CachePointAssignment {
	var j : nat
	var counter : nat = 1
	var L : ClusterData
	var data : nat * Seq( ClusterData ) * Seq( ObjectData )
	var m : Seq( ClusterData )
	input context { uses IVisibleClustersCA uses ICurrentTypeCA}
	output context { uses ICachePointsCC uses ICachePointsTW }
	cycleDef cycle == 1
initial i0

	state CalculateL {
		entry if ( data [2])[ counter ].clusterSize > L .clusterSize then L = (data [2])[ counter ] end
	}
	state DecideNewCachePoint {
	}
	state UpdateCache {
		entry m [ L . clusterType ] = L
	}
	state UpdateCurrentCache {
		entry if ( data [2])[ counter ].clusterType == j then (if m [ j ].clusterSize < ( data [2])[ counter ].clusterSize then m[j] = (data [2])[ counter ] end) end
	}
	state DetectCluster {
	}

	transition t0 {
		from DetectCluster
		to CalculateL
	condition 
		$  VisibleClustersCA ? data
		action exec ; counter = 0
	}
transition t1 {
		from CalculateL
		to CalculateL
		condition counter <= data [1]
		action counter = counter + 1
	}
transition t2 {
		from CalculateL
		to DecideNewCachePoint
		condition counter > data [1]
	}
	transition t3 {
		from DecideNewCachePoint
		to UpdateCache
		condition $CurrentTypeCA?j/\L .clusterSize > m [ L .clusterType].clusterSize
	}
	transition t4 {
		from DecideNewCachePoint
		to UpdateCurrentCache
		condition L.clusterSize <= m [ L .clusterType].clusterSize
	action counter = 1
	}
	transition t5 {
		from UpdateCache
		to UpdateCurrentCache
		condition $CurrentTypeCA?j
	action counter = 1
	}
	transition t6 {
		from UpdateCurrentCache
		to UpdateCurrentCache
		condition counter <= data [1]
		action counter = counter + 1
	}
transition t7 {
		from UpdateCurrentCache
		to DetectCluster
		exec
		condition counter > data [1]
		action $CachePointsCC!m; $CachePointsTW!m
	}
transition t8 {
		from i0
		to DetectCluster
	}
transition t9 {
		from DetectCluster
		to DetectCluster
		exec
		condition not $ VisibleClustersCA
	}
}
stm TargetWatch {
	var targetObjectxy : vector ( real , 2 )
	var closestTargetObject: ObjectData
	var closestTargetObjectPosition: vector ( real , 2 )
	var counter: nat  = 0
	var data : nat*Seq(ClusterData)*Seq(ObjectData)
	const ObjectThreshold: real = 1 // 0.01
	var done : boolean = false
	var validObject : boolean = true
	var targetType: nat
	var m: Seq(ClusterData)
	input context {uses ITargetWatchFromCC uses IVisibleClustersTW uses ICurrentTypeTW uses ICachePointsTW}
	output context {uses ITargetWatchToCC }
	cycleDef cycle == 1
initial i0
	state DetectingTarget {
	state ValidateTargetObject {
		entry if  (closestTargetObject.clusterID == m[targetType].clusterID) \/ (distance(closestTargetObject.position,targetObjectxy) < ObjectThreshold) then validObject = false end 
		}
		state PublishTargetObject {
			entry closestTargetObjectPosition = closestTargetObject.position
		}
	state CalculateTargetObject {
		entry if distance((data[3])[counter].position,targetObjectxy) < distance(closestTargetObject.position,targetObjectxy)  then closestTargetObject = (data[3])[counter]  end
		}
		initial i0
		state WaitForDisableTargetWatch {
		}
		transition t0 {
			from CalculateTargetObject
			to ValidateTargetObject
		condition (counter == 1) /\ $CachePointsTW?m
		}
		transition t1 {
			from ValidateTargetObject
			to PublishTargetObject
			exec
			condition validObject == true
		}
		transition t2 {
			from i0
			to CalculateTargetObject
		}
	transition t3 {
			from PublishTargetObject
			to CalculateTargetObject
			exec
			action counter = 0;$ TargetObject!closestTargetObjectPosition
		}
	transition t4 {
			from ValidateTargetObject
			to WaitForDisableTargetWatch
			exec
			condition validObject == false
			action $ InvalidTarget
		}
	transition t5 {
			from CalculateTargetObject
			to CalculateTargetObject
			condition counter < data[1]
			action counter = counter + 1
		}
	}
	state WaitingForTarget {
	}
	transition t0 {
		from i0
		to WaitingForTarget
	}
transition t1 {
		from WaitingForTarget
		to DetectingTarget
		exec
		condition $ EnableTargetWatch ? targetObjectxy /\ $ VisibleClustersTW ? data /\ $CurrentTypeTW?targetType
	}
	transition t2 {
		from DetectingTarget
		to WaitingForTarget
		exec
		condition  $ 
	DisableTargetWatch
	}
	transition t3 {
		from WaitingForTarget
		to WaitingForTarget
		exec
		condition  not 
		$ EnableTargetWatch
	}
}

stm RandomWalk {
	const lv : real //0.07
	const av : real // 0.6
	const pi : real // 3.14159
	clock T
	var randcoef : real = 1 //const randcoef : real = 0.2
	var sign : nat = 1
	input context {uses IClusterWatch }
	output context {  uses RWMove}
	cycleDef cycle == 1
	initial i0
	state Wander {
		initial i0
			state Turn {
			entry randcoef = randomcoef ( ) ; $ RWMove ! [| av * sign , 0 |] // 0.0
			}
			state Move_Forward {
			entry $ RWMove ! [| 0 , lv |] // ( 0.0 , lv )
			}
			transition t0 {
				from i0
				to Turn
			}
			transition t1 {
				from Turn
				to Move_Forward
			exec
			condition since ( T ) >= randcoef
				action 
			
			
			$ RWMove ! [| 0 , 0 |] ; // ( 0.0 , 0.0 )
			 # T ; randcoef = randomcoef ( ) ; sign = random_sign ( )
		}
			transition t2 {
				from Move_Forward
				to Turn
			exec
			condition since ( T ) >= randcoef
				action 
			
			
		$ RWMove ! [| 0 , 0 |] ; // ( 0.0 , 0.0 )
		 # T ; randcoef = randomcoef ( )
		}
			transition t3 {
				from Move_Forward
				to Move_Forward
			exec
			condition since ( T ) < randcoef
			}
			transition t4 {
				from Turn
				to Turn
			exec
			condition since ( T ) < randcoef
			}
	}
	state Wait {
	}
	transition t0 {
		from i0
		to Wait
	}
transition t1 {
		from Wait
		to Wait
		exec
		condition not $ EnableClusterWatch
	}
	transition t2 {
		from Wait
		to Wander
		condition $ EnableClusterWatch
		action exec
	}
	transition t3 {
		from Wander
		to Wait
		exec
		condition  $ 
		DisableClusterWatch
		action $ RWMove ! [| 0 , 0 |] // ( 0.0 , 0.0 )
	}
}

function sqrt(v:real): real {
	precondition v >= 0
	postcondition result*result == v
}

function distance(x1:vector(real,2), x2:vector(real,2)): real {
    postcondition result == sqrt((x2[1] - x1[1])*(x2[1] - x1[1]) - (x2[2] - x1[2])*(x2[2]- x1[2]))
    }
    
function L2(x: vector(real,2)): real{
	postcondition result == sqrt((x[1]*x[1] + x[2] * x[2]))
}

function dot(x1: vector(real,2), x2: vector(real,2)): real{
	postcondition result == x1[1]*x2[1] + x1[2]*x2[2]
}

function unit(x: vector(real,2)) : vector(real,2){
	postcondition result[1] == x[1]/L2(x)
	postcondition result[2] == x[2]/L2(x)
}

function angle_between(x1: vector(real,2), x2: vector(real,2)): real{
	postcondition result == acos(dot(unit(x1), unit(x2)))
}

function calculate_turn_angle(x1: vector(real,2), x2: vector(real,2)): real{
	postcondition result == 0
}

function acos(x: real): real{
	
}

function abs(x: real): real{
	postcondition result == sqrt(x*x)
}

function randcoef(): real{
postcondition 0 <= result <= 1	
}

function randnat(): real{
postcondition 0 <= result <= 6 	
}
function random_sign(): nat{
	postcondition result == 1
}
    
module CacheCons {
	connection TurtleBot on ObjectCarried to ctrl_ref0 on ObjectCarried ( _async )
	//connection TurtlebotGazebo on ClusterSeen to ctrl_ref0 on ClusterSeen ( _async )
	robotic platform TurtleBot {
		uses ObstacleEvents uses IStatus provides IObjectOps uses IVisibleClustersCC uses IVisibleClustersCA uses IVisibleClustersTW uses IPose provides Move }

	cref ctrl_ref0 = CacheConsC
	cycleDef cycle == 1

	connection TurtleBot on VisibleClustersCC to ctrl_ref0 on VisibleClustersCC ( _async )
connection TurtleBot on Pose_O to ctrl_ref0 on Pose_O ( _async )
	connection TurtleBot on VisibleClustersTW to ctrl_ref0 on VisibleClustersTW ( _async )
	connection TurtleBot on VisibleClustersCA to ctrl_ref0 on VisibleClustersCA ( _async )
connection TurtleBot on closestDistance to ctrl_ref0 on closestDistance ( _async )
connection TurtleBot on closestAngle to ctrl_ref0 on closestAngle ( _async )
}

