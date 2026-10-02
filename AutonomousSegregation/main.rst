datatype Vec2 {
	x : real
	y : real
}

datatype ObjectData {
	objectID : nat
	clusterID : nat
	graphOrder : nat
	position : Vec2
	positionx : real
	positiony : real
}


datatype ClusterData { clusterID : nat
	clusterSize : nat
	clusterType : nat
	centroid : Vec2
}

interface IVisibleClustersCC {
    event VisibleClustersCC : nat * Seq( ClusterData ) * Seq( ObjectData )
}

interface IVisibleClustersCA {
    event VisibleClustersCA : nat * Seq( ClusterData ) * Seq( ObjectData )
}

interface IVisibleClustersTW {
    event VisibleClustersTW : nat * Seq( ClusterData ) * Seq( ObjectData )
}

interface IClusterWatch {
    event EnableClusterWatch
    event DisableClusterWatch
}

interface ITargetWatchFromCC {
    event EnableTargetWatch : Vec2
    event DisableTargetWatch
}

interface ITargetWatchToCC {
    event InvalidTarget
    event TargetObject : Vec2
}

interface ICoord {
    event Coord_O : Vec2
}

interface ICachePointsCC {
    event CachePointsCC : Seq( ClusterData )
}

interface ICachePointsTW {
    event CachePointsTW : Seq( ClusterData )
}

interface IStatus {
    event ObjectCarried
}

interface IObjectOps {
	PickUpObject ( )
    DepositObject ( )
}

interface ObstacleEvents {
    event closestDistance : real
    event closestAngle : real
}

interface Move {
    move ( cmd : Vec2 )
}

interface CCMove {
    event CCMove : Vec2
}

interface RWMove {
    event RWMove : Vec2
}

interface OAMove {
    event OAMove : Vec2
}

interface NOAMove {
    event NOAMove : Vec2
}

interface IOA {
    event EnableOA
    event DisableOA
}

interface ICurrentTypeCA {
    event CurrentTypeCA : nat
}

interface ICurrentTypeTW {
    event CurrentTypeTW : nat
}


controller CacheConsC {
    uses ObstacleEvents uses IStatus uses IVisibleClustersCC uses IVisibleClustersCA uses IVisibleClustersTW uses ICoord requires Move requires IObjectOps sref stm_ref0 = CacheConsS
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
    connection CacheConsC on Coord_O to stm_ref0 on Coord_O
    connection CacheConsC on VisibleClustersTW to stm_ref4 on VisibleClustersTW
    connection stm_ref5 on RWMove to stm_ref1 on RWMove
    connection stm_ref2 on OAMove to stm_ref1 on OAMove
    connection CacheConsC on closestAngle to stm_ref2 on closestAngle
    connection CacheConsC on closestDistance to stm_ref2 on closestDistance
    connection stm_ref1 on NOAMove to stm_ref2 on NOAMove
    cycleDef cycle == 1
}

stm CacheConsS {
    const lv : real // 0.25
    const av : real // 1.0
    const pi : real = 3 // 3.14159
    var randcoef : real // 100
    const k1 : real  // 0.1
    const timeout : real // 6
    const timeout_PUSH : real // 0.5
    const timeout_BACKUP : real // 5.0
    const timeout_EXILE : real // 5.0
    const PUSH_LV : real // 0.2
    const TARGET_AV : real // 0.3
    const ANGLE_DIFF : real // 0.1
    const ANGLE_DIFF_TOLERANCE : real // 0.01
    const LINEAR_HOME : real //0.8
    var done : boolean
    var angle : real // 10.0
    var prob : real // 0.0
    var counter : nat = 0
    var data : nat * Seq( ClusterData ) * Seq( ObjectData )
    var SmallestVisibleCluster : ClusterData
    var m : Seq( ClusterData )
    var leftObject : nat // 0
    var leftObjectx : real // 0.0
    var leftObjecty : real // 0.0
    var rightObject : nat // 0
    var rightObjectx : real // 0.0
    var rightObjecty : real // 0.0
    var targetObjectID : nat 
    var targetObjectx : real // 0.0
    var targetObjecty : real // 0.0
    var targetPosition : Vec2
    var targetObject : Vec2
    var ct : nat
    var coord : Vec2
    var moveCmd : Vec2
    clock T
    input context { uses IStatus uses IVisibleClustersCC uses ITargetWatchToCC uses ICoord uses ICachePointsCC }
    output context { uses CCMove uses IOA uses IClusterWatch uses ITargetWatchFromCC uses ICurrentTypeCA uses ICurrentTypeTW requires IObjectOps }
    cycleDef cycle == 1
    state PU_SCAN {
        initial i0
        state ClusterSeen {
            state CalculateProb {
                entry randcoef = randomcoef ( ) ; prob = ( k1 / ( k1 + SmallestVisibleCluster . clusterSize ) ) * ( k1 / ( k1 + SmallestVisibleCluster . clusterSize ) ) ; ct = SmallestVisibleCluster . clusterType
            }
            state CalcSmallestVisibleCluster {
                entry if ( data [ 2 ] ) [ counter ] . clusterSize > SmallestVisibleCluster . clusterSize then SmallestVisibleCluster = ( data [ 2 ] ) [ counter ] end
            }
            initial i0
            transition t0 {
                from CalcSmallestVisibleCluster
                to CalcSmallestVisibleCluster
                condition counter < data [ 1 ]
                action counter = counter + 1
            }
            transition t1 {
                from CalcSmallestVisibleCluster
                to CalculateProb
                condition counter == data [ 1 ]
            }
            transition t2 {
                from i0
                to CalcSmallestVisibleCluster
            }
        }
        state ChooseTargetPosition {
            initial i0
            state CalcLeftObject {
                entry if ( data [ 3 ] ) [ counter ] . clusterID == SmallestVisibleCluster . clusterID /\ ( data [ 3 ] ) [ counter ] . positionx < leftObjectx then leftObjectx = ( data [ 3 ] ) [ counter ] . positionx ; leftObjecty = ( data [ 3 ] ) [ counter ] . positiony ; leftObject = ( data [ 3 ] ) [ counter ] . objectID end
            }
            state CalcRightObject {
                entry if ( data [ 3 ] ) [ counter ] . clusterID == SmallestVisibleCluster . clusterID /\ ( data [ 3 ] ) [ counter ] . positionx > rightObjectx then rightObjectx = ( data [ 3 ] ) [ counter ] . positionx ; rightObjecty = ( data [ 3 ] ) [ counter ] . positiony ; rightObject = ( data [ 3 ] ) [ counter ] . objectID end
            }
            state ChooseLeftObject {
                entry targetPosition . x = leftObjectx ; targetPosition . y = leftObjecty ; done = true
            }
            state ChooseRightObject {
                entry targetPosition . x = rightObjectx ; targetPosition . y = rightObjecty ; done = true
            }
            transition t0 {
                from CalcLeftObject
                to CalcLeftObject
                condition counter < data [ 1 ]
                action counter = counter + 1
            }
            transition t1 {
                from i0
                to CalcLeftObject
            }
            transition t2 {
                from CalcLeftObject
                to CalcRightObject
                condition counter == data [ 1 ]
                action counter = 1
            }
            transition t3 {
                from CalcRightObject
                to CalcRightObject
                condition counter < data [ 1 ]
                action counter = counter + 1
            }
            transition t4 {
                from CalcRightObject
                to ChooseLeftObject
                condition leftObject > 0 /\ rightObject > 0 /\ ( data [ 3 ] ) [ leftObject ] . graphOrder < ( data [ 3 ] ) [ rightObject ] . graphOrder
            }
            transition t5 {
                from CalcRightObject
                to ChooseRightObject
                condition leftObject > 0 /\ rightObject > 0 /\ ( data [ 3 ] ) [ rightObject ] . graphOrder < ( data [ 3 ] ) [ leftObject ] . graphOrder
            }
            transition t6 {
                from CalcRightObject
                to ChooseLeftObject
                condition leftObject == 0 \/ rightObject == 0
                action done = true
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
            condition $ VisibleClustersCC ? data
            action counter = 1 ; SmallestVisibleCluster . clusterSize = 0 ; $ DisableClusterWatch ; moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd ; exec
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
            action counter = 1
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
        entry $ EnableClusterWatch ; $ EnableOA
    }
    initial i0
    state DE_BACKUP {
        entry # T
    }
    state EXILE {
        entry # T
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
            condition since ( T ) > pi / ( 2 * av )
            action moveCmd . x = 0 ; moveCmd . y = lv ; $ CCMove ! moveCmd
        }
        transition t2 {
            from EXILE_Turn
            to EXILE_Turn
            exec
            condition since ( T ) <= pi / ( 2 * av )
            action moveCmd . x = av ; moveCmd . y = 0 ; $ CCMove ! moveCmd
        }
        transition t3 {
            from EXILE_LinearMove
            to EXILE_LinearMove
            exec
            condition since ( T ) < timeout_EXILE
            action moveCmd . x = 0 ; moveCmd . y = lv ; $ CCMove ! moveCmd
        }
    }
    state DE_PUSH {
        entry # T ; moveCmd . x = 0 ; moveCmd . y = PUSH_LV * lv ; $ CCMove ! moveCmd
    }
    state PU_TARGET {
        entry # T
        state MoveToTarget {
            state TurnToTarget {
                entry $ DisableOA ; if angle > 0 then moveCmd . x = TARGET_AV * av ; moveCmd . y = 0 ; $ CCMove ! moveCmd end ; if angle < 0 then moveCmd . x = - TARGET_AV * av ; moveCmd . y = 0 ; $ CCMove ! moveCmd end
            }
            state LinearMoveToTarget {
                entry # T ; moveCmd . x = 0 ; moveCmd . y = lv ; $ CCMove ! moveCmd ; $ PickUpObject ( )
            }
            initial i0
            transition t0 {
                from TurnToTarget
                to TurnToTarget
                condition not $ TargetObject
                action exec
            }
            transition t1 {
                from TurnToTarget
                to LinearMoveToTarget
                exec
                condition abs ( angle - ANGLE_DIFF ) <= ANGLE_DIFF_TOLERANCE
                action $ EnableOA ; moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd
            }
            transition t2 {
                from LinearMoveToTarget
                to LinearMoveToTarget
                exec
                condition not $ ObjectCarried
    			action $ PickUpObject ( )
            }
            transition t3 {
                from TurnToTarget
                to TurnToTarget
                exec
                condition $ Coord_O ? coord /\ abs ( angle - ANGLE_DIFF ) > ANGLE_DIFF_TOLERANCE
                action targetPosition . x = targetObject . x + coord . x ; targetPosition . y = targetObject . y + coord . y ; angle = calculate_turn_angle ( coord , targetPosition )
            }
            transition t4 {
                from TurnToTarget
                to TurnToTarget
                condition not ( abs ( angle - ANGLE_DIFF ) <= ANGLE_DIFF_TOLERANCE /\ not ( abs ( angle - ANGLE_DIFF ) > ANGLE_DIFF_TOLERANCE ) )
                action $ CCMove ! moveCmd ; exec
            }
            transition t5 {
                from LinearMoveToTarget
                to TurnToTarget
                exec
                condition not $ InvalidTarget /\ since ( T ) >= timeout /\ $ TargetObject ? targetObject /\ $ Coord_O ? coord
                action targetPosition . x = targetObject . x + coord . x ; targetPosition . y = targetObject . y + coord . y
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
        state TurnToHome {
            entry if angle > 0 then moveCmd . x = TARGET_AV * av ; moveCmd . y = 0 ; $ CCMove ! moveCmd else moveCmd . x = - TARGET_AV * av ; moveCmd . y = 0 ; $ CCMove ! moveCmd end
        }
        initial i0
        state LinearMoveToHome {
            entry # T
        }
        state WaitForCoord {
        }
        transition t0 {
            from i0
            to WaitForCoord
        }
        transition t1 {
            from TurnToHome
            to LinearMoveToHome
            exec
            condition $ Coord_O ? coord /\ abs ( angle - ANGLE_DIFF ) <= ANGLE_DIFF_TOLERANCE
            action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd ; $ EnableOA
        }
        transition t2 {
            from TurnToHome
            to TurnToHome
            exec
            condition abs ( angle - ANGLE_DIFF ) > ANGLE_DIFF_TOLERANCE /\ $ Coord_O ? coord
            action angle = calculate_turn_angle ( coord , m [ ct ] . centroid )
        }
        transition t3 {
            from LinearMoveToHome
            to LinearMoveToHome
            exec
            condition since ( T ) < timeout
            action moveCmd . x = 0 ; moveCmd . y = LINEAR_HOME * lv ; $ CCMove ! moveCmd
        }
        transition t4 {
            from WaitForCoord
            to TurnToHome
            exec
            condition $ Coord_O ? coord
            action angle = calculate_turn_angle ( coord , targetPosition )
        }
        transition t5 {
            from WaitForCoord
            to WaitForCoord
            exec
            condition not $ Coord_O
        }
        transition t6 {
            from LinearMoveToHome
            to TurnToHome
            exec
            condition since ( T ) >= timeout
            action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd ; angle = calculate_turn_angle ( coord , m [ ct ] . centroid ) ; $ DisableOA
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
        action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd ; $ DisableTargetWatch ; exec
    }
    transition t9 {
        from HOMING
        to PU_SCAN
        exec
        condition not $ ObjectCarried
        action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd
    }
    transition t1 {
        from PU_SCAN
        to PU_TARGET
        condition done == true
        action $ CurrentTypeCA ! ct ; $ DisableClusterWatch ; moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd ; done = false ; angle = 10 ; $ EnableTargetWatch ! targetPosition ; exec
    }
    transition t15 {
        from DE_PUSH
        to DE_BACKUP
        exec
        condition since ( T ) >= timeout_PUSH
        action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd ; $ DepositObject ( )
    }
    transition t18 {
        from EXILE
        to PU_SCAN
        exec
        condition since ( T ) >= timeout_EXILE
        action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd
    }
    transition t19 {
        from PU_TARGET
        to PU_SCAN
        condition $ InvalidTarget \/ since ( T ) > timeout
        action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd ; $ EnableOA ; $ DisableTargetWatch ; exec
    }
    transition t2 {
        from DE_BACKUP
        to EXILE
        exec
        condition ( not $ ObjectCarried ) /\ since ( T ) >= timeout_BACKUP
        action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd
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
        action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd ; $ DisableTargetWatch ; exec
    }
    transition t7 {
        from HOMING
        to DE_PUSH
        exec
        condition distance ( m [ ct ] . centroid , coord ) < LINEAR_HOME
        action moveCmd . x = 0 ; moveCmd . y = 0 ; $ CCMove ! moveCmd
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
        action moveCmd . x = 0 ; moveCmd . y = - lv ; $ CCMove ! moveCmd
    }
}

stm ObstacleAvoidance {
    const pi : real = 3
    const DISTANCE : real // 0.4
    const min_range : real // 0.1
    const max_range : real // 0.4
    const v_closest_angle : real 
    const v_closest_distance : real
    const v_av : real // 0.7
    const v_lv : real // 0.07
    var closest_angle : real = v_closest_angle
    var closest_distance : real = v_closest_distance
    var av : real = v_av
    var lv : real = v_lv
    var NOA_Move : Vec2
    var current_speed : real
    var oaCmd : Vec2
    clock T
    input context { uses ObstacleEvents uses IOA uses NOAMove }
    output context { uses OAMove }
    cycleDef cycle == 1
    state OAEnabled {
        entry oaCmd . x = av ; oaCmd . y = lv ; $ OAMove ! oaCmd
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
        condition $ DisableOA
    }
    transition t8 {
        from OADisabled
        to OAEnabled
        exec
        condition $ EnableOA
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
        condition ( closest_distance >= min_range ) /\ ( closest_distance < max_range ) /\ ( abs ( closest_angle ) <= 90 )
        action current_speed = NOA_Move.y
    }

    transition t3 {
        from j0
        to OAEnabled
        condition ( closest_angle > 0 ) /\ ( abs ( closest_angle ) >= 30 )
        action av = ( closest_angle - 100 ) * pi / 180
    }
    transition t4 {
        from j0
        to OAEnabled
        condition ( closest_angle <= 0 ) /\ ( abs ( closest_angle ) >= 30 )
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
        condition not ( ( closest_distance >= min_range ) /\ ( closest_distance < max_range ) /\ ( abs ( closest_angle ) <= 90 ) )
        action lv = NOA_Move.y; av = NOA_Move.x
    }
    transition t9 {
        from j0
        to j2
        condition ( abs ( closest_angle ) < 30 )
    }
    transition t10 {
        from j2
        to OAEnabled
        condition ( closest_distance < DISTANCE )
        action lv = - DISTANCE
    }
    transition t11 {
        from j2
        to OAEnabled
        condition ( closest_distance >= DISTANCE )
        action lv = 0
    }
    transition t12 {
        from j3
        to j0
        condition ( closest_distance > DISTANCE )
        action lv = current_speed / 2
    }
    transition t13 {
        from j3
        to j0
        condition ( closest_distance <= DISTANCE )
        action lv = 0
    }
}

stm MoveManager {
    var NOAcmd : Vec2
    var cmd : Vec2
    input context { uses CCMove uses OAMove uses RWMove }
    output context { uses NOAMove requires Move }
    cycleDef cycle == 1
    initial i0
    state MoveHandler {
        entry $ move ( cmd ) ; $ NOAMove ! NOAcmd
    }

    transition t0 {
        from MoveHandler
        to MoveHandler
        exec
        condition $ RWMove ? cmd /\ not $ OAMove /\ not $ CCMove
        action NOAcmd = cmd
    }
    transition t1 {
        from i0
        to MoveHandler
        action cmd.x = 0 ; cmd.y = 0 ; NOAcmd.x = 0 ; NOAcmd . y = 0
    }
    transition t2 {
        from MoveHandler
        to MoveHandler
        exec
        condition $ CCMove ? cmd /\ not $ OAMove
        action NOAcmd = cmd
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
        condition $ OAMove ? cmd /\ $ CCMove ? NOAcmd
    }
    transition t5 {
        from MoveHandler
        to MoveHandler
        exec
        condition $ OAMove ? cmd /\ not $ CCMove /\ not $ RWMove
    }
    transition t6 {
        from MoveHandler
        to MoveHandler
        exec
        condition not $ RWMove /\ not $ CCMove /\ not $ OAMove
    }
}

stm CachePointAssignment {
    var j : nat
    var counter : nat
    var L : ClusterData
    var data : nat * Seq( ClusterData ) * Seq( ObjectData )
    var m : Seq( ClusterData )
    input context { uses IVisibleClustersCA uses ICurrentTypeCA }
    output context { uses ICachePointsCC uses ICachePointsTW }
    cycleDef cycle == 1
    initial i0
    state CalculateL {
        entry if ( data [ 2 ] ) [ counter ] . clusterSize > L . clusterSize then L = ( data [ 2 ] ) [ counter ] end
    }
    state DecideNewCachePoint {
    }
    state UpdateCache {
        entry m [ L . clusterType ] = L
    }
    state UpdateCurrentCache {
        entry if ( data [ 2 ] ) [ counter ] . clusterType == j then ( if m [ j ] . clusterSize < ( data [ 2 ] ) [ counter ] . clusterSize then m [ j ] = ( data [ 2 ] ) [ counter ] end ) end
    }
    state DetectCluster {
    }
    transition t0 {
        from DetectCluster
        to CalculateL
        condition $ VisibleClustersCA ? data
        action exec ; counter = 0
    }
    transition t1 {
        from CalculateL
        to CalculateL
        condition counter <= data [ 1 ]
        action counter = counter + 1
    }
    transition t2 {
        from CalculateL
        to DecideNewCachePoint
        condition counter > data [ 1 ]
    }
    transition t3 {
        from DecideNewCachePoint
        to UpdateCache
        condition $ CurrentTypeCA ? j /\ L . clusterSize > m [ L . clusterType ] . clusterSize
    }
    transition t4 {
        from DecideNewCachePoint
        to UpdateCurrentCache
        condition L . clusterSize <= m [ L . clusterType ] . clusterSize
        action counter = 1
    }
    transition t5 {
        from UpdateCache
        to UpdateCurrentCache
        condition $ CurrentTypeCA ? j
        action counter = 1
    }
    transition t6 {
        from UpdateCurrentCache
        to UpdateCurrentCache
        condition counter <= data [ 1 ]
        action counter = counter + 1
    }
    transition t7 {
        from UpdateCurrentCache
        to DetectCluster
        exec
        condition counter > data [ 1 ]
        action $ CachePointsCC ! m ; $ CachePointsTW ! m
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
    const ObjectThreshold : real
    var targetObjectxy : Vec2
    var closestTargetObject : ObjectData
    var closestTargetObjectPosition : Vec2
    var counter : nat
    var data : nat * Seq( ClusterData ) * Seq( ObjectData )
    var done : boolean
    var validObject : boolean
    var targetType : nat
    var m : Seq( ClusterData )
    input context { uses ITargetWatchFromCC uses IVisibleClustersTW uses ICurrentTypeTW uses ICachePointsTW }
    output context { uses ITargetWatchToCC }
    cycleDef cycle == 1
    initial i0
    state DetectingTarget {
        state ValidateTargetObject {
            entry if ( closestTargetObject . clusterID == m [ targetType ] . clusterID ) \/ ( distance ( closestTargetObject . position , targetObjectxy ) < ObjectThreshold ) then validObject = false end
        }
        state PublishTargetObject {
            entry closestTargetObjectPosition = closestTargetObject . position
        }
        state CalculateTargetObject {
            entry if distance ( ( data [ 3 ] ) [ counter ] . position , targetObjectxy ) < distance ( closestTargetObject . position , targetObjectxy ) then closestTargetObject = ( data [ 3 ] ) [ counter ] end
        }
        initial i0
        state WaitForDisableTargetWatch {
        }
        transition t0 {
            from CalculateTargetObject
            to ValidateTargetObject
            condition ( counter == 1 ) /\ $ CachePointsTW ? m
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
            action counter = 0 ; $ TargetObject ! closestTargetObjectPosition
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
            condition counter < data [ 1 ]
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
        condition $ EnableTargetWatch ? targetObjectxy /\ $ VisibleClustersTW ? data /\ $ CurrentTypeTW ? targetType
    	action counter = 1 ; validObject = true
    }
    transition t2 {
        from DetectingTarget
        to WaitingForTarget
        exec
        condition $ DisableTargetWatch
    }
    transition t3 {
        from WaitingForTarget
        to WaitingForTarget
        exec
        condition not $ EnableTargetWatch
    }
}

stm RandomWalk {
    const lv : real // 0.07
    const av : real // 0.6
    const pi : real = 3 // 3.14159
    const v_randcoef : real
    var randcoef : real = v_randcoef
    var sign : nat
    var rwCmd : Vec2
    clock T
    input context { uses IClusterWatch }
    output context { uses RWMove }
    cycleDef cycle == 1
    initial i0
    state Wander {
        initial i0
        state Turn {
            entry # T ; randcoef = randomcoef ( ) ; rwCmd . x = av * sign ; rwCmd . y = 0 ; $ RWMove ! rwCmd
        }
        state Move_Forward {
            entry # T ; rwCmd . x = 0 ; rwCmd . y = lv ; $ RWMove ! rwCmd
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
            action rwCmd . x = 0 ; rwCmd . y = 0 ; $ RWMove ! rwCmd ; randcoef = randomcoef ( ) ; sign = random_sign ( )
        }
        transition t2 {
            from Move_Forward
            to Turn
            exec
            condition since ( T ) >= randcoef
            action rwCmd . x = 0 ; rwCmd . y = 0 ; $ RWMove ! rwCmd ; randcoef = randomcoef ( )
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
        action sign = 1; exec
    }
    transition t3 {
        from Wander
        to Wait
        exec
        condition $ DisableClusterWatch
        action rwCmd . x = 0 ; rwCmd . y = 0 ; $ RWMove ! rwCmd
    }
}

function randomnat ( ) : nat { }

function randomcoef ( ) : real { }

function sqrt ( v : real ) : real {
    precondition v >= 0
    postcondition result * result == v
}

function distance ( x1 : Vec2 , x2 : Vec2 ) : real {
    postcondition result == sqrt ( ( x2 . x - x1 . x ) * ( x2 . x - x1 . x ) + ( x2 . y - x1 . y ) * ( x2 . y - x1 . y ) )
}

function L2 ( x : Vec2 ) : real {
    postcondition result == sqrt ( ( x . x * x . x + x . y * x . y ) )
}

function dot ( x1 : Vec2 , x2 : Vec2 ) : real {
    postcondition result == x1 . x * x2 . x + x1 . y * x2 . y
}

function unit ( x : Vec2 ) : Vec2 {
    postcondition result . x == x . x / L2 ( x )
    postcondition result . y == x . y / L2 ( x )
}

function angle_between ( x1 : Vec2 , x2 : Vec2 ) : real {
    postcondition result == acos ( dot ( unit ( x1 ) , unit ( x2 ) ) )
}

function calculate_turn_angle ( x1 : Vec2 , x2 : Vec2 ) : real {
    postcondition result == 0
}
function acos ( x : real ) : real { }
function abs ( x : real ) : real {
    postcondition result == sqrt ( x * x )
}
function randcoef ( ) : real {
    postcondition 0 <= result <= 1
}
function randnat ( ) : real {
    postcondition 0 <= result <= 6
} function random_sign ( ) : nat {
    postcondition result == 1
}

module CacheConsM {
    connection TurtleBot on ObjectCarried to ctrl_ref0 on ObjectCarried ( _async )
    connection TurtleBot on VisibleClustersCC to ctrl_ref0 on VisibleClustersCC ( _async )
    connection TurtleBot on Coord_O to ctrl_ref0 on Coord_O ( _async )
    connection TurtleBot on VisibleClustersTW to ctrl_ref0 on VisibleClustersTW ( _async )
    connection TurtleBot on VisibleClustersCA to ctrl_ref0 on VisibleClustersCA ( _async )
    connection TurtleBot on closestDistance to ctrl_ref0 on closestDistance ( _async )
    connection TurtleBot on closestAngle to ctrl_ref0 on closestAngle ( _async )
    robotic platform TurtleBot {
        uses ObstacleEvents uses IStatus uses IVisibleClustersCC uses IVisibleClustersCA uses IVisibleClustersTW uses ICoord provides IObjectOps provides Move }

    cref ctrl_ref0 = CacheConsC
    cycleDef cycle == 1
}

