function CubicTrajectory(q_0,v_0,q_f,v_f, stepNum ){
  this.stepNum = stepNum;
  var t_0 = 0;
  var t_f = 1;
  var known = [
    [ q_0 ],
    [ v_0 ],
    [ q_f ],
    [ v_f ]
  ];

  var A= math.matrix( [
    [ Math.pow(t_0,3),Math.pow(t_0,2) , t_0 , 1 ],
    [ 3*Math.pow(t_0,2) , 2*t_0 , 1   , 0       ],
    [ Math.pow(t_f,3),Math.pow(t_f,2) , t_f , 1 ],
    [ 3*Math.pow(t_f,2) , 2*t_f , 1   ,       0 ]
  ]);

  var paramsMat = math.multiply( math.inv(A),  known);
  this.params= vec4(
    paramsMat.subset( math.index( 0 ,0)),
    paramsMat.subset( math.index( 1, 0)),
    paramsMat.subset( math.index( 2, 0)),
    paramsMat.subset( math.index( 3, 0))
  );

  this.getPositions = function(){

    var a = this.params[0];
    var b = this.params[1];
    var c = this.params[2];
    var d = this.params[3];

    var t= 0.0;
    var pos= 0.0;
    var positions= [];
    var delta_t = 1/this.stepNum;

    for (var i = 0; i < this.stepNum ; i++) {
      pos = a*Math.pow(t,3) + b*Math.pow(t,2) + c*t + d;
      positions.push( pos);
      t += delta_t;
    }
    return positions;
  }
};


function TrajectoryPlanner( 
  initialPosition,
  droneNum,
  stepNum,
  initialDistFromCenter,
  finalDistFromCenter,
  arcLengthRad
){

  this.initialPosition;
  this.droneNum = droneNum;
  this.curr_step = 0;
  var delta_theta = (Math.PI*2)/droneNum;
  var theta_angles =[];
  var curr_theta= 0.0;
  for( var n =0; n<droneNum; n++){
    theta_angles.push( curr_theta);
    curr_theta += delta_theta;
  }

  this.buildThreePhasesTrajectory =function(
    theta,
    initialDistFromCenter,
    finalDistFromCenter,
    stepNum ){

    var v_0 = 0.0;
    var v_f = 0.0;
    var t_initial = 0;
    var t_final = 1;

    var c= Math.cos(theta);
    var s= Math.sin(theta);

    var R= math.matrix( [
      [ c,-s , 0.0 ],
      [ s, c , 0.0 ],
      [ 0.0,0.0,1.0 ]
    ]);

    var p_0_frame0 = [
      [ initialDistFromCenter+ this.initialPosition[0]],
      [ this.initialPosition[1]],
      [ this.initialPosition[2]]
    ];

    var p_1_frame0 = [
      [ finalDistFromCenter+ this.initialPosition[0]],
      [ this.initialPosition[1]],
      [ this.initialPosition[2]]
    ];
    var p_0_mat = math.multiply( R,  p_0_frame0);
    var p_0 = vec3(
      p_0_mat.subset( math.index( 0 ,0)),
      p_0_mat.subset( math.index( 1, 0)),
      p_0_mat.subset( math.index( 2, 0))
    );
    var p_1_mat = math.multiply( R,  p_1_frame0);
    var p_1 = vec3(
      p_1_mat.subset( math.index( 0 ,0)),
      p_1_mat.subset( math.index( 1, 0)),
      p_1_mat.subset( math.index( 2, 0))
    );


    var drone_positioning_x=
      new CubicTrajectory(p_0[0],v_0,p_1[0],v_f,stepNum);
    var drone_positioning_y=
      new CubicTrajectory(p_0[1],v_0,p_1[1],v_f,stepNum);
    var drone_positioning_z=
      new CubicTrajectory(p_0[2],v_0,p_1[2],v_f,stepNum);

    var drone_hovering_x=
      new CubicTrajectory(p_1[0],v_0,p_1[0],v_f,stepNum);
    var drone_hovering_y=
      new CubicTrajectory(p_1[1],v_0,p_1[1],v_f,stepNum);
    var drone_hovering_z=
      new CubicTrajectory(p_1[2],v_0,p_1[2],v_f,stepNum);

    var drone_returning_x=
      new CubicTrajectory(p_1[0],v_0,p_0[0],v_f,stepNum);
    var drone_returning_y=
      new CubicTrajectory(p_1[1],v_0,p_0[1],v_f,stepNum);
    var drone_returning_z=
      new CubicTrajectory(p_1[2],v_0,p_0[2],v_f,stepNum);

    var positions = [ 
      (drone_positioning_x.getPositions()).concat(
        drone_hovering_x.getPositions(),
        drone_returning_x.getPositions()),
      (drone_positioning_y.getPositions()).concat(
        drone_hovering_y.getPositions(),
        drone_returning_y.getPositions()),
      (drone_positioning_z.getPositions()).concat(
        drone_hovering_z.getPositions(),
        drone_returning_z.getPositions())
    ];
    return positions;
  }

  this.buildSixPhasesTrajectory =function(
    theta,
    initialDistFromCenter,
    finalDistFromCenter,
    stepNum,
    arcLengthRad
  ){

    var v_0 = 0.0;
    var v_f = 0.0;
    var t_initial = 0;
    var t_final = 1;

    var c= Math.cos(theta);
    var s= Math.sin(theta);

    var R= math.matrix( [
      [ c,-s , 0.0 ],
      [ s, c , 0.0 ],
      [ 0.0,0.0,1.0 ]
    ]);

    var p_0_frame0 = [
      [ initialDistFromCenter],
      [ 0.0],
      [ 0.0]
    ];

    var p_1_frame0 = [
      [ finalDistFromCenter],
      [ 0.0],
      [ 0.0]
    ];
    var p_0_mat = math.multiply( R,  p_0_frame0);
    var p_0 = vec3(
      p_0_mat.subset( math.index( 0 ,0)),
      p_0_mat.subset( math.index( 1, 0)),
      p_0_mat.subset( math.index( 2, 0))
    );
    var p_1_mat = math.multiply( R,  p_1_frame0);
    var p_1 = vec3(
      p_1_mat.subset( math.index( 0 ,0)),
      p_1_mat.subset( math.index( 1, 0)),
      p_1_mat.subset( math.index( 2, 0))
    );

    var drone_positioning_x=
      new CubicTrajectory(p_0[0],v_0,p_1[0],v_f,stepNum);
    var drone_positioning_y=
      new CubicTrajectory(p_0[1],v_0,p_1[1],v_f,stepNum);
    var drone_positioning_z=
      new CubicTrajectory(p_0[2],v_0,p_1[2],v_f,stepNum);

    var drone_firstHovering_x=
      new CubicTrajectory(p_1[0],v_0,p_1[0],v_f,stepNum);
    var drone_firstHovering_y=
      new CubicTrajectory(p_1[1],v_0,p_1[1],v_f,stepNum);
    var drone_firstHovering_z=
      new CubicTrajectory(p_1[2],v_0,p_1[2],v_f,stepNum);

    var drone_arcMotion =
      new CubicTrajectory(0,v_0,1,v_f,stepNum);

    var arcMotionCordalPoints = drone_arcMotion.getPositions();
    var arcMotion_x= [];
    var arcMotion_y= [];
    var arcMotion_z= [];
    var p_2_mat;
    var p_2;
    var cordalPoint;
    var curr_beta;
    var R_beta;
    var R_tot
    for (var i= 0; i<arcMotionCordalPoints.length; i++ ){
      cordalPoint = arcMotionCordalPoints[i];
      curr_beta = arcLengthRad * cordalPoint;
      c= Math.cos(curr_beta);
      s= Math.sin(curr_beta); 
    
      R_beta = math.matrix( [
        [ c,-s , 0.0 ],
        [ s, c , 0.0 ],
        [ 0.0,0.0,1.0 ]
      ]);

      R_tot= math.multiply( R, R_beta);

      p_2_mat = math.multiply( R_tot,  p_1_frame0);
      p_2 = vec3(
        p_2_mat.subset( math.index( 0 ,0)),
        p_2_mat.subset( math.index( 1, 0)),
        p_2_mat.subset( math.index( 2, 0))
      );
      arcMotion_x.push( p_2[0]);
      arcMotion_y.push( p_2[1]);
      arcMotion_z.push( p_2[2]);
    }

    var drone_secondHovering_x=
      new CubicTrajectory(p_2[0],v_0,p_2[0],v_f,stepNum);
    var drone_secondHovering_y=
      new CubicTrajectory(p_2[1],v_0,p_2[1],v_f,stepNum);
    var drone_secondHovering_z=
      new CubicTrajectory(p_2[2],v_0,p_2[2],v_f,stepNum);

    var arcMotion_opposite_x= arcMotion_x.slice(0);
    var arcMotion_opposite_y= arcMotion_y.slice(0);
    var arcMotion_opposite_z= arcMotion_z.slice(0);  
    arcMotion_opposite_x.reverse();
    arcMotion_opposite_y.reverse();
    arcMotion_opposite_z.reverse();
    

    var drone_returning_x=
      new CubicTrajectory(p_1[0],v_0,p_0[0],v_f,stepNum);
    var drone_returning_y=
      new CubicTrajectory(p_1[1],v_0,p_0[1],v_f,stepNum);
    var drone_returning_z=
      new CubicTrajectory(p_1[2],v_0,p_0[2],v_f,stepNum);

    var positions = [];
    positions[0] = 
      drone_positioning_x.getPositions().concat(
        drone_firstHovering_x.getPositions(),
        arcMotion_x,
        drone_secondHovering_x.getPositions(),
        arcMotion_opposite_x,
        drone_returning_x.getPositions());

    positions[1] = 
      drone_positioning_y.getPositions().concat(
        drone_firstHovering_y.getPositions(),
        arcMotion_y,
        drone_secondHovering_y.getPositions(),
        arcMotion_opposite_y,
        drone_returning_y.getPositions());

    positions[2] = 
      drone_positioning_z.getPositions().concat(
        drone_firstHovering_z.getPositions(),
        arcMotion_z,
        drone_secondHovering_z.getPositions(),
        arcMotion_opposite_z,
        drone_returning_z.getPositions());

    return positions;

  }

  this.drones = [];
  for( var k = 0; k<this.droneNum; k++){
    this.drones[k] = this.buildSixPhasesTrajectory(
    theta_angles[k] ,initialDistFromCenter,
    finalDistFromCenter, stepNum, arcLengthRad );
  }
 
  this.stepConstellation = function(){

    if ( this.curr_step <= this.drones[0][0].length ){
      var drones_positions = [];
      for( var k = 0; k<this.droneNum; k++){
        for( var j=0; j<3;j++){
          drones_positions[k*3+j]=
            this.drones[k][j][this.curr_step];
        }
      }
      this.curr_step += 1;
      return drones_positions;
    }
    else{
      return [ ];
    }
  };
};










