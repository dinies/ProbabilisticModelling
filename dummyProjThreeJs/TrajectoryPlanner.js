export class TrajectoryPlanner{
  constructor(){
  this.curr_step = 0;
  var initialDistFromCenter = 0.2;
  var finalDistFromCenter = 2.2;
  var theta_angles = [0.0 ,  Math.PI/2 , Math.PI, Math.PI*3/2 ];
  this.drone_1 =  this.buildThreePhasesTrajectory( theta_angles[0] ,initialDistFromCenter, finalDistFromCenter );
  this.drone_2 =  this.buildThreePhasesTrajectory( theta_angles[1] ,initialDistFromCenter, finalDistFromCenter );
  this.drone_3 =  this.buildThreePhasesTrajectory( theta_angles[2] ,initialDistFromCenter, finalDistFromCenter );
  this.drone_4 =  this.buildThreePhasesTrajectory( theta_angles[3] ,initialDistFromCenter, finalDistFromCenter );

  }
  buildThreePhasesTrajectory(theta,initialDistFromCenter, finalDistFromCenter ){

    var delta_t_des= 0.1;
    var v_0 = 0.0;
    var v_f = 0.0;
    var t_initial = 0;
    var t_final = 10;

    var c= Math.cos(theta);
    var s= Math.sin(theta);

    var R= math.matrix( [
      [  c, s , 0.0 ],
      [ -s, c , 0.0 ],
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
      p_0_mat.subset( math.index( 0 ,0)),
      p_0_mat.subset( math.index( 1, 0)),
      p_0_mat.subset( math.index( 2, 0))
    );

    var drone_positioning_x = new CubicTrajectory( p_0[0], v_0, p_1[0], v_f, t_initial, t_final, delta_t_des);
    var drone_positioning_y = new CubicTrajectory( p_0[1], v_0, p_1[1], v_f, t_initial, t_final, delta_t_des);
    var drone_positioning_z = new CubicTrajectory( p_0[2], v_0, p_1[2], v_f, t_initial, t_final, delta_t_des);

    var drone_hovering_x= new CubicTrajectory( p_1[0], v_0, p_1[0], v_f, t_initial, t_final, delta_t_des);
    var drone_hovering_y= new CubicTrajectory( p_1[1], v_0, p_1[1], v_f, t_initial, t_final, delta_t_des);
    var drone_hovering_z= new CubicTrajectory( p_1[0], v_0, p_1[2], v_f, t_initial, t_final, delta_t_des);
   
    var drone_returning_x= new CubicTrajectory( p_1[0], v_0, p_0[0], v_f, t_initial, t_final, delta_t_des);
    var drone_returning_y= new CubicTrajectory( p_1[1], v_0, p_0[1], v_f, t_initial, t_final, delta_t_des);
    var drone_returning_z= new CubicTrajectory( p_1[2], v_0, p_0[2], v_f, t_initial, t_final, delta_t_des);

    var positions = [ 
      (drone_positioning_x.getPositions()).concat( drone_positioning_y.getPositions() , drone_positioning_z.getPositions()),
      (drone_hovering_x.getPositions()).concat( drone_hovering_y.getPositions(), drone_hovering_z.getPositions()),
      (drone_returning_x.getPositions()).concat( drone_returning_y.getPositions(), drone_returning_z.getPositions())
    ];

    return positions;

  }
  

  stepConstellation(){

    if ( curr_step <= this.drone_1[0].length ){
      var drones_positions = [
        this.drone_1[0][curr_step],
        this.drone_1[1][curr_step],
        this.drone_1[2][curr_step],
        this.drone_2[0][curr_step],
        this.drone_2[1][curr_step],
        this.drone_2[2][curr_step],
        this.drone_3[0][curr_step],
        this.drone_3[1][curr_step],
        this.drone_3[2][curr_step],
        this.drone_4[0][curr_step],
        this.drone_4[1][curr_step],
        this.drone_4[2][curr_step]
      ];

      this.curr_step += 1;
      return drones_positions;
    }
    else{
      return [ ];
    }
  }
}

