export class CubicTrajectory{
  constructor(  q_0, v_0, q_f, v_f, t_initial, t_final, delta_t_des){

    var t_f = t_final - t_initial;
    var ideal_step_num = t_f/delta_t_des;
    var rounded_step_num = Math.round(ideal_step_num);
    this.delta_t = t_f/rounded_step_num;
    var t_0 = 0;
    this.totTime= t_f;

    var known = [
      [ q_0 ],
      [ v_0 ],
      [ q_f ],
      [ v_f ]
    ];

    var A= math.matrix( [
      [  t_0^3  , t_0^2 , t_0 , 1 ],
      [ 3*t_0^2 , 2*t_0 , 1   , 0 ],
      [ t_f^3   , t_f^2 , t_f , 1 ],
      [ 3*t_f^2 , 2*t_f , 1   , 0 ]
    ]);

    var paramsMat = math.multiply( math.inv(A),  known);
    this.params= vec4(
      paramsMat.subset( math.index( 0 ,0)),
      paramsMat.subset( math.index( 1, 0)),
      paramsMat.subset( math.index( 2, 0))
      paramsMat.subset( math.index( 3, 0))
    );
  }

  getPositions(){

    var a = this.params[0];
    var b = this.params[1];
    var c = this.params[2];
    var d = this.params[3];

    var numOfSteps= this.totTime/ this.delta_t;
    var t= 0.0;
    var pos= 0.0;
    var positions= [ ];

    for (var i = 0; i < numOfSteps-1 ; i++) {
      pos = a*t^3 + b*t^2 + c*t + d;
      positions.push( pos);
      t += this.delta_t;
    }
    return positions;
  }
}
