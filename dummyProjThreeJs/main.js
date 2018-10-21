"use strict";

var droneNum = 4;
var stepNum = 1000;
var initialDistFromCenter= 2;
var finalDistFromCenter= 10;
var arcLengthRad = Math.PI/3;
var initialPosition = [ 250.0, 250.0, 0.0];


var planner = new TrajectoryPlanner(
  initialPosition,
  droneNum,
  stepNum,
  initialDistFromCenter,
  finalDistFromCenter,
  arcLengthRad
);

var result;
var t = 0.0;
var d_t = 0.01;

var node1_x= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n1 x'
};

var node1_y= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n1 y'
};

var node1_z= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n1 z'
};



var node2_x= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n2 x'
};

var node2_y= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n2 y'
};

var node2_z= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n2 z'
};


var node3_x= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n3 x'
};

var node3_y= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n3 y'
};

var node3_z= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n3 z'
};


var node4_x= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n4 x'
};

var node4_y= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n4 y'
};

var node4_z= {
  x: [],
  y: [],
  mode: 'lines',
  name: 'n4 z'
};

var svg =  document.getElementById('svg_anim');
var drone1_draw = svg.getElementById('drone1');
var drone2_draw = svg.getElementById('drone2');
var drone3_draw = svg.getElementById('drone3');
var drone4_draw = svg.getElementById('drone4');


for ( var i = 0 ; i <= stepNum*6; i++ ){

  result = planner.stepConstellation();

  drone1_draw["cx"] = result[0];
  drone1_draw["cy"] = result[1];
  drone2_draw["cx"] = result[3];
  drone2_draw["cy"] = result[4];
  drone3_draw["cx"] = result[6];
  drone3_draw["cy"] = result[7];
  drone4_draw["cx"] = result[9];
  drone4_draw["cy"] = result[10];



  node1_x["x"].push(t);
  node1_x["y"].push(result[0]);

  node1_y["x"].push(t);
  node1_y["y"].push(result[1]);

  node1_z["x"].push(t);
  node1_z["y"].push(result[2]);

  node2_x["x"].push(t);
  node2_x["y"].push(result[3]);

  node2_y["x"].push(t);
  node2_y["y"].push(result[4]);

  node2_z["x"].push(t);
  node2_z["y"].push(result[5]);

  node3_x["x"].push(t);
  node3_x["y"].push(result[6]);

  node3_y["x"].push(t);
  node3_y["y"].push(result[7]);

  node3_z["x"].push(t);
  node3_z["y"].push(result[8]);

  node4_x["x"].push(t);
  node4_x["y"].push(result[9]);

  node4_y["x"].push(t);
  node4_y["y"].push(result[10]);

  node4_z["x"].push(t);
  node4_z["y"].push(result[11]);

  t += d_t;
}


var dataNode1 = [ node1_x, node1_y, node1_z];
var dataNode2 = [ node2_x, node2_y, node2_z];
var dataNode3 = [ node3_x, node3_y, node3_z];
var dataNode4 = [ node4_x, node4_y, node4_z];

var layout = {};

Plotly.newPlot('plot1', dataNode1, layout);
Plotly.newPlot('plot2', dataNode2, layout);
Plotly.newPlot('plot3', dataNode3, layout);
Plotly.newPlot('plot4', dataNode4, layout);


