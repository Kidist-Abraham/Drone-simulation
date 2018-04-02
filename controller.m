function [F, M] = controller(t, state, des_state, params)



kpT3=96;
kdT3=7;

kpT1=60;
kdT1=7;

kpT2=50;
kdT2=7;

kpM1=100;
kdM1=7;

kpM2=90;
kdM2=7;

kpM3=90;
kdM3=7;

DesiredX=des_state.acc(1)+kpT1*(des_state.pos(1)-state.pos(1))+kdT1*(des_state.vel(1)-state.vel(1));
DesiredY=des_state.acc(2)+kpT2*(des_state.pos(2)-state.pos(2))+kdT2*(des_state.vel(2)-state.vel(2));
PitchDes=(1/params.gravity)*(DesiredX*sin(des_state.yaw)-DesiredY*cos(des_state.yaw));
RollDes=(1/params.gravity)*(DesiredX*cos(des_state.yaw)+DesiredY*sin(des_state.yaw));
PitchDesVel=(1/params.gravity)*(des_state.acc(1)*des_state.yawdot*cos(des_state.yaw)+des_state.acc(2)*des_state.yawdot*sin(des_state.yaw));
RollDesVel=(1/params.gravity)*(des_state.acc(2)*des_state.yawdot*cos(des_state.yaw)-des_state.acc(1)*des_state.yawdot*sin(des_state.yaw));

% Thrust
F = params.mass*params.gravity+params.mass*(des_state.acc(3)+kpT3*(des_state.pos(3)-state.pos(3))+kdT3*(des_state.vel(3)-state.vel(3)));

% Moment
M =[kpM1*(PitchDes-state.rot(1))+kdM1*(PitchDesVel-state.omega(1)); kpM2*(RollDes-state.rot(2))+kdM2*(RollDesVel-state.omega(2)); kpM3*(des_state.yaw-state.rot(3))+kdM3*(des_state.yawdot-state.omega(3))];


end
