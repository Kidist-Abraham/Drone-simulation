function [ desired_state ] = traj_generator(t, state, waypoints)




   desired_state.pos = zeros(3,1);
   desired_state.vel = zeros(3,1);
   desired_state.acc = zeros(3,1);
   desired_state.yaw = 0;
   desired_state.yawdot = 0;
persistent coffx coffy coffz  traj_time d0 
if nargin > 2
   waypoints=waypoints';
   
   coffx=getCoff(waypoints(:,1),4);
   coffy=getCoff(waypoints(:,2),4);
   coffz=getCoff(waypoints(:,3),4);
   
  
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    
    %d0 =  2*(sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2+d(4,:).^2+d(4,:).^2));
    d0 = [1; 1 ;1 ;1];
   % traj_time = [0, cumsum(d0)];
    traj_time = [0,1,2,3,4];
    
    
else
    if(t > traj_time(end))
        t = traj_time(end)-0.0001;
       
    end
    
   
    t_index = find(traj_time >= t,1)-1;
   
    t_index=max(t_index,1);
  
    scale=(t-traj_time(t_index))/d0(t_index);

    poss=[1;scale;scale^2;scale^3;scale^4;scale^5;scale^6;scale^7];
    vell=[0;1;2*scale;3*(scale^2);4*(scale^3);5*(scale^4);6*(scale^5);7*(scale^6)];
    accc=[0;0;2;6*scale;12*(scale^2);20*(scale^3);30*(scale^4);42*(scale^5)];
    index = (t_index-1)*8+1:t_index*8;
    t0=polyT(8,0,scale)';
    t1=polyT(8,1,scale)';
    t2=polyT(8,2,scale)';
   % if(t == 0)
    %    desired_state.pos = waypoints0(:,1);
    %else
    % disp(traj_time);
      % desired_state.pos=[coffx(index)'*poss;coffy(index)'*poss;coffz(index)'*poss];
       % desired_state.vel=[coffx(index)'*vell;coffy(index)'*vell;coffz(index)'*vell];
        % desired_state.acc=[coffx(index)'*accc;coffy(index)'*accc;coffz(index)'*accc];
    %end
       % desired_state.pos = (1 - scale) * waypoints0(:,t_index) + scale * waypoints0(:,t_index);
       
       %if( t_index==1)
    desired_state.pos=[coffx(index)'*t0;coffy(index)'*t0;coffz(index)'*t0];
  
     %  else
       
   %  desired_state.pos=  [coffx(index)'*t0;coffy(index)'*t0;coffz(index)'*t0]; 
     
    % desired_state.pos=[coffx((t_index-2)*8+1:(t_index-1)*8)'*polyT(8,0,1)';coffy((t_index-2)*8+1:(t_index-1)*8)'*polyT(8,0,1)';coffz((t_index-2)*8+1:(t_index-1)*8)'*polyT(8,0,1)']+b;
      

    
       
  desired_state.vel=[coffx(index)'*t1;coffy(index)'*t1;coffz(index)'*t1];
  desired_state.acc=[coffx(index)'*t2;coffy(index)'*t2;coffz(index)'*t2];
end

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

