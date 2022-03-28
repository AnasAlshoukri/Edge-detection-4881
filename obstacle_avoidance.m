%rosshutdown;clear;clc;
% Initiliaze the ROS
ipaddress = "http://192.168.209.129:11311";
rosinit(ipaddress)
% Create ROS subscribers and publishers

lidarSub = rossubscriber("/scan");
[velPub,velMsg] = rospublisher("/cmd_vel");
posSub = rossubscriber("/odom");

% plotobj = CanHelperTurtleBotVisualizer([-11,11,-11,11]);

 figure
 hold on;
 grid on;grid minor;
 axis([-11 11 -11 11])
%%
 tic;
  while toc < 15
      % Collect information from laser scan
      scan = receive(lidarSub);
      position = receive(posSub);
      q3=position.Pose.Pose.Orientation.Z;
      q4=position.Pose.Pose.Orientation.W;
      
      quat = position.Pose.Pose.Orientation;
      angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
      theta = rad2deg(angles(1));
      
      pose = [position.Pose.Pose.Position.X,position.Pose.Pose.Position.Y,quat2angle([q4 0 0 q3])];
      
      laserData = readCartesian(scan) * [0 1; -1 0];

      while(theta<0) 
            theta = theta + 360;
      end 
 % M is the closest distance, N is the closest angle
      [M,N] = min(scan.Ranges);
      a = theta + N -180;
      while(a>=360)
          a = a-360;
      end    
      while(a<0)
          a = a+360;
      end    
      P = N+180;
      while(P>360)
          P = P-360;
      end
      T = scan.Ranges(P);
      O = (M+T)/2;
      
      x1 = pose(1) + cosd(a)*(O-M);
      y1 = pose(2) + sind(a)*(O-M);
      
%% count the left and right wall values(needs work)(fails mostly)
    
      count1 = 0;
      count2 = 0;
      i = 0;
      c = -1;

      while(scan.Ranges(mod(N+i,360)) < Inf)
         count1 = count1 + 1;
         i = i + 1;
         if(N+i == 360)
             i = i +1;
             count1 = count1 +1;
         end
              
      end
       while(scan.Ranges(mod(N+c,360)) < Inf)
         count2 = count2 + 1;
         c = c - 1;
         if(N + c == 0)
             c = c -1;
             count2 = count2 +1;
         end
       end
       
       count3 = 0;
     for a1=1:1:360
         if(scan.Ranges(a1) < Inf)
             count3 = count3 + 1;
         end
     end
   % Right wall = Total Wall - ( Left1 Wall + Left2 Wall )
     count4 = count3 - (count1 + count2) ;
%%     
    th = pose(3)-pi/2;
    data = laserData;
    
    dataWorld = data*[cos(th) sin(th);-sin(th) cos(th)] ...
      + repmat(pose(1:2),[numel(data(:,1)),1]);
     
     
      v = count1 + count2;
      p1 = dataWorld(1:v,:);
      p2 = dataWorld(v+1:end,:);

%%      
%       p1_new = p1(1:60,:);
%       c3 = polyfit(p1_new(:,1),p1_new(:,2),1);
%       t_est = polyval(c3,p1_new(:,1));

        plan_w = length(p1);
        plan_w_1 = length(p2);
        
        if(plan_w > 100 && plan_w_1 > 100)
            plan = 1;
        elseif(plan_w > 70 && plan_w_1 > 70)
            plan = 2;
        elseif(plan_w > 50 && plan_w_1 > 50)
            plan = 3;
        elseif(plan_w > 30 && plan_w_1 > 30)
            plan = 4;
        end
        
      t1_new = p1(20:30,:);
      c4 = polyfit(t1_new(:,1),t1_new(:,2),1);
      x_est = polyval(c4,t1_new(:,1));
      
      t2_new = p2(20:30,:);
      c5 = polyfit(t2_new(:,1),t2_new(:,2),1);
      c_est = polyval(c5,t2_new(:,1));
%       
%       c1 = polyfit(p1(:,1),p1(:,2),1);
%       y_est = polyval(c1,p1(:,1));
%       c2 = polyfit(p2(:,1),p2(:,2),1);
%       z_est = polyval(c2,p2(:,1));

       o1 = (t1_new(5,1) + t2_new(5,1))/2;
       o2 = (t1_new(5,2) + t2_new(5,2))/2;
       
        
%       plot(p1_new(:,1),t_est,'.')
       plot(t1_new(:,1),x_est,'.','Color','k')  % Outer walls
       plot(t2_new(:,1),c_est,'.','Color','k')  % ""     ""
       plot(pose(1),pose(2),'*','Color','r')    % robot position
       plot(o1,o2,'.','Color','b')              % estimated position
%       plot(p2(:,1),z_est,'.')
    
%%      
%         plot_Data(plotobj,pose,laserData,x1,y1);

      ranges = double(scan.Ranges);
%       plot(scan)
       
      front_1 = min(min(ranges(1:10),3.5));
      front_2 = min(min(ranges(350:360),3.5));
      front_left = min(min(ranges(11:50),3.5));
      left = min(min(ranges(51:90),3.5));
      rear = min(min(ranges(91:269),3.5));
      right = min(min(ranges(270:309),3.5));
      front_right = min(min(ranges(310:349),3.5));
      
%     Basic Obstacle Avoidance

      if ((floor(front_1) && floor(front_2)) == 1)
%           disp('Going straight...')
          velMsg.Linear.X = 1;
          velMsg.Angular.Z = 0;
      end
        if((front_left - front_right) < 0)
%           disp('Turning right...')
          velMsg.Linear.X = 0.7;
          velMsg.Angular.Z = -0.15;
        elseif(front_right - front_left < 0)
%           disp('Turning left...')
          velMsg.Linear.X = 0.7;
          velMsg.Angular.Z = 0.15;
        end
      send(velPub,velMsg);
  end
%%  
  % Stop the robot
  
    disp('Stopped.')
    velMsg.Linear.X = 0;
    velMsg.Linear.Y = 0;
    velMsg.Linear.Z = 0;
    velMsg.Angular.X = 0;
    velMsg.Angular.Y = 0;
    velMsg.Angular.Z = 0;
    send(velPub,velMsg);
 % end 