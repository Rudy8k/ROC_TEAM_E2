a1 = 0.75;
a2 = 0.75;

I = imread('Untitled.png');
y=255-I;
BW = im2bw(y);
dim = size(BW)
col = round(dim(2)/2)-90;
row = min(find(BW(:,col), 1 ));
%BW_filled = imfill(BW,'holes');
boundaries = bwboundaries(BW);

px=[boundaries{1}(1,1)/1080];
py=[boundaries{1}(1,1)/1080];
[th_1,th_2,th_0,th_00]=iKin(a1,a2,px(1),py(1));
th1=[th_1,];
th2=[th_2,];
p1_x=[px(1)];
p1_y=[py(1)];
for i=1:length(boundaries)
    for j=1:length(boundaries{i})
        p2_x=boundaries{i}(j,1)/1080;
        p2_y=boundaries{i}(j,2)/1080;
        px=[px,p2_x];
        py=[py,p2_y];
        p1_x=p2_x;
        p1_y=p2_y;
    end
end

for i=1:length(px)-2
    if abs(2*px(i+1)-px(i+2)-px(i))>abs(px(i)-px(i+2))
        px(i+1)=px(i+2);
        py(i+1)=py(i+2);
    end
end

for i=1:length(px)
    [temp_th1,temp_th2,~,~]=iKin(a1,a2,px(i),py(i));
    th1=[th1,temp_th1];
    th2=[th2,temp_th2];
end
plot_da_points(a1,a2,px,py)
th1=th1*180/pi;
th1=th1';

th2=th2*180/pi;
th2=th2';
timelist=zeros(length(th1),1);
time=5;
for i=1:length(timelist)
    timelist(i)=(i-1)*5/length(timelist);
end

function plot_points(px, py)
    plot(px, py, 'b-o');
    axis equal;
    xlabel('X');
    ylabel('Y');
    title('Plot of Points');
end

function plot_da_points(a1, a2, px_list, py_list)
    figure;
    hold on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    title('Robotic Arm Configuration');
    
    % Initialize variables for tracking end effector trajectory
    trajectory_x = [];
    trajectory_y = [];

    for i = 1:length(px_list)
        % Retrieve end effector coordinates from px_list and py_list
        b_x = px_list(i);
        b_y = py_list(i);
        
        % Compute joint coordinates
        theta_2 = acos((b_x^2 + b_y^2 - a1^2 - a2^2) / (2 * a1 * a2));
        theta_1 = atan2(b_y, b_x) - atan2(a2 * sin(theta_2), a1 + a2 * cos(theta_2));
        a_x = a1 * cos(theta_1);
        a_y = a1 * sin(theta_1);

        % Plot arm configuration
        plot([0, a_x, b_x], [0, a_y, b_y], 'b-o');
        
        % Plot arm segments
        plot([0, a_x], [0, a_y], 'r--');
        plot([a_x, b_x], [a_y, b_y], 'g--');
        
        % Plot joint positions
        plot(a_x, a_y, 'ro', 'MarkerSize', 8);
        plot(b_x, b_y, 'go', 'MarkerSize', 8);
        
        % Update end effector trajectory   
        trajectory_x = [trajectory_x, b_x];
        trajectory_y = [trajectory_y, b_y];
        
        % Plot end effector trajectory
        plot(trajectory_x, trajectory_y, 'k-');
        
        % Plot axes
        plot([-a1 - a2, a1 + a2], [0, 0], 'k--');  % X-axis
        plot([0, 0], [-a1 - a2, a1 + a2], 'k--');  % Y-axis
        
        % Add a pause to observe the animation (adjust duration as needed)
        pause(0.0001);
        
        % Clear the current plot to update the animation
        if i ~= length(px_list)
            cla;
        end
    end
end
function [theta_1,theta_2,px,py]=develop_curve_points(a1,a2,X_list,Y_list)
    for i=1:length(X_list)
        
    end
end
function [theta_list_1,theta_list_2,px_list,py_list]= points(a1,a2,p1_x,p1_y,p2_x,p2_y,th_1,th_2,n)
    P1=[p1_x,p1_y];
    P2=[p2_x,p2_y];
    dP=(P2-P1)/n;
    theta_list_1=[];
    theta_list_2=[];
    px_list=[P1(1)];
    py_list=[P1(2)];
    for i=1:n+1
        P=P1+(i*dP);
        px=P(1);
        py=P(2);
        [theta_1_a, theta_2_a, theta_1_b, theta_2_b]=iKin(a1,a2,px,py);
        if ((theta_1_a-th_1)^2+(theta_2_a-th_2)^2)<((theta_1_b-th_1)^2+(theta_2_b-th_2)^2)
            theta_list_1(end+1)=theta_1_a;
            theta_list_2(end+1)=theta_2_a;
            
            th_1=theta_1_a;
            th_2=theta_2_a;
        else
            theta_list_1(end+1)=theta_1_b;
            theta_list_2(end+1)=theta_2_b;            
            th_1=theta_1_b;
            th_2=theta_2_b;
        end
        px_list(end+1)=px;
        py_list(end+1)=py; 
    end
end
function [theta_1_a, theta_2_a, theta_1_b, theta_2_b] = iKin(a1, a2, px, py)
    c2 = (px^2 + py^2 - a1^2 - a2^2) / (2 * a1 * a2);

    s2_1=sqrt(1-c2^2);
    s2_2 = -sqrt(1 - c2^2);
    theta_2_1=atan2(s2_1,c2);
    theta_2_2=atan2(s2_2,c2);
    theta_2_a=theta_2_1;
    theta_2_b=theta_2_2;
    s1_1=(py*(a1+a2*c2)-px*(a2*s2_1))/(a1^2+a2^2+2*a1*a2*c2);
    c1_1=(px*(a1+a2*c2)+py*(a2*s2_1))/(a1^2+a2^2+2*a1*a2*c2);
    theta_1_a=atan2(s1_1,c1_1);
    s1_2=(py*(a1+a2*c2)-px*(a2*s2_2))/(a1^2+a2^2+2*a1*a2*c2);
    c1_2=(px*(a1+a2*c2)+py*(a2*s2_2))/(a1^2+a2^2+2*a1*a2*c2);
    theta_1_b=atan2(s1_2,c1_2);
end