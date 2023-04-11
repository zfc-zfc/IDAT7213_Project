clear all
close all

pose= load('/home/fangcheng/Workspace/fastlio2_ws/src/FAST-LIO2/Log/quad0_pose.txt');

offset = [0;0;1.5];

pose_esti_x = pose(:,1);
pose_esti_y = pose(:,2);
pose_esti_z = pose(:,3);

pose_real_x = pose(:,4) - offset(1);
pose_real_y = pose(:,5) - offset(2);
pose_real_z = pose(:,6) - offset(3);

N = length(pose_esti_x);

sum = 0;
for i=1:N
    err = [pose_esti_x(i) - pose_real_x(i); pose_esti_y(i) - pose_real_y(i);pose_esti_z(i) - pose_real_z(i)];
    sum = sum + norm(err).^2;
end

RMSE = sqrt(sum/N)

