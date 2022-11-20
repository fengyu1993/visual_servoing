%% 2D 实验
clc;
close all;
clear;
%%
resolution_x = 512;
resolution_y = 512;
%% 数据加载
load('DCT_VS_experience_data_2D.mat');
error_feature_DCT = DCT_VS_error_feature;
error_pixel_DCT = DCT_VS_error_pixel*resolution_x*resolution_y;
[error_pose_DCT, T_DCT] = get_pose_error_T(DCT_VS_camera_pose, DCT_VS_camera_desired_pose);
velocity_DCT = DCT_VS_camera_velocity;
order_list_DCT = DCT_VS_order;
load('HM_VS_experience_data_2D.mat');
error_feature_HM = HM_VS_error_feature;
error_pixel_HM = HM_VS_error_pixel*resolution_x*resolution_y;
[error_pose_HM, T_HM] = get_pose_error_T(HM_VS_camera_pose, HM_VS_camera_desired_pose);
velocity_HM = HM_VS_camera_velocity;
order_list_HM = HM_VS_order;
parameter_ab_xy_HM = HM_VS_abxy;
load('KM_VS_experience_data_2D.mat');
error_feature_KM = KM_VS_error_feature;
error_pixel_KM = KM_VS_error_pixel*resolution_x*resolution_y;
[error_pose_KM, T_KM] = get_pose_error_T(KM_VS_camera_pose, KM_VS_camera_desired_pose);
velocity_KM = KM_VS_camera_velocity;
order_list_KM = KM_VS_order;
p_list_KM = KM_VS_pxy;
load('TM_VS_experience_data_2D.mat');
error_feature_TM = TM_VS_error_feature;
error_pixel_TM = TM_VS_error_pixel*resolution_x*resolution_y;
[error_pose_TM, T_TM] = get_pose_error_T(TM_VS_camera_pose, TM_VS_camera_desired_pose);
velocity_TM = TM_VS_camera_velocity;
order_list_TM = TM_VS_order;
%% 全局信息
LineWidth = 1.8;
FontSize_1 = 18;
FontSize_2 = 15;
%% 图片显示
figure; imshow(1-image_gray_new_norm_init, [0,1]);
figure; imshow(1-image_gray_old_norm, [0,1]);
%% 像素误差
f_pixel = figure;
hold on; box on; grid on;
plot(0:length(error_pixel_TM)-1, error_pixel_TM, 'b-', 'LineWidth', LineWidth);
plot(0:length(error_pixel_KM)-1, error_pixel_KM, 'g-', 'LineWidth', LineWidth);
plot(0:length(error_pixel_HM)-1, error_pixel_HM, 'r-', 'LineWidth', LineWidth);
plot(0:length(error_pixel_DCT)-1, error_pixel_DCT, 'c-', 'LineWidth', LineWidth);
ylabel("$||\bf{I} - \bf{I}^*||^2$",'Interpreter','latex','FontSize',FontSize_2,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 140 0 12e4])
set(gca,'xtick',0:20:140,'FontSize',FontSize_2,'FontName','Times New Roman');
set(gca,'ytick',0:2e4:12e4,'FontSize',FontSize_2,'FontName','Times New Roman');
set(gca,'LineWidth', LineWidth);
legend('TM-VS', 'KM-VS', 'HM-VS', 'DCT-VS','FontName','Times New Roman', ...
    'FontSize',FontSize_2*0.8,'LineWidth', LineWidth,'Location', 'northeast');
%% 轨迹
f_trajection = figure; 
axis equal; hold on; box on; grid on;
x_DCT = T_DCT(1,4,:); y_DCT = T_DCT(2,4,:); z_DCT = T_DCT(3,4,:);
x_HM = T_HM(1,4,:); y_HM = T_HM(2,4,:); z_HM = T_HM(3,4,:);
x_KM = T_KM(1,4,:); y_KM = T_KM(2,4,:); z_KM = T_KM(3,4,:);
x_TM = T_TM(1,4,:); y_TM = T_TM(2,4,:); z_TM = T_TM(3,4,:);
plot3(x_TM(:), y_TM(:), z_TM(:), 'b-', 'LineWidth', LineWidth);
plot3(x_KM(:), y_KM(:), z_KM(:), 'g-', 'LineWidth', LineWidth);
plot3(x_HM(:), y_HM(:), z_HM(:), 'r-', 'LineWidth', LineWidth);
plot3(x_DCT(:), y_DCT(:), z_DCT(:), 'c-', 'LineWidth', LineWidth);
SL = [0.01;0.02;0.005]*6;
eul_start = rotm2eul(T_DCT(1:3,1:3,1),'ZYX');
off_start = T_DCT(1:3,1:3,1) * [0; 0; -SL(3)/2];
drawCuboid(SL,[T_DCT(1,4,1)+off_start(1);T_DCT(2,4,1)+off_start(2);T_DCT(3,4,1)+off_start(3)],eul_start,'m',0.5);
eul_end = rotm2eul(T_DCT(1:3,1:3,end),'ZYX');
off_end = T_DCT(1:3,1:3,end) * [0; 0; -SL(3)/2];
drawCuboid(SL,[T_DCT(1,4,end)+off_end(1);T_DCT(2,4,end)+off_end(2);T_DCT(3,4,end)+off_end(3)],eul_end,'m',0.5);
xlabel('x','FontSize',FontSize_2,'FontName','Times New Roman'); 
ylabel('y','FontSize',FontSize_2,'FontName','Times New Roman'); 
zlabel('z','FontSize',FontSize_2,'FontName','Times New Roman');
view(135, 15);
axis([-0.4 0.8 -0.8 0 0.6 1.6]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',-0.4:0.2:0.8,'FontSize',FontSize_2,'FontName','Times New Roman');
set(gca,'ytick',-0.8:0.2:0,'FontSize',FontSize_2,'FontName','Times New Roman');
set(gca,'LineWidth', LineWidth);
legend('TM-VS', 'KM-VS', 'HM-VS', 'DCT-VS','FontName','Times New Roman', ...
    'FontSize',FontSize_2*0.8,'LineWidth', LineWidth,'Location', 'northeast');
%% DCT 
% 速度 
f_velocity_DCT = figure;
hold on; box on; grid on;
plot(0:length(velocity_DCT)-1, velocity_DCT(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(velocity_DCT)-1, velocity_DCT(:,2), 'g-', 'LineWidth', LineWidth);
plot(0:length(velocity_DCT)-1, velocity_DCT(:,3), 'b-', 'LineWidth', LineWidth);
plot(0:length(velocity_DCT)-1, velocity_DCT(:,4), 'c-', 'LineWidth', LineWidth);
plot(0:length(velocity_DCT)-1, velocity_DCT(:,5), 'm-', 'LineWidth', LineWidth);
plot(0:length(velocity_DCT)-1, velocity_DCT(:,6), 'y-', 'LineWidth', LineWidth);
ylabel("Velocities $\nu$ and $\omega$",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 80 -1.5 2]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:10:80,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',-1.5:0.5:2,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'LineWidth', LineWidth);
legend('$\nu_x$', '$\nu_y$', '$\nu_z$', '$\omega_x$', '$\omega_y$', '$\omega_z$',...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'northeast');
% 位姿误差
f_pose_DCT = figure;
hold on; box on; grid on;
plot(0:length(error_pose_DCT)-1, error_pose_DCT(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(error_pose_DCT)-1, error_pose_DCT(:,2), 'g-', 'LineWidth', LineWidth);
plot(0:length(error_pose_DCT)-1, error_pose_DCT(:,3), 'b-', 'LineWidth', LineWidth);
plot(0:length(error_pose_DCT)-1, error_pose_DCT(:,4), 'c-', 'LineWidth', LineWidth);
plot(0:length(error_pose_DCT)-1, error_pose_DCT(:,5), 'm-', 'LineWidth', LineWidth);
plot(0:length(error_pose_DCT)-1, error_pose_DCT(:,6), 'y-', 'LineWidth', LineWidth);
ylabel("Errors on pose",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 80 -0.8 0.6]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:10:80,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',-0.8:0.2:0.6,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'LineWidth', LineWidth);
legend('$e_{t_x}$', '$e_{t_y}$', '$e_{t_z}$', '$e_{\theta\omega_x}$', '$e_{\theta\omega_y}$', '$e_{\theta\omega_z}$',...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'southeast');
% 阶数
f_order_DCT = figure;
box on; 
plot(0:length(order_list_DCT)-1, order_list_DCT, '*');
ylabel("Order $l$",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 80 3 9]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:10:80,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',3:1:9,'FontSize',FontSize_1,'FontName','Times New Roman');
grid on;
%% TM-VS
% 速度 
f_velocity_TM = figure;
hold on; box on; grid on;
plot(0:length(velocity_TM)-1, velocity_TM(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(velocity_TM)-1, velocity_TM(:,2), 'g-', 'LineWidth', LineWidth);
plot(0:length(velocity_TM)-1, velocity_TM(:,3), 'b-', 'LineWidth', LineWidth);
plot(0:length(velocity_TM)-1, velocity_TM(:,4), 'c-', 'LineWidth', LineWidth);
plot(0:length(velocity_TM)-1, velocity_TM(:,5), 'm-', 'LineWidth', LineWidth);
plot(0:length(velocity_TM)-1, velocity_TM(:,6), 'y-', 'LineWidth', LineWidth);
ylabel("Velocities $\nu$ and $\omega$",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 140 -1.5 2]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:20:140,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',-1.5:0.5:2,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'LineWidth', LineWidth);
legend('$\nu_x$', '$\nu_y$', '$\nu_z$', '$\omega_x$', '$\omega_y$', '$\omega_z$',...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'northeast');
% 位姿误差
f_pose_TM = figure;
hold on; box on; grid on;
plot(0:length(error_pose_TM)-1, error_pose_TM(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(error_pose_TM)-1, error_pose_TM(:,2), 'g-', 'LineWidth', LineWidth);
plot(0:length(error_pose_TM)-1, error_pose_TM(:,3), 'b-', 'LineWidth', LineWidth);
plot(0:length(error_pose_TM)-1, error_pose_TM(:,4), 'c-', 'LineWidth', LineWidth);
plot(0:length(error_pose_TM)-1, error_pose_TM(:,5), 'm-', 'LineWidth', LineWidth);
plot(0:length(error_pose_TM)-1, error_pose_TM(:,6), 'y-', 'LineWidth', LineWidth);
ylabel("Errors on pose",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 140 -0.8 0.6]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:20:140,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',-0.8:0.2:0.6,'FontSize',FontSize_1,'FontName','Times New Roman');
legend('$e_{t_x}$', '$e_{t_y}$', '$e_{t_z}$', '$e_{\theta\omega_x}$', '$e_{\theta\omega_y}$', '$e_{\theta\omega_z}$',...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'southeast');
% 阶数
f_order_TM = figure;
box on; 
plot(0:length(order_list_TM)-1, order_list_TM, '*');
ylabel("Order $l$",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 140 3 12]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:20:140,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',3:1:12,'FontSize',FontSize_1,'FontName','Times New Roman');
grid on;
%% KM-VS
% 速度 
f_velocity_KM = figure;
hold on; box on; grid on;
plot(0:length(velocity_KM)-1, velocity_KM(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(velocity_KM)-1, velocity_KM(:,2), 'g-', 'LineWidth', LineWidth);
plot(0:length(velocity_KM)-1, velocity_KM(:,3), 'b-', 'LineWidth', LineWidth);
plot(0:length(velocity_KM)-1, velocity_KM(:,4), 'c-', 'LineWidth', LineWidth);
plot(0:length(velocity_KM)-1, velocity_KM(:,5), 'm-', 'LineWidth', LineWidth);
plot(0:length(velocity_KM)-1, velocity_KM(:,6), 'y-', 'LineWidth', LineWidth);
ylabel("Velocities $\nu$ and $\omega$",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 120 -6 8]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:20:120,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',-6:2:8,'FontSize',FontSize_1,'FontName','Times New Roman');
legend('$\nu_x$', '$\nu_y$', '$\nu_z$', '$\omega_x$', '$\omega_y$', '$\omega_z$',...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'northeast');
% 位姿误差
f_pose_KM = figure;
hold on; box on; grid on;
plot(0:length(error_pose_KM)-1, error_pose_KM(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(error_pose_KM)-1, error_pose_KM(:,2), 'g-', 'LineWidth', LineWidth);
plot(0:length(error_pose_KM)-1, error_pose_KM(:,3), 'b-', 'LineWidth', LineWidth);
plot(0:length(error_pose_KM)-1, error_pose_KM(:,4), 'c-', 'LineWidth', LineWidth);
plot(0:length(error_pose_KM)-1, error_pose_KM(:,5), 'm-', 'LineWidth', LineWidth);
plot(0:length(error_pose_KM)-1, error_pose_KM(:,6), 'y-', 'LineWidth', LineWidth);
ylabel("Errors on pose",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 120 -0.8 0.6]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:20:120,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',-0.8:0.2:0.6,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'LineWidth', LineWidth);
legend('$e_{t_x}$', '$e_{t_y}$', '$e_{t_z}$', '$e_{\theta\omega_x}$', '$e_{\theta\omega_y}$', '$e_{\theta\omega_z}$',...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'southeast');
% 阶数
f_order_KM = figure;
box on; 
plot(0:length(order_list_KM)-1, order_list_KM, '*');
ylabel("Order $l$",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 120 3 9]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:20:120,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',3:1:9,'FontSize',FontSize_1,'FontName','Times New Roman');
grid on;
% 参数 p
f_p_KM = figure;
box on; hold on; grid on;
plot(0:length(p_list_KM)-1, p_list_KM(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(p_list_KM)-1, p_list_KM(:,2), 'g-', 'LineWidth', LineWidth);
ylabel("Parameters of KM",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 120 0.5 0.6]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:20:120,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',0.5:0.02:0.6,'FontSize',FontSize_1,'FontName','Times New Roman');
legend('${}^{\alpha}p$', '${}^{\beta}p$', ...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'northeast');
%% HM-VS
% 速度 
f_velocity_HM = figure;
hold on; box on; grid on;
plot(0:length(velocity_HM)-1, velocity_HM(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(velocity_HM)-1, velocity_HM(:,2), 'g-', 'LineWidth', LineWidth);
plot(0:length(velocity_HM)-1, velocity_HM(:,3), 'b-', 'LineWidth', LineWidth);
plot(0:length(velocity_HM)-1, velocity_HM(:,4), 'c-', 'LineWidth', LineWidth);
plot(0:length(velocity_HM)-1, velocity_HM(:,5), 'm-', 'LineWidth', LineWidth);
plot(0:length(velocity_HM)-1, velocity_HM(:,6), 'y-', 'LineWidth', LineWidth);
ylabel("Velocities $\nu$ and $\omega$",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 80 -1 6]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:10:80,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',-1:1:6,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'LineWidth', LineWidth);
legend('$\nu_x$', '$\nu_y$', '$\nu_z$', '$\omega_x$', '$\omega_y$', '$\omega_z$',...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'northeast');
% 位姿误差
f_pose_HM = figure;
hold on; box on; grid on;
plot(0:length(error_pose_HM)-1, error_pose_HM(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(error_pose_HM)-1, error_pose_HM(:,2), 'g-', 'LineWidth', LineWidth);
plot(0:length(error_pose_HM)-1, error_pose_HM(:,3), 'b-', 'LineWidth', LineWidth);
plot(0:length(error_pose_HM)-1, error_pose_HM(:,4), 'c-', 'LineWidth', LineWidth);
plot(0:length(error_pose_HM)-1, error_pose_HM(:,5), 'm-', 'LineWidth', LineWidth);
plot(0:length(error_pose_HM)-1, error_pose_HM(:,6), 'y-', 'LineWidth', LineWidth);
ylabel("Errors on pose",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 80 -0.8 0.6]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:10:80,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',-0.8:0.2:0.6,'FontSize',FontSize_1,'FontName','Times New Roman');
legend('$e_{t_x}$', '$e_{t_y}$', '$e_{t_z}$', '$e_{\theta\omega_x}$', '$e_{\theta\omega_y}$', '$e_{\theta\omega_z}$',...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'southeast');
% 阶数
f_order_HM = figure;
box on; 
plot(0:length(order_list_HM)-1, order_list_HM, '*');
ylabel("Order $l$",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 80 3 9]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:10:80,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',3:1:9,'FontSize',FontSize_1,'FontName','Times New Roman');
grid on;
% ab参数
f_ab_HM = figure;
box on; hold on; grid on;
plot(0:length(parameter_ab_xy_HM)-1, parameter_ab_xy_HM(:,1), 'r-', 'LineWidth', LineWidth);
plot(0:length(parameter_ab_xy_HM)-1, parameter_ab_xy_HM(:,2), 'g-', 'LineWidth', LineWidth);
plot(0:length(parameter_ab_xy_HM)-1, parameter_ab_xy_HM(:,3), 'b-', 'LineWidth', LineWidth);
plot(0:length(parameter_ab_xy_HM)-1, parameter_ab_xy_HM(:,4), 'c-', 'LineWidth', LineWidth);
ylabel("Parameters of HM",'Interpreter','latex','FontSize',FontSize_1,'FontName','Times New Roman');
xlabel('Iterations');
axis([0 80 0 80]);
set(gca,'LineWidth', LineWidth);
set(gca,'xtick',0:10:80,'FontSize',FontSize_1,'FontName','Times New Roman');
set(gca,'ytick',0:10:80,'FontSize',FontSize_1,'FontName','Times New Roman');
legend('${}^{\alpha}a$', '${}^{\alpha}b$', '${}^{\beta}a$', '${}^{\beta}b$',...
    'Interpreter','latex','FontName','Times New Roman', 'NumColumns', 2, ...
    'FontSize',FontSize_1,'LineWidth', LineWidth,'Location', 'northeast');

