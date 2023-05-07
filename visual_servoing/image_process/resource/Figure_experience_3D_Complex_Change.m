%% 3D 复杂实验 
clc;
close all;
clear;
%%
resolution_x = 640;
resolution_y = 480;
%% 数据加载
load('HM_VS_experience_data_3D_Complex_Change.mat');
error_feature_HM = HM_VS_error_feature;
error_pixel_HM = HM_VS_error_pixel*resolution_x*resolution_y;
[error_pose_HM, T_HM] = get_pose_error_T(HM_VS_camera_pose, HM_VS_camera_desired_pose);
velocity_HM = HM_VS_camera_velocity;
order_list_HM = HM_VS_order;
parameter_ab_xy_HM = HM_VS_abxy;
%% 全局信息
LineWidth = 1.8;
FontSize_1 = 18;
FontSize_2 = 15;
%% 像素误差
f_pixel = figure;
hold on; box on; grid on;
plot(0:length(error_pixel_HM)-1, error_pixel_HM, 'r-', 'LineWidth', LineWidth);
ylabel("$||\bf{I} - \bf{I}^*||^2$",'Interpreter','latex','FontSize',FontSize_2,'FontName','Times New Roman');
xlabel('Iterations');
% axis([0 140 0 12e4])
% set(gca,'xtick',0:20:140,'FontSize',FontSize_2,'FontName','Times New Roman');
% set(gca,'ytick',0:2e4:12e4,'FontSize',FontSize_2,'FontName','Times New Roman');
% set(gca,'LineWidth', LineWidth);
legend('HM-VS','FontName','Times New Roman', ...
    'FontSize',FontSize_2*0.8,'LineWidth', LineWidth,'Location', 'northeast');


