%% 将excel数据存储为.mat格式 3D实验数据
clc;
clear;
close all
%% 准备
location_name = "./data/";
camera_velocity_name = "camera velocity";
camera_pose_name = "camera pose";
camera_desired_pose_name = "camera desired pose";
error_feature_ave_name = "error feature";
error_pixel_ave_name = "error pixel ave";
order_name = "order";
pxy_name = ["px", "py"];
abxy_name = ["ax", "bx", "ay", "by"];
%% 读取TM_VS_data
[~, ~, TM_VS_data] = xlsread(location_name+'TM_VS_data_3D_Complex.xlsx');
TM_VS_camera_velocity = get_camera_velocity_data(TM_VS_data, camera_velocity_name, camera_pose_name);
TM_VS_camera_pose = get_camera_pose_data(TM_VS_data, camera_pose_name, camera_desired_pose_name);
TM_VS_camera_desired_pose = get_camera_desired_pose_data(TM_VS_data, camera_desired_pose_name, error_feature_ave_name);
TM_VS_error_feature = get_camera_error_feature_data(TM_VS_data, error_feature_ave_name, error_pixel_ave_name);
TM_VS_error_pixel = get_camera_error_pixel_data(TM_VS_data, error_pixel_ave_name, order_name);
TM_VS_order = get_camera_order_data(TM_VS_data, order_name, "end");
%% 读取KM_VS_data
[~, ~, KM_VS_data] = xlsread(location_name+'KM_VS_data_3D_Complex.xlsx');
KM_VS_camera_velocity = get_camera_velocity_data(KM_VS_data, camera_velocity_name, camera_pose_name);
KM_VS_camera_pose = get_camera_pose_data(KM_VS_data, camera_pose_name, camera_desired_pose_name);
KM_VS_camera_desired_pose = get_camera_desired_pose_data(KM_VS_data, camera_desired_pose_name, error_feature_ave_name);
KM_VS_error_feature = get_camera_error_feature_data(KM_VS_data, error_feature_ave_name, error_pixel_ave_name);
KM_VS_error_pixel = get_camera_error_pixel_data(KM_VS_data, error_pixel_ave_name, order_name);
KM_VS_order = get_camera_order_data(KM_VS_data, order_name, pxy_name(1));
KM_VS_pxy = get_camera_KM_pxy_data(KM_VS_data, pxy_name);
%% 读取HM_VS_data
[~, ~, HM_VS_data] = xlsread(location_name+'HM_VS_data_3D_Complex.xlsx');
HM_VS_camera_velocity = get_camera_velocity_data(HM_VS_data, camera_velocity_name, camera_pose_name);
HM_VS_camera_pose = get_camera_pose_data(HM_VS_data, camera_pose_name, camera_desired_pose_name);
HM_VS_camera_desired_pose = get_camera_desired_pose_data(HM_VS_data, camera_desired_pose_name, error_feature_ave_name);
HM_VS_error_feature = get_camera_error_feature_data(HM_VS_data, error_feature_ave_name, error_pixel_ave_name);
HM_VS_error_pixel = get_camera_error_pixel_data(HM_VS_data, error_pixel_ave_name, order_name);
HM_VS_order = get_camera_order_data(HM_VS_data, order_name, abxy_name(1));
HM_VS_abxy = get_camera_HM_abxy_data(HM_VS_data, abxy_name);
%% 读取图像
image_rgb_desired = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/image_rgb_desired.png");
image_rgb_init = imread("/home/cyh/Work/visual_servoing_ws/src/visual_servoing/robot_control_VS/param/image_rgb_init.png");
%% 保存数据
save("TM_VS_experience_data_3D_Complex.mat", "TM_VS_camera_velocity", "TM_VS_camera_pose", "TM_VS_camera_desired_pose", "TM_VS_error_feature", "TM_VS_error_pixel", "TM_VS_order");
save("KM_VS_experience_data_3D_Complex.mat", "KM_VS_camera_velocity", "KM_VS_camera_pose", "KM_VS_camera_desired_pose", "KM_VS_error_feature", "KM_VS_error_pixel", "KM_VS_order", "KM_VS_pxy");
save("HM_VS_experience_data_3D_Complex.mat", "HM_VS_camera_velocity", "HM_VS_camera_pose", "HM_VS_camera_desired_pose", "HM_VS_error_feature", "HM_VS_error_pixel", "HM_VS_order", "HM_VS_abxy");
imwrite(image_rgb_desired, "image_rgb_desired_3D_Complex.png");
imwrite(image_rgb_init, "image_rgb_init_3D_Complex.png");




%% 读取速度
function camera_velocity = get_camera_velocity_data(data, name_begin, name_end)
    [row_begin, ~] = find(strcmp(data, name_begin));
    [row_end, ~] = find(strcmp(data, name_end));
    camera_velocity = cell2mat(data(row_begin+1:row_end-1, 1:6));
end

%% 读取位姿
function camera_pose = get_camera_pose_data(data, name_begin, name_end)
    [row_begin, ~] = find(strcmp(data, name_begin));
    [row_end, ~] = find(strcmp(data, name_end));
    camera_pose = cell2mat(data(row_begin+1:row_end-1, 1:4));
end

%% 读取期望位姿
function camera_desired_pose = get_camera_desired_pose_data(data, name_begin, name_end)
    [row_begin, ~] = find(strcmp(data, name_begin));
    [row_end, ~] = find(strcmp(data, name_end));
    camera_desired_pose = cell2mat(data(row_begin+1:row_end-1, 1:4));    
end

%% 读取特征平均误差
function error_feature = get_camera_error_feature_data(data, name_begin, name_end)
    [row_begin, ~] = find(strcmp(data, name_begin));
    [row_end, ~] = find(strcmp(data, name_end));
    if isempty(row_end)
        row_end = size(data, 1);
    end
    error_feature = cell2mat(data(row_begin+1:row_end-1, 1));
end

%% 读取像素平均误差
function error_pixel = get_camera_error_pixel_data(data, name_begin, name_end)
    [row_begin, ~] = find(strcmp(data, name_begin));
    [row_end, ~] = find(strcmp(data, name_end));
    error_pixel = cell2mat(data(row_begin+1:row_end-1, 1));
end

%% 读取阶数
function order = get_camera_order_data(data, name_begin, name_end)
    [row_begin, ~] = find(strcmp(data, name_begin));
    if name_end == "end"
        order = cell2mat(data(row_begin+1:end, 1));
    else
        [row_end, ~] = find(strcmp(data, name_end));
         order = cell2mat(data(row_begin+1:row_end-1, 1));
    end
end

%% 读取KM参数
function pxy = get_camera_KM_pxy_data(data, name_begin)
    [row_begin_1, ~] = find(strcmp(data, name_begin(1)));
    [row_begin_2, ~] = find(strcmp(data, name_begin(2)));
    px = cell2mat(data(row_begin_1+1:row_begin_2-1, 1));
    py = cell2mat(data(row_begin_2+1:end, 1));
    pxy = [px, py];
end

%% 读取HM参数
function abxy = get_camera_HM_abxy_data(data, name_begin)
    [row_begin_1, ~] = find(strcmp(data, name_begin(1)));
    [row_begin_2, ~] = find(strcmp(data, name_begin(2)));
    [row_begin_3, ~] = find(strcmp(data, name_begin(3)));
    [row_begin_4, ~] = find(strcmp(data, name_begin(4)));
    ax = cell2mat(data(row_begin_1+1:row_begin_2-1, 1));
    bx = cell2mat(data(row_begin_2+1:row_begin_3-1, 1));
    ay = cell2mat(data(row_begin_3+1:row_begin_4-1, 1));
    by = cell2mat(data(row_begin_4+1:end, 1));
    abxy = [ax, bx, ay, by];
end
