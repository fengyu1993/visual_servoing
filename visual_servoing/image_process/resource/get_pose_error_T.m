%% 输入位姿 , 期望位姿-> 位姿旋量误差，位姿矩阵
function [error_pose, T_lise] = get_pose_error_T(camera_pose, pose_desrired)

    num = size(camera_pose, 1) / 4;
    T_lise = zeros(4, 4, num);
    error_pose = zeros(num, 6);

    for i = 1 : num
        T_lise(:, :, i) = camera_pose(4*i-3 : 4*i, :);
    end
    
    for i = 1 : num
        T = TransInv(pose_desrired) * T_lise(:, :, i);
        temp = se3ToVec(MatrixLog6(T));
        error_pose(i, :) = [temp(4:6)', temp(1:3)'];
    end

end