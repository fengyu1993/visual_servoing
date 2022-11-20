%% 输入四元数位姿 , 期望位姿四元数-> 位姿旋量误差，位姿矩阵
function [error_pose, T_lise] = get_pose_error_T(camera_pose, q_desrired)

    T_lise = zeros(4, 4, size(camera_pose, 1));
    error_pose = zeros(size(camera_pose, 1), 6);

    for i = 1 : size(camera_pose, 1)
        p = camera_pose(i, 1:3);
        R = quat2dcm(camera_pose(i, 4:end));
        T_lise(:, :, i) = [R, p'; 0 0 0 1];
    end

    T_desrired = quat2dcm(q_desrired);
    for i = 1 : size(camera_pose, 1)
        T = TransInv(T_desrired) * T_lise(:, :, i);
        temp = se3ToVec(MatrixLog6(T));
        error_pose(i, :) = [temp(4:6)', temp(1:3)'];
    end

end