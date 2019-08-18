function [ init_error pose_error shape_error ] = reprojection_error(keypoints, init_pts, pose_pts, shape_pts)

    init_error = sum(sqrt((keypoints(:,1) - init_pts(:,1)).^2 + (keypoints(:,2) - init_pts(:,2)).^2));
    pose_error = sum(sqrt((keypoints(:,1) - pose_pts(:,1)).^2 + (keypoints(:,2) - pose_pts(:,2)).^2));
    shape_error = sum(sqrt((keypoints(:,1) - shape_pts(:,1)).^2 + (keypoints(:,2) - shape_pts(:,2)).^2));    

end