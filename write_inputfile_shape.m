function write_inputfile_shape(keypoints, car_points, car_centre, H, W, L, K, ry, lambda, eig_vec, kp_lookup, delta_rot, delta_t)

    numViews = 1 ; 
    numPts = 14 ; 
    numObs = 14 ; 
    
    disp(delta_rot) ; 
    disp(delta_t) ; 
    
    %Create file
    file_to_be_created = "ceres_input_singleViewShapeAdjuster.txt" ; 
    f = fopen(file_to_be_created, 'w') ;

    % Write number of views, keypts, and obs
    fprintf(f,'%d %d %d\n\n',numViews, numPts, numObs);

    % Write can centre coords
    fprintf(f, '%f %f %f\n\n',car_centre(1), car_centre(2), car_centre(3));

    % Write height, width, and length
    fprintf(f, '%f %f %f\n\n', H, W, L);

    % Write K matrix values
    for i=1:3
        for j=1:3
            fprintf(f,'%f ',K(i,j)) ; 
        end
        fprintf(f,'\n') ; 
    end

    fprintf(f,'\n') ; 

    % Write observations, ie keypoints
    for i=1:size(keypoints,1)
        fprintf(f,'%f %f\n', keypoints(i,1) , keypoints(i,2)) ;
    end
    
    fprintf(f,'\n') ; 
    % Write observation weights, i.e combination of confidence values and
    % kplookup

    angle = round(rad2deg(ry)) ; 
    if angle <= 0
        angle = angle + 360 ; 
    end

    for i = 1:size(keypoints,1)
        fprintf(f, '%f ',(0.3*keypoints(i,3) + 0.7 * kp_lookup(angle,i)./sum(kp_lookup(angle,:)))) ; 
    end

    fprintf(f,'\n\n') ;

    for i=1:numObs
        fprintf(f,'%f %f %f\n', car_points(i,1), car_points(i,2),car_points(i,3)) ; 
    end

    fprintf(f,'\n\n') ; 
    
    
    for i=1:5
        for j=1:3*numPts
            fprintf(f,'%f ',eig_vec(i,j)) ; 
        end
        fprintf(f,'\n') ;  
    end
    
    fprintf(f,'\n') ; 
    
    for i=1:5
        fprintf(f,'%f ', lambda(i)) ; 
    end
    
    fprintf(f,'\n\n') ; 
    
    for i=1:3
        for j=1:3
            fprintf(f,'%f ', delta_rot(j,i)) ;
        end
    end
    
    fprintf(f,'\n\n') ; 
    
    for i=1:3
        fprintf(f,'%f ', delta_t(i)) ; 
    end
    
    fclose(f) ; 
    
end
