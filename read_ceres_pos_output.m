function [delta_rot, delta_t] = read_ceres_pos_output() 
    filename = 'ceres_output_singleViewPoseAdjuster.txt';
    f = fopen(filename, 'r');
    buffer = fscanf(f,'%lf'); 
    delta_rot = zeros(3,3) ; 
    delta_t = zeros(3,1) ; 
    for i=1:3
        for j=1:3
            delta_rot(j,i) = buffer(3*(i-1)+j) ; 
        end
    end
    
    for i=1:3
        delta_t(i) = buffer(9+i) ; 
    end
    fclose(f) ; 
end