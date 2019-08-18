function [wireframe] = read_ceres_shape_output() 

wireframe = zeros(14,3) ; 
filename = 'ceres_output_singleViewShapeAdjuster.txt';
f = fopen(filename, 'r');
buffer = fscanf(f,'%lf'); 

for i=1:14
    for j=1:3
        wireframe(i,j) = buffer(3*(i-1) + j) ; 
    end
end

fclose(f) ; 

end