function kp_disp = get_keypoints(keypoints, scale_x, scale_y, x1, y1)

kp_disp = zeros(1,28) ;  
k = 1 ; 
for j=1:42
       if mod(j,3) ~= 0 
           kp_disp(1,k) = keypoints(1,j) ;
           if mod(j,3) == 1
               kp_disp(1,k) = kp_disp(1,k).* (scale_x/64) + x1 ; 
           end
           if mod(j,3) == 2
               kp_disp(1,k) = kp_disp(1,k).* (scale_y/64) + y1 ;
           end
           k = k + 1 ; 
       end
end


end
