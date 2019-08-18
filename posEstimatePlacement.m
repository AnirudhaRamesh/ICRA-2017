! rm ceres_input_singleViewShapeAdjuster.txt ceres_input_singleViewPoseAdjuster.txt ceres_output_singleViewShapeAdjuster.txt ceres_output_singleViewPoseAdjuster.txt 


seq = [ 2,10,  4,  8, 2, 9]; %Sequences
frm = [98, 1,197,126,90,42]; %Frames
id  = [ 1, 0, 20, 12, 1, 1]; %CarID's


folder = ["0002", "0010","0004","0008","0002","0009"] ;
file = ["000098","000001","000197","000126","000090","000042"] ; 

avg_l = 3.8600 ; 
avg_w = 1.6362 ; 
avg_h = 1.5208 ; 

kp_lookup = load('kpLookup_azimuth.mat') ; 
kp_lookup = kp_lookup.kpLookup ; 


lambda =  [ 0.25, 0.27 ,0.01, -0.08, -0.05 ] ; 

vectors1 = [-0.151346 0.218373 0.088189 0.178544 0.222476 0.088167 -0.130965 -0.199768 0.073618 0.161234 -0.195187 0.071677 -0.108222 0.338475 0.013481 0.134977 0.343590 0.013548 -0.087232 -0.310159 -0.012570 0.120979 -0.307142 -0.013180 -0.157611 0.089988 -0.049292 0.182542 0.094109 -0.049571 -0.095605 0.049274 -0.114369 0.119684 0.051306 -0.114337 -0.086239 -0.146814 -0.115755 0.112563 -0.146889 -0.115872 
-0.098323 -0.065454 0.021927 0.087594 -0.045426 0.015041 0.173314 -0.027648 -0.026248 -0.197289 0.034519 -0.027247 -0.334409 0.314357 -0.024683 0.322223 0.313363 -0.022762 0.230236 0.202303 0.013518 -0.218483 0.230444 0.017632 -0.082111 -0.240761 -0.035543 0.069127 -0.239921 -0.034522 -0.073834 -0.244503 0.002144 0.059896 -0.247034 0.003619 0.074159 -0.014541 0.097339 -0.095463 -0.013529 0.101674 
0.144907 0.070979 0.099813 -0.171440 0.121961 0.101635 0.029681 0.074285 0.073524 -0.052592 0.054118 0.080406 0.119029 0.056095 -0.007987 -0.095969 0.039598 -0.007782 -0.049881 0.310210 0.007311 0.027442 0.288766 0.009912 0.120991 0.071623 -0.014953 -0.105925 0.095147 -0.009445 0.001524 -0.045424 -0.086102 0.038969 -0.035048 -0.081712 -0.032435 -0.538018 -0.102501 0.054810 -0.548985 -0.097703 
0.303420 0.022644 -0.223071 -0.239204 -0.000371 -0.226790 -0.230123 -0.007481 -0.154541 0.261133 0.027120 -0.155754 -0.102530 0.155292 0.034942 0.086396 0.111854 0.033742 -0.377372 0.040747 -0.055970 0.327202 0.048094 -0.077505 -0.027493 -0.007384 0.099710 0.015499 -0.001164 0.088955 -0.113268 -0.229368 0.202414 0.033187 -0.181117 0.200235 -0.042415 -0.038634 0.173792 0.023206 0.016464 0.160507 
0.109164 -0.362282 -0.059697 -0.072880 -0.359107 -0.069168 0.082780 -0.077322 -0.023031 -0.102315 0.046584 -0.014162 -0.176465 0.092178 -0.014550 0.202195 0.095555 -0.011093 -0.032977 -0.110770 -0.015152 0.039475 -0.062289 -0.018643 -0.070913 0.262228 -0.023711 0.076054 0.230184 -0.026444 0.058884 0.324946 0.029187 -0.078275 0.278465 0.029191 0.298251 -0.167472 0.083653 -0.297588 -0.172290 0.090366 
] ; 


temp = [2.5447 -3.7577 -1.5125
-3.0188 -3.8300 -1.5128
2.2950 3.4357 -1.2746
-2.8187 3.3599 -1.2430
1.8002 -5.7926 -0.2371
-2.2685 -5.8769 -0.2381
1.5809 5.3567 0.2227
-2.1554 5.3089 0.2352
2.6917 -1.6341 0.8413
-3.1261 -1.7055 0.8464
1.6331 -0.9118 1.9708
-2.0508 -0.9471 1.9701
1.4829 2.6154 2.0137
-1.9433 2.6160 2.0160
] ; 

keypoints = [4.25,30.75,0.59223163127899,17.25,43.75,0.066828712821007,17.25,44.25,0.66479653120041,4.25,30.75,0.02288468554616,27.75,30.75,0.041342653334141,56.75,30.75,0.017454901710153,27.75,30.75,0.76314502954483,56.75,30.75,0.62060898542404,7.75,16.25,0.67381626367569,57.25,30.75,0.031822223216295,15.25,4.25,0.82063239812851,46.75,4.75,0.037195831537247,26.25,6.25,0.83392602205276,48.25,6.25,0.6028813123703
4.25,44.25,0.4764512181282,5.75,45.75,0.032890759408474,14.75,48.25,0.48485720157623,56.25,50.25,0.21720230579376,22.25,32.75,0.026767304167151,14.75,49.25,0.0099000940099359,22.25,32.75,0.64503210783005,4.25,44.75,0.022746879607439,5.75,19.75,0.78437912464142,57.75,22.25,0.044407557696104,13.75,6.25,0.77545911073685,48.75,4.75,0.023976393043995,23.75,8.25,0.72658860683441,49.75,5.75,0.72363257408142
38.25,30.75,0.03088827803731,62.25,32.25,0.18814358115196,38.25,30.75,0.024294823408127,48.25,46.25,0.63121372461319,37.75,6.25,0.016919825226068,38.25,30.75,0.018494693562388,4.25,31.75,0.28886997699738,38.25,30.25,0.69038677215576,48.25,46.75,0.03056682087481,59.25,15.75,0.75671970844269,8.75,5.25,0.027118192985654,50.25,4.25,0.82626760005951,8.25,5.75,0.84216040372849,37.75,6.25,0.79417669773102
40.25,48.75,0.69430673122406,8.25,52.25,0.32315447926521,60.25,38.75,0.72894793748856,40.25,48.75,0.056191854178905,27.75,37.75,0.72420978546143,4.75,37.75,0.79827833175659,9.25,52.75,0.021689338609576,60.25,38.75,0.013817569240928,48.25,21.75,0.84908664226532,16.75,21.75,0.067913681268692,44.25,6.25,0.81060945987701,24.25,6.25,0.77422273159027,56.25,6.25,0.81189292669296,36.25,3.75,0.08854191750288
4.25,36.25,0.59156334400177,14.25,45.75,0.028220094740391,14.25,45.75,0.70071601867676,4.25,36.25,0.02361143194139,25.25,30.25,0.036072250455618,57.25,29.25,0.017704103142023,25.75,30.25,0.7234982252121,57.75,29.75,0.69848084449768,5.75,16.25,0.61246508359909,57.25,29.25,0.025887083262205,16.75,4.25,0.46049347519875,48.25,4.25,0.021271742880344,24.25,5.75,0.68283551931381,51.75,8.25,0.58403098583221
15.75,51.75,0.78975474834442,48.75,43.25,0.058626063168049,48.25,43.25,0.68798005580902,15.75,51.75,0.11900161206722,6.25,42.25,0.60396921634674,42.25,8.25,0.014241943135858,6.25,42.75,0.010095393285155,49.25,43.25,0.010161536745727,26.25,26.25,0.82884812355042,16.25,25.75,0.021373983472586,27.75,11.25,0.71640574932098,24.25,11.25,0.14733704924583,42.25,8.25,0.86258155107498,42.25,8.25,0.06828948110342
] ; 

K = [721.53,0,609.55;0,721.53,172.85;0,0,1];

rot_x = [ 1 0 0 ; 0 cos(pi/2) -sin(pi/2) ; 0 sin(pi/2) cos(pi/2) ; ]; 
rot_z = [cos(pi) sin(pi) 0;-sin(pi) cos(pi) 0; 0 0 1 ] ; 
rot_y = [cos(pi/2) 0 sin(pi/2) ; 0 1 0 ; -sin(pi/2) 0 cos(pi/2); ] ;

[temp, vectors] =  ScaleAvg(temp,avg_h,avg_w, avg_l, vectors1) ; 
temp_zrot = rot_z * temp' ;
temp_xzrot = rot_x * temp_zrot ;
temp_yxzrot = rot_y * temp_xzrot ;

reproj_init_error_arr = [] ; reproj_pos_error_arr = [] ;  reproj_shape_error_arr = [] ; 
translation_init_error_arr = [] ; translation_pos_error_arr = [] ; 
rotation_error_arr_init = [] ; rotation_error_arr_after_pose = [] ; 

change_rot_x = [1 0 0] ; 

for i=1:1
tracklets = readLabels('../data_tracking_label_2/training/label_02',seq(i)) ;
m = tracklets(frm(i)+1) ;
p =  m{1,1} ;
s_p = size(p) ; 
s_p = s_p(2) ; 
for j=1:s_p 
    if p(1,j).id == id(i) 
        ry = p(j).ry  + normrnd(0,0.005) ;
        ry_actual = p(j).ry ; 
        x2 = p(j).x2 ; 
        y2 = p(j).y2 ;
        x1 = p(j).x1 ; 
        y1 = p(j).y1 ;
        trans_actual = p(j).t ; 
        disp(p(1,j).id) ; 
    end
end
rot_ry = [cos(ry) 0 sin(ry) ; 0 1 0 ; -sin(ry) 0 cos(ry); ] ;
rot_ry_actual = [ cos(ry_actual) 0 sin(ry_actual) ; 0 1 0 ; -sin(ry_actual) 0 cos(ry_actual) ; ] ; 

temp_rot = rot_ry * temp_yxzrot ; 

n = [ 0 -1 0 ] ;

h =  1.65 ;

b =  [ (x1+x2)/2 ; y2 ; 1 ] ; 

B = (-h * inv(K) * b ) / (n*(inv(K) * b));

position_estimate = temp_rot + B ; 

%visualizeWireframe3D(position_estimate) ; 

for k=1:5
    v = vectors(k,:) ; 
    v = reshape(v, [3 14] ) ; 
    v = rot_z * v ; 
    v = rot_x * v ; 
    v = rot_y * v ; 
    v = rot_ry * v ;
    %v = scaling * v ; 
    % rotation and scaling 
    v = reshape(v, [1 42] ) ; 
    vectors(k,:) = v ; 
end

%visualizeWireframe3D(vectors) ; 

avg_l = 3.8600  ; 
avg_w = 1.6362  ;
avg_h =  1.5208  ; 

%Projecting it to 2D
position_estimate_pos_adjustment = position_estimate + [0 ;-avg_h/2 ; avg_w/2;];
pos_temp = K * position_estimate_pos_adjustment; 
pos_2d = pos_temp ./ pos_temp(3,:) ; 
%pos_2d = position_estimate .* 10 ; folder = ["0002", "0010","0004","0008","0002","0009"] ;
pos_2d = pos_2d(1:2,:) ; 

init_wireframe_image = pos_2d ; 

img  = imread(strcat('../data_tracking_image_2/training/image_02/',folder(i),'/',file(i),'.png')) ; 

text_title = "seq number " + seq(i) + " frame number " + frm(i) + " car id " + id(i) ;  
a = figure,
sgtitle(text_title) ; 
subplot(2,2,1) ; 
visualizeWireframe2D(img, pos_2d) ; 
title('Wireframe Initialization') ;

bb_x = x2-x1 ; 
bb_y = y2-y1 ; 

kp_disp = get_keypoints(keypoints(i,:), bb_x, bb_y, x1, y1) ; 

kpd = reshape(kp_disp(:,:), [2 14]) ;

subplot(2,2,2);
visualizeKeypoints2D(img,kpd) ; 
title('Keypoints') ; 



%% Single View Pose Adjustment 

% writing input pos file 

kpt_pos_adj = reshape(keypoints(i,:), [3 14] ) ; 

kpt_pos_adj = [kpd(1:2,:);kpt_pos_adj(3,:)];
write_inputfile_pos(kpt_pos_adj', position_estimate_pos_adjustment', B + [0 ;-avg_h/2 ; avg_w/2;], avg_h, avg_w, avg_l, K, ry, lambda, vectors, kp_lookup) ; 

! ./singleViewPoseAdjuster

[delta_rot, delta_t] = read_ceres_pos_output() ; 

% Display pose adjusted wireframe
pause(0.5) ; 

position_estimate_pose_adjusted =  (delta_rot * position_estimate_pos_adjustment) + delta_t; 
trans_after_pos_adjustment = ( delta_rot * B ) + delta_t ; 

pos_temp = K * position_estimate_pose_adjusted; 
pos_2d = pos_temp ./ pos_temp(3,:) ; 
pos_2d = pos_2d(1:2,:) ; 

pose_adjusted_wireframe_image = pos_2d ; 

rot_after_pose_adj = delta_rot*rot_ry;
angle_ax1 = rotm2axang(rot_after_pose_adj);
angle_ax2 = rotm2axang(rot_ry_actual);
angle_ax3 = rotm2axang(rot_ry) ; 

rotation_error_arr_init = [rotation_error_arr_init ; (abs((angle_ax3(4) - angle_ax2(4))*180/pi))]; 
rotation_error_arr_after_pose = [rotation_error_arr_after_pose; (abs((angle_ax1(4) - angle_ax2(4))*180/pi))]; 

for k=1:5
    v = vectors(k,:) ; 
    v = reshape(v, [3 14] ) ; 
    v = delta_rot * v ; 
    %v = scaling * v ; 
    % rotation and scaling 
    v = reshape(v, [1 42] ) ; 
    vectors(k,:) = v ; 
end

subplot(2,2,3) ; 
visualizeWireframe2D(img, pos_2d) ; 
title('After pose adjustment') ; 

%% Single View Shape Adjustment

% writing input shape file 

write_inputfile_shape(kpt_pos_adj',position_estimate_pos_adjustment', B + [ 0 ; -avg_h/2 ; avg_w/2  ;] , avg_h, avg_w, avg_l, K, ry, lambda, vectors, kp_lookup, delta_rot, delta_t ) ; 
! ./singleViewShapeAdjuster
                
[ wireframe ] = read_ceres_shape_output() ; 

pos_temp = K * wireframe'; 
pos_2d = pos_temp ./ pos_temp(3,:) ; 
pos_2d = pos_2d(1:2,:) ; 

shape_adjusted_wireframe_image = pos_2d ; 

subplot(2,2,4) ; 
%a = figure,
visualizeWireframe2D(img, pos_2d) ; 
title('After shape adjustment pose : pnp - 0.001 , trans reg - 0.8  ; shape : lambdaerror -  0.015 , lambda reg - 0.05') ; 
%pause(2.0) ; 
%saveas(a,sprintf([pwd '/result_3/%d.jpg'],i))  ; 
pause(0.5) ; 

%% Getting Quantitative Error 

% Getting Reprojection Error
[ reproj_init_error , reproj_pos_error,  reproj_shape_error ] = reprojection_error(kpt_pos_adj', init_wireframe_image', pose_adjusted_wireframe_image', shape_adjusted_wireframe_image') ;  
reproj_init_error_arr = [ reproj_init_error_arr ; reproj_init_error ] ; 
reproj_pos_error_arr = [ reproj_pos_error_arr ; reproj_pos_error ] ; 
reproj_shape_error_arr = [ reproj_shape_error_arr ; reproj_shape_error ] ;

% Getting Translation Error
translation_init_error = sqrt(sum((trans_actual' - B).^2)) ;
translation_pos_error = sqrt(sum((trans_actual' - trans_after_pos_adjustment).^2)) ;
translation_init_error_arr = [translation_init_error_arr ; translation_init_error ] ; 
translation_pos_error_arr = [translation_pos_error_arr ; translation_pos_error ] ; 

end

%% Quantitative Error Plots

% Reporjection Error 

err_reproj_plot = figure, 
plot(reproj_init_error_arr) ; 
hold on 
plot(reproj_pos_error_arr) ; 
plot(reproj_shape_error_arr) ; 
xlabel('Image');
ylabel('Reprojection Error');
legend('Initial error','Error after pose adjustment','Error after shape adjustment');
title('Reprojection Error');
hold off ; 
%saveas(err_reproj_plot, [pwd '/result_3/reprojection_error.jpg'] )  ;  

% Translation Error 
err_trans_plot = figure, 
plot(translation_init_error_arr) ; 
hold on
plot(translation_pos_error_arr) ; 
hold off 
xlabel('Image') ; 
ylabel('Translation Error') ; 
legend('Initial Error', 'Error after pose adjustment') ; 
title('Translation Error') ; 
%saveas(err_trans_plot, [pwd '/result_3/translation_error.jpg'])  ; 



% Rotation Error 
error_rotation_plot = figure,
plot(rotation_error_arr_after_pose) ;
xlabel('Image') ; 
ylabel('Rotation Error') ; 
legend('Error After pose adjustment') ; 
title('Rotation Error') ; 
saveas(error_rotation_plot, [pwd '/result_3/rotation_error.jpg'])  ; 




 



