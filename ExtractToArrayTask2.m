seq = [2,10,4,8,2,9] ;
frm = [98,1,197,126,90,42] ; 
id = [1,0,20,12,1,1] ; 
array = zeros(6,8) ; 

for i=1:6
    tracklets = readLabels('../data_tracking_label_2/training/label_02',seq(i)) ; 
    m = tracklets(frm(i)) ; 
    p =  m{1,1} ; 
    s_p = size(p) ; 
    s_p = s_p(2) ; 
    for j=1:s_p 
        if p(1,j).id == id(i)
            array(i,:) = [seq(i), frm(i), id(i), p(1,j).x1, p(1,j).y1, p(1,j).x2, p(1,j).y2, p(1,j).ry] ;  
        end
    end
end

disp(array) ; 

    