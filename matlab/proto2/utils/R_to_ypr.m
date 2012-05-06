function [y p r] = R_to_ypr(R)

    n = R(:,1);
    o = R(:,2);
    a = R(:,3);
    
    y = atan2(n(2), n(1));
    p = atan2(-n(3), n(1)*cos(y)+n(2)*sin(y));
    r = atan2(a(1)*sin(y)-a(2)*cos(y), -o(1)*sin(y)+o(2)*cos(y));

end