function lab2_5()
    l1 = 95;    % Given link lengths
    l2 = 100;
    l3= 100;

    a1 = sym("a1");
    a2 = sym("a2");
    a3 = sym("a3");
    % DH table values
    dh_table = [a1,        l1, 0,  -pi/2;
               -pi/2 + a2, 0,  l2, 0;
                pi/2 + a3, 0,  l3, 0];
    
    
    dh2fk(dh_table)
end

function transform = dh2mat(dhparams)
    theta = dhparams(1); % Extracts components
    d = dhparams(2);
    a = dhparams(3);
    alpha = dhparams(4);
    % DH transformation
    transform = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                 sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                 0,          sin(alpha),             cos(alpha),            d           ;
                 0,          0,                      0,                     1           ];
end

function retMat = dh2fk(dhTable)    
    rows = height(dhTable);                             % Finds number of rows
    retMat = [ 1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];     % Creates identity matrix as base
    for i = 1:rows                                      % loops for number of rows
        retMat = retMat * dh2mat(dhTable(i,:));    % multiplies each transformation matrix
    end
end