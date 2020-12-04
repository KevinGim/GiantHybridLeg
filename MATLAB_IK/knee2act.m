

function th4 = knee2act(th2, th3, draw)

% Leg Dimensions

L2 = 900;

L3 = 910;

 

%4 Bar Linkage Dimension

L1 = 198;

L4 = 382;

L5 = 960;

L6_1 = 261.4625;

L6_2 = 62.8;

L6 = sqrt(L6_1^2+L6_2^2);

offset_3 = atan(L6_2/L6_1);

 

%%the

O =[0,0];

P02 = [L2 * sin(th2), -L2 * cos(th2)];

P03 = [L1,0];

P06 = [L2 * sin(th2) + L6 * sin(offset_3 + th2 + th3), - L2 * cos(th2) - L6 * cos(offset_3 + th2 + th3)];

P09 = [L2 * sin(th2) + L6_1 * sin(th2 + th3), - L2 * cos(th2) - L6_1 * cos(th2 + th3)];

P0A = [L2 * sin(th2) + L3 * sin(th2 + th3), - L2 * cos(th2) - L3 * cos(th2 + th3)];

 

if draw == 1

    plot(0,0,'k*')

    hold on;

    plot(P02(1),P02(2),'r*')

    hold on;

    plot(P03(1),P03(2),'g*')

    hold on;

    plot(P06(1),P06(2),'b*')

    line([0,P02(1)],[0,P02(2)],'LineWidth',2,'Color','k');

    line([0,P03(1)],[0,P03(2)],'LineWidth',2,'Color','k');

    line([P02(1),P06(1)],[P02(2),P06(2)],'LineWidth',2,'Color','k');

    line([P02(1),P09(1)],[P02(2),P09(2)],'LineWidth',2,'Color','k');

    line([P02(1),P0A(1)],[P02(2),P0A(2)],'LineWidth',2,'Color','k');

    line([P06(1),P09(1)],[P06(2),P09(2)],'LineWidth',2,'Color','k');

    line([P03(1),P02(1)],[P03(2),P02(2)],'LineWidth',1,'Color','r');

    line([P03(1),P06(1)],[P03(2),P06(2)],'LineWidth',1,'Color','r');

   

    grid on;

    axis([-500 500 -1900 100])

    axis equal

    hold off

    drawnow()

end

 

 

L2_cal = norm(P02-O);

L3_cal = norm(P09-P02);

L6_2_cal = norm(P09-P06);

v1 = P03 - P02;

v2 = P06 - P03;

 

v1_norm = norm(v1);

v2_norm = norm(v2);

 

beta1 = acos((L4^2 + v2_norm^2 - L5^2) / (2 * L4 * v2_norm));

beta2 = acos((v1_norm^2 + v2_norm^2 - L6^2) / (2 * v1_norm * v2_norm));

beta3 = acos((v1_norm^2 + L1^2 - L2^2) / (2 * v1_norm * L1));

 

beta = (beta1 + beta2 + beta3);

th4 = beta;

end