function [circle] = drawCircle(posX, posY, posZ, distance)
x = posX;
y = posY;
z = posZ;
r = distance;

hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    figure(2);
    plot(xunit, yunit);
    title('Localization');
    xlabel('x-value');
    ylabel('y-value');
hold off

circle = [x, y, z, r];
end


