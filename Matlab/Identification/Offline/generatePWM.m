function [PWM] = generatePWM(u,r,p)
%Distributes forward and rotational thrust to 4 inputs of hovercraft
Bl = p(3);
Br = p(3);
Fl = p(3);
Fr = p(3);
if(r>=0 && u>=0)
    Bl = Bl + p(1)*(u/2)*(u/2) + p(2)*(u/2);
    Br = Br + p(1)*(u/2)*(u/2) + p(2)*(u/2)+p(1)*(r/2)*(r/2)+p(2)*(r/2);
    Fl = Fl + p(1)*(r/2)*(r/2)+p(2)*(r/2);
    Fr = 1000;
elseif(r>=0 && u<0)
    Bl = 1000;
    Br = Br + p(1)*(r/2)*(r/2) + p(2)*(r/2);
    Fl = Fl + p(1)*(-u/2)*(-u/2) + p(2)*(-u/2)+p(1)*(r/2)*(r/2) + p(2)*(r/2);
    Fr = Fr + p(1)*(-u/2)*(-u/2) + p(2)*(-u/2);
elseif(r<0 && u>=0)
    Bl = Bl + p(1)*(u/2)*(u/2) + p(2)*(u/2)+p(1)*(-r/2)*(-r/2) + p(2)*(-r/2);
    Br = Br + p(1)*(u/2)*(u/2) + p(2)*(u/2);
    Fl = 1000;
    Fr = Fr + p(1)*(-r/2)*(-r/2) + p(2)*(-r/2);
elseif(r<0 && u<0)
    Bl = Bl + p(1)*(-r/2)*(-r/2) + p(2)*(-r/2);
    Br = 1000;
    Fl = Fl + p(1)*(-u/2)*(-u/2) + p(2)*(-u/2);
    Fr = Fr + p(1)*(-u/2)*(-u/2) + p(2)*(-u/2)+p(1)*(-r/2)*(-r/2) + p(2)*(-r/2);
end

PWM = [fix(Bl); fix(Br); fix(Fl); fix(Fr)];

end