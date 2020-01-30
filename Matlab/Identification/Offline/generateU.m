function [U] = generateU(u,r)
%Distributes forward and rotational thrust to 4 inputs of hovercraft
Bl = 0;
Br = 0;
Fl = 0;
Fr = 0;
if(u>=0)
    Bl = u/2;
    Br = u/2;
elseif(u<0)
    Fl = -u/2;
    Fr = -u/2;
end
if(r>=0)
    Br = Br + r/2;
    Fl = Fl + r/2;
elseif(r<0)
    Bl = Bl - r/2;
    Fr = Fr - r/2;
end

U = [Bl; Br; Fl; Fr];

end