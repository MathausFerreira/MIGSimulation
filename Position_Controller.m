%==========================================================================
% This function implements the controller
%==========================================================================
function Position_Controller(stp)
global Ctrl Sim SP Sat SLC WP;

ErrPXYZ    = (SP.XYZ(:,WP) - Sim.Current_X_Y_psi);
%ErrPXYZ(3) = SP.XYZ(3,WP) - abs(Sim.Current_X_Y_psi(3))*sign(Sim.Current_X_Y_psi(3));

ErrIXYZ  = Ctrl.ErrIXYZprev + ((Sim.Ts*SLC.Freq)/2)*(ErrPXYZ+Ctrl.ErrPXYZprev);
Posi_dot = Sim.Current_u_v_r;

Ctrl.ErrPXYZprev = ErrPXYZ;
Ctrl.ErrIXYZprev = ErrIXYZ;

V = Ctrl.kpPosi * ErrPXYZ + Ctrl.kiPosi * ErrIXYZ + Ctrl.kdPosi * Posi_dot;

% Rotação de Frame
V = NED2BF(Sim.Current_X_Y_psi(3),[V(1); V(2); 0]);

Vel = sqrt(V(1)^2+V(2)^2);
% Saturação 
V(1) = Satura(Vel,Sat.MaxVelX,-Sat.MaxVelX);
V(2) = 0*Satura(V(2),Sat.MaxVelY,-Sat.MaxVelY);
V(3) = Satura(V(3),Sat.MaxVelAng,-Sat.MaxVelAng);

Sim.Vel(1,stp) = V(1);

end