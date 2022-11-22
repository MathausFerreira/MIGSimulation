%==========================================================================
% This function implements the controller
%==========================================================================

function Position_Controller_Dif(stp)
global Ctrl Sim SP Sat SLC WP;

Kp_d = 1/11;
Kp_alpha = 345/823;
Kp_beta = -1/11;

ErrPXYZ    = (SP.XYZ(:,WP) - Sim.Current_X_Y_psi);
delta_D    = sqrt(ErrPXYZ(1)^2 + ErrPXYZ(2)^2);
psi        = atan2(ErrPXYZ(2),ErrPXYZ(1));
alpha      = psi - Sim.Current_X_Y_psi(3);

alpha = mod(alpha,2*pi);
beta  = (SP.XYZ(3,WP) - psi);

% ----------- Corrige os angulos para que fiquem entre -180 e 180
alpha = Corrigir(alpha);
beta = Corrigir(beta);

if~(alpha >-90 && alpha < 90)
    delta_D = -delta_D;
    alpha = alpha + 180;
    beta  = beta + 180;
    % ----------- Corrige os angulos para que fiquem entre -180 e 180
    alpha = Corrigir(alpha);
    beta = Corrigir(beta);
end

V(1) = Kp_d * delta_D;
V(2) = 0;
V(3) = Kp_alpha*alpha + Kp_beta*beta;
% ErrPXYZ(3) = SP.XYZ(3,WP) - abs(Sim.Current_X_Y_psi(3))*sign(Sim.Current_X_Y_psi(3));
%
% ErrIXYZ  = Ctrl.ErrIXYZprev + ((Sim.Ts*SLC.Freq)/2)*(ErrPXYZ+Ctrl.ErrPXYZprev);
% Posi_dot = Sim.Current_u_v_r;
%
% Ctrl.ErrPXYZprev = ErrPXYZ;
% Ctrl.ErrIXYZprev = ErrIXYZ;
%
% V = Ctrl.kpPosi * ErrPXYZ + Ctrl.kiPosi * ErrIXYZ + Ctrl.kdPosi * Posi_dot;
%
% % Rotação de Frame
% V = NED2BF(Sim.Current_X_Y_psi(3),V);

% Saturação
V(1) = Satura(V(1),Sat.MaxVelX,-Sat.MaxVelX);
V(2) = 0*Satura(V(2),Sat.MaxVelY,-Sat.MaxVelY);
V(3) = Satura(V(3),Sat.MaxVelAng,-Sat.MaxVelAng);

Sim.Vel(:,stp) = V;

end