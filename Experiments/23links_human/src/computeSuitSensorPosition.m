function [suit] = computeSuitSensorPosition(suit)
% COMPUTESUITSENSORPOSITION computes the position of the sensors in the
% suite wrt the link frame. It returns its value in a new field of the same
% suit stucture. Notation: G = global, S = sensor; L = link.

len = suit.properties.lenData;

for sIdx = 1: suit.properties.nrOfSensors
    sensor = suit.sensors{sIdx};
    [link, ~] = linksFromName(suit.links, sensor.attachedLink);
    A = zeros(3*len,3);
    b = zeros(3*len,1);
    Rbig = [];
    SigmaBig = [];
    SigmaSens = cell(len,1);
    eigen = zeros(3,len);
    eigenMax = zeros(1,len);
    SigmaDet = zeros(1,len);
    Sigma_x_inv = eye(3); %1/0.0625
    Sigma_A_inv = 1/0.001111 * eye(3); %from datasheet
    Sigma_xgivena_inv = Sigma_x_inv;
    mu_xgivena = zeros(3,1);
    norm_Sigma_xgivena_inv = zeros(1,len);

    for i = 1 : len
        S1 = skewMatrix(link.meas.angularAcceleration(:,i));
        S2 = skewMatrix(link.meas.angularVelocity(:,i));
        quaternion = iDynTree.Vector4();
        quaternion.fromMatlab(link.meas.orientation(:,i));
        G_R_L = iDynTree.Rotation();
        G_R_L.fromQuaternion(quaternion);
        G_R_L = G_R_L.toMatlab();
        A(3*i-2:3*i,:) = (S1 + S2*S2) * G_R_L;

        quaternion = iDynTree.Vector4();
        quaternion.fromMatlab(sensor.meas.sensorOrientation(:,i));
        G_R_S = iDynTree.Rotation();
        G_R_S.fromQuaternion(quaternion);
        G_R_S = G_R_S.toMatlab();
        G_acc_S = G_R_S * sensor.meas.sensorAcceleration(:,i);

        G_acc_L = link.meas.acceleration(:,i);

        b(3*i-2:3*i) = G_acc_S - G_acc_L + [0;0;-9.81];
        
        Sigma_xgivena_inv = Sigma_xgivena_inv + ...
            A(3*i-2:3*i,:)' * Sigma_A_inv * A(3*i-2:3*i,:);
        mu_xgivena = mu_xgivena + ...
            Sigma_xgivena_inv\(A(3*i-2:3*i,:)' *  Sigma_A_inv * b(3*i-2:3*i));
        
        norm_Sigma_xgivena_inv(i) = norm(inv(Sigma_xgivena_inv));

        Rbig = blkdiag(Rbig, G_R_S);
        SigmaBig = blkdiag(SigmaBig,  0.001111 * eye(3));
        Abig = A(1:3*i,:);
        SigmaSens{i} = pinv(Abig) * Rbig * SigmaBig * Rbig' * (pinv(Abig)');
        eigen(:,i) = eig(SigmaSens{i});
        eigenMax(1,len) = max(eigen(:,len));
        SigmaDet(i) = det(SigmaSens{i});
        SigmaTotalNorm(i) = norm(SigmaSens{i});
        % norm-2 for a symmetric matrix (see MatrixCookbook 3.1.5)
        SigmaTotalNorm2(i) = max(abs(eig(SigmaSens{i}))); 

        % compute S_R_L = S_R_G x G_R_L
        S_R_L = G_R_S' * G_R_L;
        L_R_S = S_R_L' ;
        rot = iDynTree.Rotation();
        rot.fromMatlab(L_R_S);
        L_RPY_S(i,:) = rot.asRPY.toMatlab(); %RPY in rad
    end
    % matrix system
    B_pos_SL = A\b;
    
    Sigma_xgivena = inv(Sigma_xgivena_inv);

    sensor.origin = B_pos_SL;
    suit.sensors{sIdx}.position = sensor.origin;
    suit.sensors{sIdx}.RPY = mean(L_RPY_S);

end
end

function [ S ] = skewMatrix(x)
%SKEWMATRIX computes the skew matrix given a vector x 3x1

S = [  0   -x(3)   x(2);
      x(3)   0    -x(1);
     -x(2)  x(1)    0  ];
end
