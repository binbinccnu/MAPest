function [shoes] = transformShoesWrenches (shoes, subjectParamsFromData)
% TRANSFORMSHOESWRENCHES transforms external wrenches coming 
% from the ftShoes into human frames. 
%
% Inputs:
% - shoes                 : right and left data;
% - subjectParamsFromData : for getting the ankle heights, i.e the origin 
%                           position of the reference frame of both feet
%                           wrt their projection on each shoe.
% Outputs:
% - shoes       : updated struct containing new fields ...... 
%
% Shoes wrenches are estimated in their frames (origin and
% orientation) that are located at a known position. 
% For the human estimation we need to get from this wrenches but: 
% - multiplied by -1 (as the wrench applied on the human is exactly the 
%   opposite of the one excerted on each shoe)
% - expressed in the frame of the human link in contact.
%
% This function computes the wrenches that the each shoe exerts on the link 
% in contact.  See the sketch footInShoe.pdf for the reference frames 
% locations.

%% Build the transformations
gravityZero = iDynTree.Vector3();
gravityZero.zero();

% LEFT---------------------------------------------------------------------
leftHeel_T_leftFtShoeRot = iDynTree.Rotation();
leftHeel_T_leftFtShoeRot.fromMatlab([ 1.0,  0.0,  0.0; ...
                                      0.0,  1.0,  0.0; ...
                                      0.0,  0.0,  1.0]);
leftHeel_T_leftFtShoePos = iDynTree.Position();
leftHeel_T_leftFtShoePos.fromMatlab([0.037; 0 ; -0.029]); % fixed, from footInShoe.pdf 
leftFoot_T_leftHeelPos = iDynTree.Position();
leftFoot_T_leftHeelPos.fromMatlab(subjectParamsFromData.pLeftHeelFoot);
leftFoot_T_leftFtShoe = iDynTree.Transform(leftHeel_T_leftFtShoeRot,...
                        leftHeel_T_leftFtShoePos + leftFoot_T_leftHeelPos);

% RIGHT--------------------------------------------------------------------
rightHeel_T_rightFtShoeRot = iDynTree.Rotation();
rightHeel_T_rightFtShoeRot.fromMatlab([ 1.0,  0.0,  0.0; ...
                                        0.0,  1.0,  0.0; ...
                                        0.0,  0.0,  1.0]);
rightHeel_T_rightFtShoePos = iDynTree.Position();
rightHeel_T_rightFtShoePos.fromMatlab([0.037; 0 ; -0.029]); % fixed, from footInShoe.pdf 
rightFoot_T_rightHeelPos = iDynTree.Position();
rightFoot_T_rightHeelPos.fromMatlab(subjectParamsFromData.pRightHeelFoot);
rightFoot_T_rightFtShoe = iDynTree.Transform(rightHeel_T_rightFtShoeRot,...
                          rightHeel_T_rightFtShoePos + rightFoot_T_rightHeelPos);

%% Transform wrenches from shoes frames into human frames
% Only totalForce for the moment, but it could be applied to frontForce and
% rearForce as well!

leftShoeWrench(1:3,:) = shoes.Left.upsampled.totalForce.forces;
leftShoeWrench(4:6,:) = shoes.Left.upsampled.totalForce.moments;

rightShoeWrench(1:3,:) = shoes.Right.upsampled.totalForce.forces;
rightShoeWrench(4:6,:) = shoes.Right.upsampled.totalForce.moments;

shoes.Left.upsampled.totalForce.humanFootWrench = ...
      -1*(leftFoot_T_leftFtShoe.asAdjointTransformWrench().toMatlab()*leftShoeWrench);
shoes.Right.upsampled.totalForce.humanFootWrench = ...
      -1*(rightFoot_T_rightFtShoe.asAdjointTransformWrench().toMatlab()*rightShoeWrench);
end
