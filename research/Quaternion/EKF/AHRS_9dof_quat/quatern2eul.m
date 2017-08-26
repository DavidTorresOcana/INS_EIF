function [ eul ] = quatern2eul( q )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
eul(1) = atan2(2*q(3)*q(4)+2*q(1)*q(2),2*q(1)^2+2*q(4)^2-1 ); % Balance phi
eul(2) = -asin( 2*q(2)*q(4)-2*q(1)*q(3) ); % Angulo asiento theta
eul(3) = atan2( 2*q(2)*q(3)+2*q(1)*q(4),2*q(1)^2+2*q(2)^2-1 ); % Angulo de guiñada psi



end

