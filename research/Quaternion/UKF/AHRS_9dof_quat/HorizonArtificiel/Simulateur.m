
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%	Auteur 	:   Elodie Roux (elodie.roux@ifrance.com)
%	Date		:   13 mai 2001
%	Projet   :   Simulateur
%	
%	BUT	   :	repr�senter affichage tableau de bord assiette...
%
%	DEMARCHE	:	Tentative de reproduction du display BFGoodrich Aerospace
%
%	REF		:	AviationWeek 2000
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all

%Initialisation du Display
SimulateurInitialisation


%Donn�es
	%assiette 
   theta=10*pi/180;
   
   %Angle de g�te
   phi=-15*pi/180;
   
   %Vitesse
   TAS=240;	%m/s
   
   %altitude
   altitude=33000*0.3048; 	%en m�tres

Display=functionDisplay(theta,phi,altitude,TAS)



