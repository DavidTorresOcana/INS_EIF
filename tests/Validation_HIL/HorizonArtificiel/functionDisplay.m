%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%	Auteur 	:   Elodie Roux (elodie.roux@ifrance.com)
%	Date		:   13 mai 2001
%	Projet   :   Simulateur
%	
%	BUT	   :	représenter affichage tableau de bord assiette...
%
%	DEMARCHE	:	Tentative de reproduction du display BFGoodrich Aerospace
%
%	REF		:	AviationWeek 2000
%
%	NOTA		:	Attention, je travaille en unités du système international SI
%					Soit entre autre en RADIAN, m/s, m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function		displaymodifie=functionDisplay(theta,phi,altitude,TAS)

global Display

% Traitement des données (entrées)
	%Ramène les angles entre + ou - pi
   	theta=functionRameneAnglepi(theta);
	phi=functionRameneAnglepi(phi);
   


%Display de la vitesse
	if(round(TAS*1.9438)/100<1)
		set(Display.VitesseValue,'String',['  ' num2str(round(TAS*1.9438))]);
	else
	   set(Display.VitesseValue,'String',{num2str(round(TAS*1.9438))});
	end
   
  
   
%Display de l'altitude
	h=round(altitude/0.3048);	%Conversion en ft
if(h/10000<1)
   if(h/1000<1)
      if(h/100<1)
         if(h/10<1)
            set(Display.AltitudeValue,'String',['        ' num2str(round(h))]);
         else
         	set(Display.AltitudeValue,'String',['      ' num2str(round(h))]);
	      end
	   else 
   	   set(Display.AltitudeValue,'String',['    ' num2str(round(h))]);
		end
	else
   	set(Display.AltitudeValue,'String',['  ' num2str(round(h))]);
   end
else
   set(Display.AltitudeValue,'String',[num2str(round(h))]);
end




%Matrice de rotation de phi (angle de gîte)
Mrot=[cos(phi)	-sin(phi);
	   sin(phi)	cos(phi)];
% Matrice de translation assiette après la rotation gîte
Mtr=Mrot*[0;Display.PasAssiette];


%Display du marker d'angle de gîte
Pos=Mrot*[Display.RefRosasseFleche];
set(Display.RosasseFleche,'Xdata',Pos(1,:))
set(Display.RosasseFleche,'Ydata',Pos(2,:))
clear Pos





%La terre et le ciel
if(phi<pi/2 & phi>-pi/2)==1
   set(Display.Terre,'Xdata',Display.Largeur/2*[1 1 -1 -1]);
   set(Display.Terre,'Ydata',[Display.Largeur/2*tan(phi)-(theta*180/pi)/41.5*Display.Hauteur/2 -Display.Hauteur/2 -Display.Hauteur/2 -Display.Largeur/2*tan(phi)-(theta*180/pi)/41.5*Display.Hauteur/2]);
elseif(phi==-pi/2)
   set(Display.Terre,'Xdata',Display.Largeur/2*[0 -1 -1 0]);
   set(Display.Terre,'Ydata',Display.Hauteur/2*[-1 -1 1 1]);
elseif(phi==pi/2)
   set(Display.Terre,'Xdata',Display.Largeur/2*[0 1 1 0]);
   set(Display.Terre,'Ydata',Display.Hauteur/2*[1 1 -1 -1]);
else	
	set(Display.Terre,'Xdata',Display.Largeur/2*[1 1 -1 -1]);
	set(Display.Terre,'Ydata',[Display.Largeur/2*tan(phi)-(theta*180/pi)/41.5*Display.Hauteur/2 Display.Hauteur/2 Display.Hauteur/2 -Display.Largeur/2*tan(phi)-(theta*180/pi)/41.5*Display.Hauteur/2]);
end


% set(Display.PitchBartext,'Color','w')

% set(Display.PitchBar,'LineWidth',2)

displaymodifie = Display;



   