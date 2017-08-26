%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%	Auteur 	:   Elodie Roux (elodie.roux@ifrance.com)
%	Date		:   13 mai 2001
%	Projet   :   Simulateur
%	
%	BUT	   :	ramener la valeur d'un AngleRAD entre + ou - pi
%
%	DEMARCHE	:	compter le nombre de tour, 
%					les retrancher 
%					puis ramener � un cadran de + ou - pi
%					en ajoutant ou soustrayant 2pi (selon si >0 ou <0)
%
%	REF		:	mes m�ninges (si tent� que j'en aie)!
%
%	NOTA		: l'AngleRAD d'entr�e doit �tre exprim� en RADIAN !!!
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function   AngleRADpi=functionRameneAnglepi(AngleRAD);

Anglepi=AngleRAD;
% Si la mesure de phi comprend plusieurs tours.
% on se ram�ne � + ou - 2*pi
I=find(abs(AngleRAD/(2*pi))>1);
   tamp=AngleRAD(I)-fix(AngleRAD(I)/(2*pi))*2*pi;
   Anglepi(I)=tamp;

% On se ram�ne mainteant � + ou - pi
Ipi=find(AngleRAD>pi);
   Anglepi(Ipi)=-(2*pi-AngleRAD(Ipi));
Ipii=find(AngleRAD<=-pi);
   Anglepi(Ipii)=2*pi+AngleRAD(Ipii);

if(~isempty(find(AngleRAD>pi + AngleRAD<-pi))==1)
   errordlg({'Elodie a fait une erreur de raisonnement';'non conforme ce qui va entra�ner';'l''interruption de ce programme'})
end

AngleRADpi=Anglepi;