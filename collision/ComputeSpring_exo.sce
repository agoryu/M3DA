
function [F,K] = computeSpring(P1,P2, LO, k)
	// longueur actuelle
	L = norm( P2 - P1 )
	// normale actuelle
	n = (P2 - P1)*(1.0/L);
	// force du ressort
	forceIntensity = k*(L-L0);

	// force sur les noeuds
	F = [n* forceIntensity; -n* forceIntensity];

	// tangent stiffness 
    tgt = forceIntensity / L;
    kr = n*n'*(k-tgt) + [1 0;0 1] * tgt;
    K = [-kr kr; kr -kr]

endfunction



P1=[0; 0];
P2=[1;1];
L0=norm(P2-P1);
P1=[-0.1;0.2];

k = 1000;
[F,K] = computeSpring(P1,P2, L0, k);


epsilon=0.000001;
dX =[0.3 ; 1; 0; 1]*epsilon;
dF = K*dX

P1=P1+dX([1 2])
P2=P2+dX([3 4])

[F2,K] = computeSpring(P1,P2, L0, k);

dF2 = F2-F;

disp([dF dF2]/epsilon)
 
