getd();


function [F,K] = computeSpring(P1,P2, Linit, k)
	// longueur actuelle
	L = norm( P2 - P1 )
	// normale actuelle
	n = (P2 - P1)*(1.0/L);
	// force du ressort
	forceIntensity = k*(L-Linit);

	// force sur les noeuds
	F = [n* forceIntensity; -n* forceIntensity];

	// tangent stiffness 
    K=zeros(4,4)
endfunction




function [lambda, delta] = GaussSeidel(W,dfree, maxIteration)
       num=size(W,1);
    lambda=zeros(num,1);
    delta = zeros(num,1);
    // ecrire l'algorithme
    
    
endfunction

	


// MAILLAGE //
[ noeuds , elements ] = MSHLoader('circle.msh');
numElements = size(elements,2);
numNoeuds = size(noeuds,2);     



[segments] = findSegments(elements)
numSegments = size(segments,2);


// parametres physiques
k=0;
m=1;
g=9.81;

// parametres temps
dt = 0.1;
T =  1.0;

// etat courant
// historique
F_t = zeros(2*numNoeuds,1);
A = zeros(2*numNoeuds,1);
V_t = zeros(2*numNoeuds,1);
X_t = zeros(2*numNoeuds,1);

for i=1:numNoeuds
     X_t([2*i-1 2*i]) = [noeuds(1,i) ; noeuds(2,i)+5];
end

// vitesse angulaire initiale !
w = 0.0;
for i=1:numNoeuds
     V_t([2*i-1])= w * noeuds(2,i);
     V_t([2*i])= -w * noeuds(1,i);
end


// longueurs au repos
L0=[];
for s=1:numSegments,
    i1= segments(1,s);
    i2= segments(2,s);
    L0(s) = norm( X_t([2*i1-1 2*i1]) - X_t([2*i2-1 2*i2]) )
end

// gif de sortie
S = [];
S(1) = 'GIF\anim';
t=100;
S(2) = string(t);
S(3) = '.gif';
k_t=0;

Kass = zeros(2*numNoeuds,2*numNoeuds) ;
Mass = zeros(2*numNoeuds,2*numNoeuds) ;
mpoint= m / numNoeuds;
for time=0:dt:T,
    // mise à zero des forces
    F_t = zeros(2*numNoeuds,1);
    
    L=[];
    for s=1:numSegments,
        i1= segments(1,s);
        i2= segments(2,s);
        P1 = X_t([2*i1-1 2*i1]);
        P2 = X_t([2*i2-1 2*i2]);
        Linit=L0(s);
        [F,K] = computeSpring(P1,P2, Linit, k)
	
        // ajout de la force dans le vecteur
        F_t([2*i1-1 2*i1]) = F_t([2*i1-1 2*i1])  - F([1 2]);
        F_t([2*i2-1 2*i2]) = F_t([2*i2-1 2*i2])  - F([3 4]);
        
        // ajout des éléments dans la matrice tangente
        Kass([2*i1-1 2*i1 2*i2-1 2*i2],[2*i1-1 2*i1 2*i2-1 2*i2]) = Kass([2*i1-1 2*i1 2*i2-1 2*i2],[2*i1-1 2*i1 2*i2-1 2*i2]) - K;
        
    end

    for i=1:numNoeuds,
        // mass
        Mass ([2*i-1 2*i], [2*i-1 2*i]) = Mass ([2*i-1 2*i], [2*i-1 2*i]) + [1 0;0 1]* mpoint;
        // gravity
        F_t(2*i) = F_t(2*i) + mpoint*g;
    end

    // creation of the linear system to be solved at each time step
    A = Mass*(1/(dt*dt)) + Kass
    b = Mass*V_t*(1/dt) - F_t

    // solve free Motion
    dXfree = A\b;

    // Euler implicit (backward Euler)
    X_free = X_t + dXfree;
    V_free = dXfree*(1/dt);
   
    
    // Proximity detection
    col_index=[];
    num_col=0;
    prox=0.5
    for i=1:numNoeuds,
        if (X_t(2*i)-0.5)<-2.0,
            num_col = num_col+1;
            col_index(num_col) = i;
        end
    end
    
    // projection in the constraint space
    H=zeros(num_col,2*numNoeuds);
    dfree=zeros(num_col,1);
    for c=1:num_col,
        i = col_index(c);
        H(c,2*i) = 1; // direction of contact
        dfree(c)=X_free(2*i)+ 2.0;
    end
    W=H*inv(A)*H';
    
    [lambda, delta] = GaussSeidel(W,dfree, 1000);
    
    
    // constraint correction
    
    dX = inv(A)*H'*lambda;
    X_t = X_free+dX;
    V_t = (dXfree+dX)*1/dt;
    
    
     
        
            
        
    
        
    // déplacement du maillage
    noeuds_deplaces = noeuds;
    for i=1:numNoeuds,
      // deplacement selon x
      noeuds_deplaces(1,i) = X_t(2*i-1);
      // deplacement selon y
      noeuds_deplaces(2,i) = X_t(2*i);  
    end      
    
    clf;
    
    draw_mesh( noeuds_deplaces, elements)
     // draw obstacle
    x=[-10 10]
    y=[-2  -2]
    plot(x,y)
    
    k_t=k_t+1;
    S(2) = string(k_t+99);
    xs2bmp(0,strcat(S));
    
    
    
    xpause(100000);


end

  
  
  
  

  




