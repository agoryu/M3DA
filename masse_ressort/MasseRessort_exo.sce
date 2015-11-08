getd();

// MAILLAGE //
[ noeuds , elements ] = MSHLoader('rectangle1.msh');
numElements = size(elements,2);
numNoeuds = size(noeuds,2);     



[segments] = findSegments(elements);
numSegments = size(segments,2);


// parametres physiques
k=10000;
m=1;
g=9.81;

// parametres temps
dt = 0.005;
T =  0.1;



// image de sortie
S = [];
S(1) = 'BMP\anim';
t=50;
S(2) = string(t);
S(3) = '.bmp';
k_t=0;


// vecteur des positions:
X_t = zeros(2*numNoeuds,1);

for i=1:numNoeuds
     X_t([2*i-1 2*i]) = [noeuds(1,i) ; noeuds(2,i)];
end

oldVitesse = 0;
vitesse = 0;

Lz = zeros(numSegments);
for i=1:numSegments
    s1 = segments(1, i);
    s2 = segments(2, i);
    L0(i) = norm(X_t([2*s1-1 2*s1])-X_t([2*s2-1 2*s2]));
end

for time=0:dt:T,

    L = zeros(numSegments);
    
    // mouvement de translation uniforme
    for i=1:numNoeuds
        //calcule chute libre
        acceleration = [0;(-g * dt)];
        if(time-dt == 0)
            continue;
        end
        vitesse = oldVitesse + acceleration;
        X_t([2*i-1 2*i]) = X_t([2*i-1 2*i]) + vitesse;
    end
    
    oldVitesse = vitesse;
    
    for j=1:numSegments
        s1 = segments(1, j);
        s2 = segments(2, j);
        L(j) = norm(X_t([2*s1-1 2*s1]) - X_t([2*s2-1 2*s2]));
        f = k * (L(j) - L0(j)); 
        direction = X_t([2*s1-1 2*s1]) - X_t([2*s2-1 2*s2]);
        if(direction == 0) then
            continue
        end
        direction = direction / norm(direction);
        X_t([2*s1-1 2*s1]) = X_t([2*s1-1 2*s1]) + direction * f;
        X_t([2*s2-1 2*s2]) = X_t([2*s2-1 2*s2]) - direction * f;
    end
        
    // déplacement du maillage
    noeuds_deplaces = noeuds;
    for i=1:numNoeuds,
      // deplacement selon x
      noeuds_deplaces(1,i) = X_t(2*i-1);
      // deplacement selon y
      noeuds_deplaces(2,i) = X_t(2*i);  
    end      
    
    clf;
    scf(0);
    draw_mesh( noeuds_deplaces, elements)
    a=get("current_axes");//get the handle of the newly created axes
    a.data_bounds=[-1,-1;10,10];
        
        
    k_t=k_t+1;    
    S(2) = string(k_t+99);
    xs2bmp(0,strcat(S));

    
    xpause(100000);


end

  
  
  
  

  




