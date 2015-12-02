
function [lambda, delta] = GaussSeidel(W,dfree, maxIteration)
    
    num=size(W,1);
    lambda=zeros(num,1);
    delta = zeros(num,1);
    // ecrire l'algorithme
    
    
    
endfunction


W=[10 1 2; 1 5 4; 2 4 6];
dfree=[-3 ; -2; -0.1];
[lambda, delta] = GaussSeidel(W,dfree, 10000);

// visualiser les valeurs de delta et lambda dans la console
disp ([lambda, delta])
