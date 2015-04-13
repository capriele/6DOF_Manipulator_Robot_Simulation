%% Funzione che dati due punti nello spazio (p0 e pf) e un numero di passi,
% nPassi, trova la retta passante per essi. 
% ho scomposto il problema in due parti: 
% 1. Calcolo della retta soluzione nel piano xy
% 2. Effettuo una traslazione verticale lungo l'asse z 
% Ritorna la matrice T contenente le cinematiche dirette per ogni punto
% calcolato

function T = rettaPerDuePunti(p0, pf, nPassi)
    A=[p0(1) p0(2)];
    B=[pf(1) pf(2)];
    eq1 = [A(1) 1; B(1) 1];
    eq2 = [A(2); B(2)];
    X=linsolve(eq1,eq2);
    a=X(1);
    b=X(2);
    
    X = p0(1):(pf(1)-p0(1))/(nPassi-1):pf(1); %vettore delle x
    [n m] = size(X);
    count = max([nPassi m]);
    %se le coordinate x di p0 e pf coincidono
    if(p0(1) == pf(1))
        X = p0(1)*ones(1,count);
    end;
    
    Y = a*X+b;
    %se le coordinate y di p0 e pf coincidono o Y non ? definito (non esistono a e b, inversa singolare) (es. quando ho retta orizzontale)
    if(p0(2) == pf(2) || isnan(Y(1)))
        Y = p0(2)*ones(1,count);
    end;
    
    Z = p0(3):(pf(3)-p0(3))/(nPassi-1):pf(3); %vettore delle z
    %se le coordinate z di p0 e pf coincidono
    if(p0(3) == pf(3))
        Z = p0(3)*ones(1,count);
    end;
    
    %creo per ogni punto della retta (in numero definito dal numero di
    %passi) la matrice T con orientamento fisso
    for i = 1:count
        T(:,:,i) = [1 0 0 X(i);
                    0 1 0 Y(i);
                    0 0 1 Z(i);
                    0 0 0 1];
    end;
end

