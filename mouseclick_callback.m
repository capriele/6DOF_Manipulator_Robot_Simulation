%% Funzione richiamata all'evento del click del mouse
function mouseclick_callback(gcbo,eventdata)
      % Le variabili *_new descrivono la posizione nuova assunta dai bracci,
      % quelle *_cur la posizione corrente
      % startPoint: posizione inziale del polso 
      % maxPoint: definisce le coordinate massime per un punto
      % vBall: vettore velocit? iniziale della pallina
      % pBall: posizione iniziale della pallina
      % rBall: raggio della pallina
      % lastTheta: le variabili di giunto assunte nell'ultima posizione 
      % workSpaceRadius: raggi dello spazio di lavoro
      % robotCenter: centro della base del robot
      global startPoint maxPoint link1_cur link2_cur link3_cur link4_cur link5_cur end_effector_cur ...
             base_new link1_new link2_new link3_new link4_new link5_new end_effector_new;
      global vBall pBall rBall lastTheta workSpaceRadius robotCenter;

      cP = get(gca,'Currentpoint'); %coordinate (x,y,z) del cursore del mouse
      % x = input('x = ');
      % y = input('y = ');
      % z = input('z = ');
      x = cP(1,1);
      y = cP(1,2);
      z = cP(1,3);
      vMax = 20; %velocit? massima
      signX = 1;
      signY = 1;
      if(x > 0)
          signX = -1;
      end;
      if(y > 0)
          signY = -1;
      end;
      Vx = signX*10;%rand()*vMax;%input('Vx = ');
      Vy = signY*10;%rand()*vMax;%input('Vy = ');
      Vz = 0;%rand()*vMax;%input('Vz = ');
      PresaMargin = 6; %tolleranza lungo tutte le direzioni nel considerare la pallina presa
      BallPresa = false; %variabile usata per affermare che la pallina ? stata presa oppure no.
      vBall = [Vx Vy Vz];
      pBall = [x y z];
      rBall = 5;
      deltaT = 0.01; %passo di campionamento
      altezzaPresa = rand()*10 + 35;%numero casuale compreso tra 35-45
      tempoPresa = getTimeFromHeight(pBall, vBall, altezzaPresa);%dopo x secondi riprendo la pallina
      nPassi = 0; %numero di passi fatti dalla pallina per raggiungere l'altezza desiderata
      for i=0:deltaT:tempoPresa
        if pBall(3) > altezzaPresa
            [pBallEnd v] = moveBall(pBall, vBall, i);
            pBall = pBallEnd;
            vBall = v;
            nPassi = nPassi +1;
        end;
      end;
      vBall = [Vx Vy Vz];
      pBall = [x y z];
      
      %Verifico la validit? delle coordinate del cursore del mouse
      if x > maxPoint
          x = maxPoint;
      elseif x < -maxPoint
          x = -maxPoint;
      end;
      if y > maxPoint
          y = maxPoint;
      elseif y < -maxPoint
          y = -maxPoint;
      end;
      if z > maxPoint
          z = maxPoint;
      elseif z < 0
          z = 0;
      end;
      
      % Imposto il titolo del grafico
      s = sprintf('(X, Y, Z) = (%3.3f, %3.3f, %3.3f)', x, y, z);
      thandle = get(gca,'Title');
      set(thandle,'String',s);
      
      %Spazio di lavoro
      [xws yws zws] = sphere;
      xws = xws(11:end,:);       % taglio la sfera
      yws = yws(11:end,:);       % taglio la sfera
      zws = zws(11:end,:);       % taglio la sfera
      
      [xs ys zs] = sphere;
      T = rettaPerDuePunti(startPoint, [pBallEnd(1) pBallEnd(2) pBallEnd(3)-rBall], nPassi); %risolvo la cinematica diretta
      J = [];
      tam = size(T, 3);
      i = 1;
      j = 1;
      while i <= tam || BallPresa == false
          if i > tam
              i = tam;
          end;
          T_cur = T(:,:,i);
          startPoint = [T_cur(1,4) T_cur(2,4) T_cur(3,4)];
          %Cinematica Inversa
          [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6] = inverseKinematics(T(:, :, i));
          J = cat(3, J, jacobianMatrix([theta_1 theta_2 theta_3 theta_4 theta_5 theta_6]));
          
          %Cinematica Diretta
          lastTheta = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6];
          [T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics([theta_1 theta_2 theta_3 theta_4 theta_5 theta_6]);
          link1_cur = transformation(link1_new, T1_0);
          link2_cur = transformation(link2_new, T2_0);
          link3_cur = transformation(link3_new, T3_0);
          link4_cur = transformation(link4_new, T4_0);
          link5_cur = transformation(link5_new, T5_0);
          end_effector_cur = transformation(end_effector_new, T6_0);

          %Animazione
          cla, hold on;
          patch(base_new); 
          patch(link1_cur); 
          patch(link2_cur); 
          patch(link3_cur); 
          patch(link4_cur); 
          patch(link5_cur); 
          patch(end_effector_cur);
          
          %Traiettoria che il manipolatore dovrebbe seguire (se nello spazio di lavoro)
          P1(i, 1) = T(1, 4, i); 
          P1(i, 2) = T(2, 4, i); 
          P1(i, 3) = T(3, 4, i);
          plot3(P1(:, 1), P1(:, 2), P1(:, 3), 'r');
          
          %Traiettoria eseguita realmente dal manipolatore
          P2(i, 1) = T6_0(1, 4);
          P2(i, 2) = T6_0(2, 4);
          P2(i, 3) = T6_0(3, 4);
          plot3(P2(:, 1), P2(:, 2), P2(:, 3), 'g');
          
          %Creo/Animo la pallina in modo analogo allo spazio di lavoro
          ball = surf(xs*rBall+pBall(1),ys*rBall+pBall(2),zs*rBall+pBall(3),'EdgeColor','none','LineStyle','none','FaceLighting','phong'); 
          set(ball,'FaceColor',[0 0 1],'FaceAlpha',1);
          [p v] = moveBall(pBall, vBall, deltaT*j);
          pBall = p;%aggiorno posizione pallina
          vBall = v;%aggiorno velocit? pallina
          
          %Traiettoria della pallina
          P3(j, 1) = pBall(1);
          P3(j, 2) = pBall(2);
          P3(j, 3) = pBall(3);
          plot3(P3(:, 1), P3(:, 2), P3(:, 3), 'b');
          
          %Verifico la validit? della presa in base alla tolleranza => se ?
          %vera fermo l'animazione
          %[abs(round(P2(i, 1)) - round(pBall(1))) abs(round(P2(i, 2)) - round(pBall(2))) abs(round(P2(i, 3)) - round(pBall(3)+2*rBall))]
          if abs(round(P2(i, 1)) - round(pBall(1))) <= PresaMargin && abs(round(P2(i, 2)) - round(pBall(2))) <= PresaMargin && abs(round(P2(i, 3)) - round(pBall(3)-rBall)) <= PresaMargin
              BallPresa = true;
          end;
          
          i = i + 1;
          j = j + 1;
          pause(deltaT);
      end;
      
      %Spazio di lavoro
      workSpace = surf(xws*workSpaceRadius(1)+robotCenter(1),yws*workSpaceRadius(1)+robotCenter(2),zws*workSpaceRadius(1)+robotCenter(3),'EdgeColor','none','LineStyle','none','FaceLighting','phong'); 
      set(workSpace,'FaceColor',[0 1 0],'FaceAlpha',0.2);
      workSpace = surf(xws*workSpaceRadius(2)+robotCenter(1),yws*workSpaceRadius(2)+robotCenter(2),zws*workSpaceRadius(2)+robotCenter(3),'EdgeColor','none','LineStyle','none','FaceLighting','phong'); 
      set(workSpace,'FaceColor',[1 1 0],'FaceAlpha',0.2);
end

%% Date posizione, velocit? correnti della pallina calcola, in base alle leggi di moto, i loro valori successivi (considerando gli attriti (viscoso e radente) e la costante elastica)
function [p v] = moveBall(p, v, t)
    global rBall;
    g = 9.81;
    %vLimit = 0.5;
    
    %Moto rettilineo
    x = p(1) + v(1)*t;
    y = p(2) + v(2)*t;
    vx = v(1) - 0.05*v(1);
    vy = v(2) - 0.05*v(2);
%     if abs(vx) < vLimit
%         vx = 0;
%     end;
%     if abs(vy) < vLimit
%         vy = 0;
%     end;
    
    %Moto accelerato
    z = p(3) - v(3)*t - g*t^2/2;
    vz = v(3) + g*t;
    vz = vz - 0.01*vz;%attrito viscoso
    if z <= rBall
        vz = -0.4*vz;
    end;
    if z < rBall
        z = rBall;
    end;
    p = [x y z];
    v = [vx vy vz];
end

%% Data una altezza di riferimento calcolo il tempo necessario a un corpo in caduta (con Vz = 0) per raggiungere il punto a quella quota 
function [t] = getTimeFromHeight(p, v, h)
    g = 9.81;
    
    %Moto accelerato
    %z = p(3) - v(3)*t - g*t^2/2;
    t = (-v(3) + sqrt(v(3)^2 - 2*(h-p(3))*g))/g;
end