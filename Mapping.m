function ROM3_MAPPING()
    global slamObj u_in u_out hFig ROBOT_POSE history targetPos EKF
    global h_map_points h_traj h_ellipse h_robot h_target
    
    ROBOT_POSE = [0; 0; 0]; 
    
    % Inizializzazione matrici di covarianza EKF
    EKF.P = diag([0.01, 0.01, deg2rad(1)]);  
    EKF.Q = diag([0.08, 0.08, deg2rad(8)]);  
    EKF.R = diag([0.002, 0.002, deg2rad(1.5)]); 
    
    % Configurazione dell'oggetto lidarSLAM
    slamObj = lidarSLAM(67, 8);
    slamObj.LoopClosureThreshold = 250; 
    history.scans = {}; history.poses = []; targetPos = []; 
    
    try delete(udpport("datagram", "LocalPort", 5007)); catch; end
    u_in = udpport("datagram", "LocalPort", 5007, "Timeout", 5); 
    u_out = udpport("datagram");                                 
    
    hFig = figure('Name', 'ROM3 - Tesi 1: SLAM & Odometry Benchmark', 'Color', 'w', ...
                  'WindowKeyPressFcn', @gestoreEventi, 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.7]);
    
    hold on; grid on; axis equal;
    h_map_points = plot(NaN, NaN, 'm.', 'MarkerSize', 2);
    h_traj = plot(NaN, NaN, 'b-o', 'LineWidth', 1.2);
    h_ellipse = plot(NaN, NaN, 'r--', 'LineWidth', 1.2);
    h_robot = quiver(NaN, NaN, NaN, NaN, 'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
    h_target = plot(NaN, NaN, 'gx', 'MarkerSize', 15, 'LineWidth', 2.5);
    axis([-2 2 -2 2]);
    
    fprintf('TESI 1 ONLINE. Modulo Mappatura e Benchmark Odometrico.\n');
    fprintf('Clicca SULLA FINESTRA GRAFICA e primi [N] per catturare la posa zero.\n');
    fprintf('Premi [R] durante la mappatura per annullare l''ultima scansione e ricalcolare.\n');
    
    while ishandle(hFig), pause(0.1); end
end

function gestoreEventi(~, ev)
    global slamObj u_in u_out ROBOT_POSE history targetPos EKF hFig
    target_ip = "172.20.10.2"; 
    tasto = lower(ev.Key);
    
    % TASTO [N]: Ancoraggio della posa iniziale
    if strcmp(tasto, 'n')
        fprintf('\nAcquisizione scan di ancoraggio... ');
        scan = catturaScanConFlush(u_in, u_out, target_ip, 5); 
        if ~isempty(scan.Cartesian)
            addScan(slamObj, scan);
            [~, p] = scansAndPoses(slamObj);
            history.scans{end+1} = scan; history.poses(end+1,:) = p(end,:); 
            ROBOT_POSE = p(end,:)'; 
            fprintf('OK!\n'); mostraMappa();
        end
        
    % TASTO [S]: Esportazione strutturata della mappa binaria d'occupazione
    elseif strcmp(tasto, 's')
        fprintf('\n⚙️ Elaborazione ed esportazione della mappa standardizzata...');
        
        [scans, poses] = scansAndPoses(slamObj);
        raw_map = buildMap(scans, poses, 20, 5);
        
        matrice_ostacoli = raw_map.occupancyMatrix > 0.4;
        
        export_map = binaryOccupancyMap(matrice_ostacoli, raw_map.Resolution);
        export_map.GridLocationInWorld = raw_map.GridLocationInWorld;
        
        timestamp = datestr(now, 'yyyy-mm-dd_HH-MM');
        fileName = sprintf('SLAM_Log_%s.mat', timestamp);
        
        save(fileName, 'slamObj', 'history', 'ROBOT_POSE', 'export_map');
        fprintf('\nMappa standardizzata salvata con successo in: %s\n', fileName);
        
    % TASTO [G]: Navigazione e aggiornamento incrementale SLAM
    elseif strcmp(tasto, 'g')
        if isempty(history.poses), return; end
        [gx, gy] = ginput(1); targetPos = [gx, gy]; 
        
        dx = gx - ROBOT_POSE(1); dy = gy - ROBOT_POSE(2);
        dist = sqrt(dx^2 + dy^2);
        ang_rel = wrapToPi(atan2(dy, dx) - ROBOT_POSE(3));
        t_rot = abs(ang_rel) * 0.396; 
        
        posa_predetta = [gx; gy; wrapToPi(ROBOT_POSE(3) + ang_rel)];
        F = [1, 0, -dist*sin(posa_predetta(3)); 0, 1, dist*cos(posa_predetta(3)); 0, 0, 1];
        EKF.P = F * EKF.P * F' + EKF.Q;
        
        write(u_out, sprintf('M,%.2f,%.2f,%.2f', -0.4*sign(ang_rel), 0.4*sign(ang_rel), t_rot), "string", target_ip, 5005);
        pause(t_rot + 1.2); 
        write(u_out, sprintf('M,0.4,0.4,%.2f', dist * 2.591), "string", target_ip, 5005); 
        pause(dist * 2.591 + 1.5);
        
        scan = catturaScanConFlush(u_in, u_out, target_ip, 2);
        
        if ~isempty(scan.Cartesian)
            [isAccepted, ~] = addScan(slamObj, scan, posa_predetta');
            if isAccepted
                [~, p_all] = scansAndPoses(slamObj);
                z = p_all(end,:)'; needRebuild = false;
                
                % Gestione dell'inversione di orientamento dello scan cartesiano
                if abs(wrapToPi(z(3) - posa_predetta(3))) > (3*pi/4)
                    scan = lidarScan(-scan.Cartesian); 
                    z(1:2) = posa_predetta(1:2); z(3) = wrapToPi(z(3) + pi); needRebuild = true;                
                end
                
                % Innovazione e Gating probabilistico (Distanza di Mahalanobis)
                innov = z - posa_predetta; innov(3) = wrapToPi(innov(3));  
                S = EKF.P + EKF.R; d_mahalanobis = innov' * (S \ innov); 
                
                if d_mahalanobis < 16.27
                    K = EKF.P * (S \ eye(3)); ROBOT_POSE = posa_predetta + K * innov; 
                    ROBOT_POSE(3) = wrapToPi(ROBOT_POSE(3)); EKF.P = (eye(3) - K) * EKF.P;           
                    history.scans{end+1} = scan; history.poses(end+1,:) = ROBOT_POSE';
                    if needRebuild, rebuildSLAM(); else, optimizePoseGraph(slamObj.PoseGraph); end
                else, ROBOT_POSE = posa_predetta; end
            else, ROBOT_POSE = posa_predetta; end
        else, ROBOT_POSE = posa_predetta; end
        mostraMappa(); if ishandle(hFig), figure(hFig); end
        
    % TASTO [P]: Esperimento di calibrazione e benchmark odometrico (Quadrato)
    elseif strcmp(tasto, 'p')
        if isempty(history.poses), return; end
        
        fprintf('\n======================================================\n');
        fprintf('AVVIO ESPERIMENTO: "Quadrato Odometrico" (0.5m x 0.5m)\n');
        fprintf('Assicurati di avere 50cm liberi in avanti e a sinistra!\n');
        fprintf('Il LiDAR è DISATTIVATO. Il robot naviga alla cieca...\n');
        fprintf('======================================================\n');
        
        posa_iniziale_teorica = ROBOT_POSE;
        
        for lato = 1:4
            fprintf('\n   ▶ Lato %d/4: Avanzamento di 0.5m...', lato);
            dist = 0.5;
            t_avanti = dist * 2.591; 
            v = dist / t_avanti;
            
            write(u_out, sprintf('M,0.4,0.4,%.2f', t_avanti), "string", target_ip, 5005);
            pause(t_avanti + 0.8);
            
            theta = ROBOT_POSE(3);
            dx = v * t_avanti * cos(theta);
            dy = v * t_avanti * sin(theta);
            dtheta = 0;
            F = [1, 0, -v*t_avanti*sin(theta); 0, 1, v*t_avanti*cos(theta); 0, 0, 1];
            
            ROBOT_POSE = ROBOT_POSE + [dx; dy; dtheta];
            ROBOT_POSE(3) = wrapToPi(ROBOT_POSE(3));
            EKF.P = F * EKF.P * F' + EKF.Q;
            mostraMappa();
            
            fprintf('\n   ▶ Curva %d/4: Rotazione di 90 gradi...', lato);
            ang_rel = pi/2;
            t_rot = abs(ang_rel) * 0.396; 
            
            write(u_out, sprintf('M,%.2f,%.2f,%.2f', -0.4, 0.4, t_rot), "string", target_ip, 5005);
            pause(t_rot + 0.8);
            
            dtheta = ang_rel;
            F = eye(3); 
            
            ROBOT_POSE = ROBOT_POSE + [0; 0; dtheta];
            ROBOT_POSE(3) = wrapToPi(ROBOT_POSE(3));
            EKF.P = F * EKF.P * F' + EKF.Q;
            mostraMappa();
        end
        
        fprintf('\nPercorso teorico chiuso. Il robot calcola di essere a: [%.2f, %.2f]\n', ROBOT_POSE(1), ROBOT_POSE(2));
        fprintf('ACCENSIONE LASER E SNAP FINALE (Correzione SLAM)...\n');
        write(u_out, sprintf('M,0.0,0.0,0.1'), "string", target_ip, 5005); 
        pause(1.0); 
        
        % Acquisizione finale ad alta densita' per stima quantitativa del drift
        scan_finale = catturaScanConFlush(u_in, u_out, target_ip, 5); 
        if ~isempty(scan_finale.Cartesian)
            [isAccepted, ~] = addScan(slamObj, scan_finale, ROBOT_POSE');
            if isAccepted
                optimizePoseGraph(slamObj.PoseGraph);
                [~, p_all] = scansAndPoses(slamObj);
                posa_reale = p_all(end,:)';
                drift_error = norm(posa_iniziale_teorica(1:2) - posa_reale(1:2));
                
                fprintf('\n======================================================\n');
                fprintf('RISULTATI ESPERIMENTO DEAD-RECKONING:\n');
                fprintf(' - Posa Iniziale Reale:      [%.2f, %.2f]\n', posa_iniziale_teorica(1), posa_iniziale_teorica(2));
                fprintf(' - Posa Teorica (Odometria): [%.2f, %.2f]\n', ROBOT_POSE(1), ROBOT_POSE(2));
                fprintf(' - Posa Corretta (SLAM):     [%.2f, %.2f]\n', posa_reale(1), posa_reale(2));
                fprintf(' - DERIVA FISICA TOTALE:     %.3f metri!\n', drift_error);
                fprintf('======================================================\n');
                
                ROBOT_POSE = posa_reale;
                EKF.P = diag([0.01, 0.01, deg2rad(1)]); 
            else
                fprintf('\nDRIFT TROPPO ELEVATO! Lo SLAM non riconosce la stanza.\n');
            end
        end
        mostraMappa();
    end
end

function rebuildSLAM()
    global slamObj history
    slamObj = lidarSLAM(67, 8); slamObj.LoopClosureThreshold = 250;
    for i = 1:length(history.scans), addScan(slamObj, history.scans{i}, history.poses(i,:)); end
    optimizePoseGraph(slamObj.PoseGraph);
end

function mostraMappa()
    global slamObj ROBOT_POSE EKF h_map_points h_traj h_ellipse h_robot h_target targetPos
    try
        if slamObj.PoseGraph.NumNodes > 0
            [scans, poses] = scansAndPoses(slamObj);
            all_x = []; all_y = [];
            for i = 1:numel(scans)
                p = poses(i,:); p_loc = scans{i}.Cartesian;
                rotM = [cos(p(3)) -sin(p(3)); sin(p(3)) cos(p(3))];
                p_glob = (rotM * p_loc')' + [p(1), p(2)];
                all_x = [all_x; p_glob(:,1)]; all_y = [all_y; p_glob(:,2)];
            end
            set(h_map_points, 'XData', all_x, 'YData', all_y);
            set(h_traj, 'XData', poses(:,1), 'YData', poses(:,2));
            
            % Ellisse di incertezza associata alla covarianza EKF
            [V, D] = eig(EKF.P(1:2, 1:2));
            t = linspace(0, 2*pi, 50); p_ell = [cos(t); sin(t)];
            xy = 2 * V * sqrt(D) * p_ell + ROBOT_POSE(1:2);
            set(h_ellipse, 'XData', xy(1,:), 'YData', xy(2,:));
            set(h_robot, 'XData', ROBOT_POSE(1), 'YData', ROBOT_POSE(2), ...
                         'UData', cos(ROBOT_POSE(3))*0.3, 'VData', sin(ROBOT_POSE(3))*0.3);
            if ~isempty(all_x)
                axis([min(all_x)-0.5, max(all_x)+0.5, min(all_y)-0.5, max(all_y)+0.5]);
            end
        end
    catch; end
    if ~isempty(targetPos), set(h_target, 'XData', targetPos(1), 'YData', targetPos(2)); end
    drawnow limitrate;
end

function scan = catturaScanConFlush(u_rx, u_tx, ip, target_giri)
    while u_rx.NumDatagramsAvailable > 0, read(u_rx, u_rx.NumDatagramsAvailable, "string"); end
    
    cmd_str = sprintf("SCAN,%d", target_giri);
    write(u_tx, cmd_str, "string", ip, 5005);
    
    pts = []; t_start = tic;
    while toc(t_start) < 8 
        if u_rx.NumDatagramsAvailable > 0
            data = read(u_rx, u_rx.NumDatagramsAvailable, "string");
            for i = 1:length(data)
                str = data(i).Data;
                if strcmpi(str, "DONE"), t_start = -10; break; end 
                chunks = split(str, ';');
                for k = 1:length(chunks)
                    v = split(chunks(k), ',');
                    if length(v) == 2, pts = [pts; str2double(v(1)), str2double(v(2))]; end
                end
            end
        end
        if t_start < 0, break; end
    end
    
    % Filtraggio geometrico delle distanze radiali limite
    if size(pts,1) > 20 
        distanze = sqrt(pts(:,1).^2 + pts(:,2).^2);
        pts = pts(distanze <= 8.0, :);
        
        fprintf(' [LiDAR: %d pt Raw] ', size(pts,1));
        scan = lidarScan(pts); 
    else
        scan = lidarScan([]); 
    end
end