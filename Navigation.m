function ROM3_Navigator()
    global u_in u_out hFig ROBOT_POSE binMap_clean GLOBAL_WALLS
    global h_map_points h_traj h_ellipse h_robot h_target h_actual_traj actual_path_history
    global PLANNED_PATH ICP_LOG h_safety_cone EKF
    ICP_LOG      = [];
    PLANNED_PATH = [];
    
    % Inizializzazione matrici di covarianza EKF
    EKF.P = diag([0.01, 0.01, deg2rad(1)]);
    EKF.Q = diag([0.08, 0.08, deg2rad(8)]);
    EKF.R = diag([0.002, 0.002, deg2rad(1.5)]);
    
    [fileName, pathName] = uigetfile('*.mat', 'Seleziona Mappa Pre-Registrata');
    if isequal(fileName, 0), return; end
    data_struct = load(fullfile(pathName, fileName));
    if ~isfield(data_struct, 'ROBOT_POSE')
        error('File di log non valido o corrotto.');
    end
    ROBOT_POSE = data_struct.ROBOT_POSE(:);
    
    if isfield(data_struct, 'export_map')
        binMap_clean = data_struct.export_map;
    elseif isfield(data_struct, 'slamObj')
        [scans, poses] = scansAndPoses(data_struct.slamObj);
        map_raw = buildMap(scans, poses, 20, 5);
        binMap_clean = binaryOccupancyMap(map_raw.occupancyMatrix > 0.4, map_raw.Resolution);
        binMap_clean.GridLocationInWorld = map_raw.GridLocationInWorld;
    else
        error('Nessuna mappa trouvata nel file.');
    end
    
    % Estrazione e de-duplicazione dei punti geometrici della mappa statica
    [r, c]    = find(occupancyMatrix(binMap_clean) > 0.4);
    wall_pts  = grid2world(binMap_clean, [r, c]);
    grid_size = 0.05;
    x_idx     = round(wall_pts(:,1) / grid_size);
    y_idx     = round(wall_pts(:,2) / grid_size);
    [~, unique_idx] = unique([x_idx, y_idx], 'rows');
    GLOBAL_WALLS    = wall_pts(unique_idx, :);
    fprintf('Mappa Globale Statica caricata. (%d punti muro estratti).\n', size(GLOBAL_WALLS, 1));
    
    actual_path_history = [];
    try delete(udpport("datagram", "LocalPort", 5007)); catch; end
    u_in  = udpport("datagram", "LocalPort", 5007, "Timeout", 5);
    u_out = udpport("datagram");
    
    hFig = figure('Name', 'ROM3 - Navigation (Straight Move)', 'Color', 'w', ...
                  'WindowKeyPressFcn', @gestoreEventi, ...
                  'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);
    hold on; grid on; axis equal;
    title('WORLD VIEW (Tracking Ibrido - Straight Move)');
    show(binMap_clean);
    
    h_map_points  = plot(NaN, NaN, 'm.', 'MarkerSize', 2);
    h_actual_traj = plot(NaN, NaN, 'b-',  'LineWidth', 2);
    h_traj        = plot(NaN, NaN, 'r-o', 'LineWidth', 2.5, 'MarkerSize', 6);
    h_safety_cone = fill(NaN, NaN, 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'g', ...
                         'LineWidth', 1.5, 'DisplayName', 'Cono Sicurezza (±10°)');
    h_ellipse     = plot(NaN, NaN, 'r:', 'LineWidth', 1.2);
    h_robot       = quiver(ROBOT_POSE(1), ROBOT_POSE(2), ...
                           cos(ROBOT_POSE(3))*0.3, sin(ROBOT_POSE(3))*0.3, ...
                           'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
    h_target      = plot(NaN, NaN, 'gx', 'MarkerSize', 15, 'LineWidth', 2.5);
    
    fprintf('Premi [P] per navigare verso un punto.\n');
    fprintf('Premi [N] per scansione di localizzazione manuale.\n');
    fprintf('Premi [Q] per terminare ed esportare il report.\n');
    while ishandle(hFig), pause(0.1); end
end

function gestoreEventi(~, ev)
    global u_in u_out ROBOT_POSE hFig h_safety_cone EKF
    global h_traj h_target h_actual_traj actual_path_history binMap_clean
    global PLANNED_PATH ICP_LOG
    
    target_ip = "172.20.10.2";
    tasto     = lower(ev.Key);
    
    % TASTO [Q]: ESPORTA REPORT DI SESSIONE
    if strcmp(tasto, 'q')
        fprintf('\nGENERAZIONE REPORT FINE CORSA...\n');
        if isempty(actual_path_history), return; end
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        save(sprintf('Nav_Session_%s.mat', timestamp), ...
             'actual_path_history', 'PLANNED_PATH', 'ICP_LOG');
        if ~isempty(ICP_LOG)
            writetable(array2table(ICP_LOG, ...
                'VariableNames', {'Correzione_Traslazione_m', 'Correzione_Rotazione_deg'}), ...
                sprintf('ICP_Precision_Log_%s.csv', timestamp));
        end
        fprintf('File salvati!\n');
        
    % TASTO [N]: LOCALIZZAZIONE MANUALE ICP-EKF
    elseif strcmp(tasto, 'n')
        scan = catturaScanConFlush(u_in, u_out, target_ip);
        if isa(scan, 'lidarScan')
            [z_meas, icp_ok] = localizzaConMatematicaPura(scan, ROBOT_POSE);
            if icp_ok, aggiornaPosaIbrida(z_meas); end
        end
        aggiornaGrafica(scan);
        
    % TASTO [P]: PIANIFICAZIONE A* E NAVIGAZIONE STOP & GO
    elseif strcmp(tasto, 'p')
        fprintf('\nClicca la DESTINAZIONE FINALE sulla mappa...');
        [gx, gy]  = ginput(1);
        goal_pt   = [gx, gy];
        
        cSpace = copy(binMap_clean);
        inflate(cSpace, 0.20);
        if checkOccupancy(cSpace, goal_pt) == 1
            fprintf('Destinazione non valida (occupata).\n'); return;
        end
        
        % Area di clearing locale intorno alla posa iniziale del robot
        r_clear   = 0.25;
        [cX, cY]  = meshgrid(ROBOT_POSE(1)-r_clear : 0.05 : ROBOT_POSE(1)+r_clear, ...
                             ROBOT_POSE(2)-r_clear : 0.05 : ROBOT_POSE(2)+r_clear);
        clear_pts = [cX(:), cY(:)];
        valid_idx = clear_pts(:,1) >= cSpace.XWorldLimits(1) & ...
                    clear_pts(:,1) <= cSpace.XWorldLimits(2) & ...
                    clear_pts(:,2) >= cSpace.YWorldLimits(1) & ...
                    clear_pts(:,2) <= cSpace.YWorldLimits(2);
        setOccupancy(cSpace, clear_pts(valid_idx,:), 0);
        setOccupancy(cSpace, goal_pt, 0);
        
        planner  = plannerAStarGrid(cSpace);
        path_idx = plan(planner, ...
                        world2grid(cSpace, ROBOT_POSE(1:2)'), ...
                        world2grid(cSpace, goal_pt));
        if isempty(path_idx), fprintf('Strada bloccata.\n'); return; end
        
        % Algoritmo di smoothing Line-of-Sight (LOS)
        path_xy         = grid2world(cSpace, path_idx);
        simplified_path = path_xy(1,:);
        curr_idx        = 1;
        while curr_idx < size(path_xy, 1)
            found_next = false;
            for test_idx = size(path_xy,1) : -1 : (curr_idx+1)
                distAB    = norm(path_xy(test_idx,:) - path_xy(curr_idx,:));
                alpha     = linspace(0, 1, max(2, ceil(distAB/0.01)))';
                line_pts  = path_xy(curr_idx,:) + alpha .* (path_xy(test_idx,:) - path_xy(curr_idx,:));
                if max(checkOccupancy(cSpace, line_pts)) == 0
                    simplified_path = [simplified_path; path_xy(test_idx,:)]; %#ok<AGROW>
                    curr_idx        = test_idx;
                    found_next      = true;
                    break;
                end
            end
            if ~found_next
                curr_idx        = curr_idx + 1;
                simplified_path = [simplified_path; path_xy(curr_idx,:)]; %#ok<AGROW>
            end
        end
        simplified_path(end,:) = goal_pt;
        PLANNED_PATH           = simplified_path;
        
        if ishandle(h_traj)
            set(h_traj, 'XData', simplified_path(:,1), 'YData', simplified_path(:,2));
        end
        if ishandle(h_target)
            set(h_target, 'XData', goal_pt(1), 'YData', goal_pt(2));
        end
        
        actual_path_history = ROBOT_POSE(1:2)';
        fprintf('\n======================================================\n');
        fprintf('AVVIO NAVIGAZIONE (Straight Move + EKF-ICP)\n');
        fprintf('======================================================\n');
        set(h_safety_cone, 'FaceColor', 'g', 'EdgeColor', 'g');
        
        wp_idx = 2;
        while wp_idx <= size(simplified_path, 1)
            if ~ishandle(hFig), break; end
            target_wp = simplified_path(wp_idx,:);
            dx = target_wp(1) - ROBOT_POSE(1);
            dy = target_wp(2) - ROBOT_POSE(2);
            
            if sqrt(dx^2 + dy^2) <= 0.15
                wp_idx = wp_idx + 1;
                continue;
            end
            emergenza_attivata = false;
            
            % FASE 1: ROTAZIONE SUL POSTO (Allineamento angolare)
            while true
                dx  = target_wp(1) - ROBOT_POSE(1);
                dy  = target_wp(2) - ROBOT_POSE(2);
                if sqrt(dx^2 + dy^2) <= 0.15, break; end
                
                ang_rel = wrapToPi(atan2(dy, dx) - ROBOT_POSE(3));
                if abs(ang_rel) <= deg2rad(3), break; end
                
                chunk_ang = sign(ang_rel) * min(abs(ang_rel), deg2rad(90));
                t_rot     = abs(chunk_ang) * 0.396;
                if t_rot > 0 && t_rot < 0.06, t_rot = 0.06; end
                
                fprintf('  Rotazione %+.0f deg (rimanente: %.0f deg)...\n', ...
                        rad2deg(chunk_ang), rad2deg(ang_rel));
                write(u_out, sprintf('M,%.2f,%.2f,%.2f', ...
                      -0.4*sign(chunk_ang), 0.4*sign(chunk_ang), t_rot), ...
                      "string", target_ip, 5005);
                  
                if attesaAttivaConSicurezza(t_rot, target_ip)
                    emergenza_attivata = true; break;
                end
                pause(0.4);
                
                % Aggiornamento stima a priori EKF per rotazione pura (F = identita')
                ROBOT_POSE(3) = wrapToPi(ROBOT_POSE(3) + chunk_ang);
                EKF.P = EKF.P + EKF.Q;   
                
                flushUDP(u_in);
                scan = catturaScanConFlush(u_in, u_out, target_ip);
                if isa(scan, 'lidarScan')
                    [z_meas, icp_ok] = localizzaConMatematicaPura(scan, ROBOT_POSE);
                    if icp_ok, aggiornaPosaIbrida(z_meas); end
                end
                aggiornaGrafica(scan);
            end
            
            if emergenza_attivata
                gestisciRecuperoOstacolo(target_ip);
                continue;
            end
            
            % FASE 2: AVANZAMENTO RETTILINEO PURO
            while true
                dx   = target_wp(1) - ROBOT_POSE(1);
                dy   = target_wp(2) - ROBOT_POSE(2);
                dist = sqrt(dx^2 + dy^2);
                if dist <= 0.15
                    wp_idx = wp_idx + 1;
                    break;
                end
                
                % Controllo deriva angolare cumulativa in corsa
                err_ang = wrapToPi(atan2(dy, dx) - ROBOT_POSE(3));
                if abs(err_ang) > deg2rad(20)
                    fprintf('  Deriva angolare (%.0f deg). Riallineamento...\n', rad2deg(err_ang));
                    break;
                end
                
                chunk_dist = min(dist, 0.30);
                t_avanti   = chunk_dist * 2.591;
                fprintf('  Avanzamento dritto: %.2f m (%.2f m al target)...\n', chunk_dist, dist);
                
                write(u_out, sprintf('M,0.40,0.40,%.2f', t_avanti), ...
                      "string", target_ip, 5005);
                  
                if attesaAttivaConSicurezza(t_avanti, target_ip)
                    emergenza_attivata = true;
                    actual_path_history = [actual_path_history; ROBOT_POSE(1:2)']; %#ok<AGROW>
                    break;
                end
                pause(0.4);
                
                % Passo di predizione cinematica EKF e calcolo Jacobiana F
                th_prev       = ROBOT_POSE(3);
                ROBOT_POSE(1) = ROBOT_POSE(1) + chunk_dist * cos(th_prev);
                ROBOT_POSE(2) = ROBOT_POSE(2) + chunk_dist * sin(th_prev);
                
                F     = [1, 0, -chunk_dist * sin(th_prev);
                         0, 1,  chunk_dist * cos(th_prev);
                         0, 0,  1];
                EKF.P = F * EKF.P * F' + EKF.Q;
                
                flushUDP(u_in);
                scan = catturaScanConFlush(u_in, u_out, target_ip);
                if isa(scan, 'lidarScan')
                    [z_meas, icp_ok] = localizzaConMatematicaPura(scan, ROBOT_POSE);
                    if icp_ok, aggiornaPosaIbrida(z_meas); end
                end
                
                actual_path_history = [actual_path_history; ROBOT_POSE(1:2)']; %#ok<AGROW>
                if ishandle(h_actual_traj)
                    set(h_actual_traj, 'XData', actual_path_history(:,1), ...
                                       'YData', actual_path_history(:,2));
                end
                aggiornaGrafica(scan);
            end
            
            if emergenza_attivata
                gestisciRecuperoOstacolo(target_ip);
                continue;
            end
        end
        fprintf('======================================================\n');
        fprintf('Target finale raggiunto.\n');
        write(u_out, 'M,0,0,0.1', "string", target_ip, 5005);
    end
end

function flushUDP(u_rx)
    pause(0.2);
    while u_rx.NumDatagramsAvailable > 0
        read(u_rx, u_rx.NumDatagramsAvailable, "string");
    end
end

function aggiornaPosaIbrida(z_meas)
    global ROBOT_POSE EKF
    
    % Hard Update orientamento angolare (misura ICP ad alta affidabilita)
    ROBOT_POSE(3)  = z_meas(3);
    EKF.P(3,3)     = deg2rad(0.5)^2;
    
    % Gating probabilistico XY tramite Distanza di Mahalanobis (Chi-Quadro, dof=2, conf=99%)
    innov_xy      = z_meas(1:2) - ROBOT_POSE(1:2);
    S_xy          = EKF.P(1:2,1:2) + EKF.R(1:2,1:2);
    d_mahalanobis = innov_xy' * (S_xy \ innov_xy);
    
    if d_mahalanobis < 9.21
        K_xy            = EKF.P(1:2,1:2) * (S_xy \ eye(2));
        ROBOT_POSE(1:2) = ROBOT_POSE(1:2) + K_xy * innov_xy;
        EKF.P(1:2,1:2)  = (eye(2) - K_xy) * EKF.P(1:2,1:2);
        fprintf('  [FUSIONE] Posizione accettata (D_M = %.2f)\n', d_mahalanobis);
    else
        fprintf('  [FUSIONE] Posizione SCARTATA - Aperture Problem (D_M = %.2f)\n', d_mahalanobis);
    end
end

function ostacolo_trovato = attesaAttivaConSicurezza(t_movimento, target_ip)
    global u_in u_out h_safety_cone
    ostacolo_trovato = false;
    t_start = tic;
    
    while toc(t_start) < t_movimento
        while u_in.NumDatagramsAvailable > 0
            read(u_in, u_in.NumDatagramsAvailable, "string");
        end
        write(u_out, "SCAN,1", "string", target_ip, 5005);
        
        pts = []; t_read = tic; done_ricevuto = false;
        while toc(t_read) < 0.6 && toc(t_start) < t_movimento
            if u_in.NumDatagramsAvailable > 0
                data = read(u_in, u_in.NumDatagramsAvailable, "string");
                for i = 1:numel(data)
                    str = strtrim(data(i).Data);
                    if strcmpi(str, "DONE")
                        done_ricevuto = true; break;
                    end
                    chunks = split(str, ';');
                    for k = 1:numel(chunks)
                        ck = strtrim(chunks(k));
                        if isempty(ck), continue; end
                        v = split(ck, ',');
                        if numel(v) == 2
                            x = str2double(v(1)); y = str2double(v(2));
                            if ~isnan(x) && ~isnan(y)
                                pts(end+1,:) = [x, y]; %#ok<AGROW>
                            end
                        end
                    end
                end
            end
            if done_ricevuto, break; end
        end
        
        % Monitoraggio Cono di Sicurezza frontale (arco polarizzato +/- 10 gradi)
        if size(pts,1) > 5
            angoli        = atan2(pts(:,2), pts(:,1));
            idx_frontali  = angoli >= -deg2rad(10) & angoli <= deg2rad(10);
            dist_frontali = hypot(pts(idx_frontali,1), pts(idx_frontali,2));
            if sum(idx_frontali) > 0 && any(dist_frontali > 0.05 & dist_frontali < 0.20)
                fprintf('\n[EMERGENZA] OSTACOLO! Freno di emergenza!\n');
                for k = 1:10
                    write(u_out, 'M,0,0,0.1', "string", target_ip, 5005);
                    pause(0.01);
                end
                set(h_safety_cone, 'FaceColor', 'r', 'EdgeColor', 'r');
                ostacolo_trovato = true;
                return;
            else
                set(h_safety_cone, 'FaceColor', 'g', 'EdgeColor', 'g');
            end
        end
        pause(0.05);
    end
end

function gestisciRecuperoOstacolo(target_ip)
    global u_in u_out h_safety_cone ROBOT_POSE
    fprintf('[RECUPERO] Percorso bloccato. Attendo rimozione ostacolo...\n');
    ostacolo_presente = true;
    t_recovery        = tic;
    MAX_RECOVERY_S    = 30;
    
    while ostacolo_presente
        if toc(t_recovery) > MAX_RECOVERY_S
            fprintf('[RECUPERO] Timeout (%ds). Uscita forzata.\n', MAX_RECOVERY_S);
            break;
        end
        while u_in.NumDatagramsAvailable > 0
            read(u_in, u_in.NumDatagramsAvailable, "string");
        end
        write(u_out, "SCAN,1", "string", target_ip, 5005);
        
        pts = []; t_read = tic; done_ricevuto = false;
        while toc(t_read) < 0.6
            if u_in.NumDatagramsAvailable > 0
                data = read(u_in, u_in.NumDatagramsAvailable, "string");
                for i = 1:numel(data)
                    str = strtrim(data(i).Data);
                    if strcmpi(str, "DONE")
                        done_ricevuto = true; break;
                    end
                    chunks = split(str, ';');
                    for k = 1:numel(chunks)
                        ck = strtrim(chunks(k));
                        if isempty(ck), continue; end
                        v = split(ck, ',');
                        if numel(v) == 2
                            x = str2double(v(1)); y = str2double(v(2));
                            if ~isnan(x) && ~isnan(y)
                                pts(end+1,:) = [x, y]; %#ok<AGROW>
                            end
                        end
                    end
                end
            end
            if done_ricevuto, break; end
        end
        
        if size(pts,1) > 5
            angoli        = atan2(pts(:,2), pts(:,1));
            idx_frontali  = angoli >= -deg2rad(10) & angoli <= deg2rad(10);
            dist_frontali = hypot(pts(idx_frontali,1), pts(idx_frontali,2));
            if any(dist_frontali > 0.05 & dist_frontali < 0.30)
                pause(0.5);
            else
                ostacolo_presente = false;
            end
        else
            pause(0.5);
        end
    end
    
    fprintf('[RECUPERO] Strada libera. Ricalcolo posizione...\n');
    set(h_safety_cone, 'FaceColor', 'g', 'EdgeColor', 'g');
    flushUDP(u_in);
    scan = catturaScanConFlush(u_in, u_out, target_ip);
    if isa(scan, 'lidarScan')
        [z_meas, icp_ok] = localizzaConMatematicaPura(scan, ROBOT_POSE);
        if icp_ok, aggiornaPosaIbrida(z_meas); end
    end
    aggiornaGrafica(scan);
end

function [estPose, success] = localizzaConMatematicaPura(scan, guessPose)
    global GLOBAL_WALLS ICP_LOG
    success = false;
    estPose = guessPose;
    try
        % ROI local-gating (raggio di 5 metri basato sulla posa stimata)
        dx_glob      = GLOBAL_WALLS(:,1) - guessPose(1);
        dy_glob      = GLOBAL_WALLS(:,2) - guessPose(2);
        dist_sq      = dx_glob.^2 + dy_glob.^2;
        local_pts    = GLOBAL_WALLS(dist_sq <= 25.0, :);
        if size(local_pts,1) < 20, return; end
        
        % Rototraslazione dei punti della mappa globale nel frame locale del veicolo
        th       = -guessPose(3);
        dx       = local_pts(:,1) - guessPose(1);
        dy       = local_pts(:,2) - guessPose(2);
        local_x  = dx .* cos(th) - dy .* sin(th);
        local_y  = dx .* sin(th) + dy .* cos(th);
        
        angles   = atan2(local_y, local_x);
        ranges   = hypot(local_x, local_y);
        ang_deg  = round(rad2deg(angles));
        [~, uid] = unique(ang_deg);
        refScan  = lidarScan(ranges(uid), angles(uid));
        
        % Algoritmo ICP geometrico di Scan Matching
        [relPose, stats] = matchScans(scan, refScan, 'MaxIterations', 50);
        dx_corr  = relPose(1);
        dy_corr  = relPose(2);
        dth_corr = relPose(3);
        
        % Filtro di validazione geometrico sulle correzioni limite dell'ICP
        if hypot(dx_corr, dy_corr) > 0.40 || abs(dth_corr) > deg2rad(30)
            return;
        end
        if isfield(stats, 'Inlier') && stats.Inlier < 0.45
            fprintf('  [ICP] Qualita'' bassa (inlier=%.2f). Scartato.\n', stats.Inlier);
            return;
        end
        
        % Proiezione della correzione relativa calcolata nel frame globale
        th0      = guessPose(3);
        estPose  = zeros(3,1);
        estPose(1) = guessPose(1) + dx_corr*cos(th0) - dy_corr*sin(th0);
        estPose(2) = guessPose(2) + dx_corr*sin(th0) + dy_corr*cos(th0);
        estPose(3) = wrapToPi(th0 + dth_corr);
        
        ICP_LOG  = [ICP_LOG; hypot(dx_corr, dy_corr), rad2deg(dth_corr)]; %#ok<AGROW>
        success  = true;
    catch ME
        fprintf('  [ICP] Errore: %s\n', ME.message);
    end
end

function aggiornaGrafica(scan)
    global ROBOT_POSE h_robot h_ellipse h_map_points h_safety_cone EKF
    if ~ishandle(h_robot), return; end
    set(h_robot, 'XData', ROBOT_POSE(1), 'YData', ROBOT_POSE(2), ...
                 'UData', cos(ROBOT_POSE(3))*0.3, 'VData', sin(ROBOT_POSE(3))*0.3);
             
    % Calcolo dell'ellisse di incertezza cartesiana tramite autovettori di P
    [V, D]  = eig(EKF.P(1:2,1:2));
    t_ell   = linspace(0, 2*pi, 50);
    xy      = 2 * V * sqrt(abs(D)) * [cos(t_ell); sin(t_ell)] + ROBOT_POSE(1:2);
    set(h_ellipse, 'XData', xy(1,:), 'YData', xy(2,:));
    
    if ishandle(h_safety_cone)
        cone_t = linspace(-deg2rad(10), deg2rad(10), 20) + ROBOT_POSE(3);
        cone_x = [ROBOT_POSE(1), ROBOT_POSE(1) + 0.20*cos(cone_t), ROBOT_POSE(1)];
        cone_y = [ROBOT_POSE(2), ROBOT_POSE(2) + 0.20*sin(cone_t), ROBOT_POSE(2)];
        set(h_safety_cone, 'XData', cone_x, 'YData', cone_y);
    end
    
    if isa(scan, 'lidarScan') && ~isempty(scan.Cartesian)
        rotM   = [cos(ROBOT_POSE(3)), -sin(ROBOT_POSE(3));
                  sin(ROBOT_POSE(3)),  cos(ROBOT_POSE(3))];
        p_glob = (rotM * scan.Cartesian')' + ROBOT_POSE(1:2)';
        set(h_map_points, 'XData', p_glob(:,1), 'YData', p_glob(:,2));
    end
    drawnow limitrate;
end

function scan = catturaScanConFlush(u_rx, u_tx, ip)
    while u_rx.NumDatagramsAvailable > 0
        read(u_rx, u_rx.NumDatagramsAvailable, "string");
    end
    write(u_tx, "SCAN,5", "string", ip, 5005);
    
    pts  = [];
    T    = 13.5;   % Timeout adattivo basato su clock di scansione (5 rotazioni complete)
    t0   = tic;
    done = false;
    
    while toc(t0) < T
        if u_rx.NumDatagramsAvailable > 0
            pkts = read(u_rx, u_rx.NumDatagramsAvailable, "string");
            for i = 1:numel(pkts)
                s = strtrim(pkts(i).Data);
                if strcmpi(s, "DONE")
                    done = true; break;
                end
                chunks = split(s, ';');
                for k = 1:numel(chunks)
                    ck = strtrim(chunks(k));
                    if isempty(ck), continue; end
                    v  = split(ck, ',');
                    if numel(v) ~= 2, continue; end
                    x  = str2double(v(1));
                    y  = str2double(v(2));
                    if ~isnan(x) && ~isnan(y)
                        pts(end+1,:) = [x, y]; %#ok<AGROW>
                    end
                end
            end
        end
        if done, break; end
        pause(0.005);
    end
    
    if ~done
        fprintf('  [WARN] DONE non ricevuto entro %.0fs.\n', T);
    end
    
    if size(pts,1) > 20
        % Rimozione riflessioni sul telaio del robot e saturazione di raggio [5cm, 5m]
        d    = hypot(pts(:,1), pts(:,2));
        pts  = pts(d >= 0.05 & d <= 5.0, :);
        
        % Downsampling spaziale su griglia di discretizzazione a 3cm
        x_idx = round(pts(:,1) / 0.03);
        y_idx = round(pts(:,2) / 0.03);
        [~, uid] = unique([x_idx, y_idx], 'rows');
        pts  = pts(uid, :);
        
        fprintf('  [LiDAR] %d pt filtrati.\n', size(pts,1));
        scan = lidarScan(pts);
    else
        fprintf('  [LiDAR] Punti insufficienti (%d). Scan scartato.\n', size(pts,1));
        scan = [];
    end
end