% =========================================================================
% TESI 1: MAPPATURA EKF-SLAM E BENCHMARK ODOMETRICO (VERSIONE ULTRA)
% =========================================================================
% Comandi da tastiera sulla finestra grafica:
%   [N] - Cattura posa zero di ancoraggio (5 giri ad alta densita')
%   [G] - Clicca un punto e naviga con aggiornamento EKF-SLAM + Mahalanobis
%   [R] - Ctrl+Z SLAM: annulla l'ultima scansione e ricalcola da posa sana
%   [P] - Esegue il Benchmark "Quadrato Odometrico" (0.5m x 0.5m)
%   [S] - Costruisce ed esporta la Binary Occupancy Map (.mat) per Navigation
% =========================================================================

function Mapping()
    global slamObj u_in u_out hFig ROBOT_POSE history targetPos EKF
    global h_map_points h_traj h_ellipse h_robot h_target

    ROBOT_POSE = [0; 0; 0];

    % Inizializzazione matrici di covarianza EKF
    EKF.P = diag([0.01, 0.01, deg2rad(1)^2]);
    EKF.Q = diag([0.04, 0.04, deg2rad(5)^2]);
    EKF.R = diag([0.002, 0.002, deg2rad(1.5)^2]);

    % Configurazione dell'oggetto lidarSLAM (risoluzione 67 celle/m, portata 8m)
    slamObj = lidarSLAM(67, 8);
    slamObj.LoopClosureThreshold = 250;
    slamObj.LoopClosureSearchRadius = 3.0;
    history.scans = {};
    history.poses = [];
    targetPos     = [];

    try delete(udpport("datagram", "LocalPort", 5007)); catch; end
    u_in  = udpport("datagram", "LocalPort", 5007, "Timeout", 5);
    u_out = udpport("datagram");

    hFig = figure('Name', 'ROM3 - Fase 1: EKF-SLAM & Odometry Benchmark (Ultra)', ...
                  'Color', 'w', 'WindowKeyPressFcn', @gestoreEventi, ...
                  'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.75]);

    hold on; grid on; axis equal;
    title('MAPPA SLAM IN TEMPO REALE ([N]=Ancora, [G]=Muovi, [R]=Annulla, [S]=Salva, [P]=Quadrato)');
    xlabel('X [m]'); ylabel('Y [m]');

    h_map_points = plot(NaN, NaN, 'm.', 'MarkerSize', 2.5, 'DisplayName', 'Punti Mappa LiDAR');
    h_traj       = plot(NaN, NaN, 'b-o', 'LineWidth', 1.4, 'MarkerSize', 5, 'DisplayName', 'Pose Graph SLAM');
    h_ellipse    = plot(NaN, NaN, 'r--', 'LineWidth', 1.3, 'DisplayName', 'Ellisse Incertezza EKF (2\sigma)');
    h_robot      = quiver(NaN, NaN, NaN, NaN, 'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5, 'DisplayName', 'Posa Robot');
    h_target     = plot(NaN, NaN, 'gx', 'MarkerSize', 14, 'LineWidth', 2.5, 'DisplayName', 'Target');
    axis([-2.5 2.5 -2.5 2.5]);

    fprintf('==================================================================\n');
    fprintf(' FASE 1 ONLINE: Modulo Mappatura EKF-SLAM e Benchmark Odometrico\n');
    fprintf('  [N] -> Cattura la posa zero di ancoraggio (5 giri)\n');
    fprintf('  [G] -> Clicca sulla mappa per muovere ed espandere lo SLAM\n');
    fprintf('  [R] -> Annulla l''ultima scansione (Ctrl+Z SLAM) e riallinea\n');
    fprintf('  [P] -> Avvia esperimento "Quadrato Odometrico" (0.5m x 0.5m)\n');
    fprintf('  [S] -> Esporta mappa finale (.mat) per la navigazione autonoma\n');
    fprintf('==================================================================\n');

    while ishandle(hFig)
        pause(0.1);
    end
end

function gestoreEventi(~, ev)
    global slamObj u_in u_out ROBOT_POSE history targetPos EKF hFig
    target_ip = "172.20.10.2";
    tasto = lower(ev.Key);

    % =====================================================================
    % TASTO [N]: ANCORAGGIO DELLA POSA INIZIALE
    % =====================================================================
    if strcmp(tasto, 'n')
        fprintf('\n[ANCORAGGIO] Acquisizione scan iniziale (5 giri)... ');
        scan = catturaScanConFlush(u_in, u_out, target_ip, 5);
        if ~isempty(scan.Cartesian)
            addScan(slamObj, scan);
            [~, p] = scansAndPoses(slamObj);
            history.scans{end+1}   = scan;
            history.poses(end+1,:) = p(end,:);
            ROBOT_POSE             = p(end,:)';
            fprintf('OK! (%d punti validati)\n', size(scan.Cartesian, 1));
            mostraMappa();
        else
            fprintf('ERRORE: Nessun dato LiDAR ricevuto.\n');
        end

    % =====================================================================
    % TASTO [S]: ESPORTAZIONE STRUTTURATA DELLA MAPPA BINARIA D'OCCUPAZIONE
    % =====================================================================
    elseif strcmp(tasto, 's')
        if isempty(history.scans)
            fprintf('\n[WARN] Nessuna scansione acquisita. Premi prima [N].\n');
            return;
        end
        fprintf('\n[EXPORT] Ottimizzazione grafo ed esportazione mappa standardizzata...');
        optimizePoseGraph(slamObj.PoseGraph);

        [scans, poses] = scansAndPoses(slamObj);
        raw_map = buildMap(scans, poses, 20, 5);

        % Muri (>0.4) e spazio ignoto (0.5) diventano ostacoli (1); spazio esplorato (<0.4) -> libero (0)
        matrice_ostacoli = raw_map.occupancyMatrix > 0.4;

        export_map = binaryOccupancyMap(matrice_ostacoli, raw_map.Resolution);
        export_map.GridLocationInWorld = raw_map.GridLocationInWorld;

        timestamp = datestr(now, 'yyyy-mm-dd_HH-MM');
        fileName  = sprintf('SLAM_Log_%s.mat', timestamp);

        save(fileName, 'slamObj', 'history', 'ROBOT_POSE', 'export_map');
        fprintf('\n[EXPORT OK] Mappa salvata con successo in: %s\n', fileName);

    % =====================================================================
    % TASTO [R]: RICALCOLO E CORREZIONE SIMMETRIA (CTRL+Z SLAM)
    % =====================================================================
    elseif strcmp(tasto, 'r')
        fprintf('\n[RICALCOLO SLAM] Annullamento ultima scansione e riallineamento...\n');
        write(u_out, 'M,0,0,0.1', "string", target_ip, 5005);
        pause(0.2);

        if slamObj.PoseGraph.NumNodes > 1
            history.scans(end)   = [];
            history.poses(end,:) = [];
            rebuildSLAM();
            fprintf('  - Rimossa l''ultima scansione dal Pose Graph.\n');
        else
            fprintf('  - Impossibile cancellare: e'' presente solo la posa zero!\n');
            return;
        end

        fprintf('  - Acquisizione nuova scansione ad alta densita'' (5 giri)...\n');
        scan = catturaScanConFlush(u_in, u_out, target_ip, 5);
        if ~isempty(scan.Cartesian)
            [~, p_sane] = scansAndPoses(slamObj);
            posa_sana   = p_sane(end, :)';
            R_sana      = [cos(posa_sana(3)), -sin(posa_sana(3)); sin(posa_sana(3)), cos(posa_sana(3))];
            rel_xy_r    = (R_sana' * (ROBOT_POSE(1:2) - posa_sana(1:2)))';
            rel_th_r    = wrapToPi(ROBOT_POSE(3) - posa_sana(3));

            [isAdded, loopClosed] = addScan(slamObj, scan, [rel_xy_r, rel_th_r]);
            if isAdded
                if loopClosed
                    fprintf('  - LOOP CLOSURE RILEVATO! Mappa ottimizzata.\n');
                end
                [~, p_nuove]           = scansAndPoses(slamObj);
                history.scans{end+1}   = scan;
                history.poses(end+1,:) = p_nuove(end,:);
                ROBOT_POSE             = p_nuove(end,:)';
                EKF.P                  = diag([0.01, 0.01, deg2rad(1)^2]);
                fprintf('  - Posa ricalcolata e agganciata: [%.2f, %.2f, %.1f deg]\n', ...
                        ROBOT_POSE(1), ROBOT_POSE(2), rad2deg(ROBOT_POSE(3)));
                mostraMappa();
            else
                fprintf('  - [WARN] Scan non agganciato. Riprova da posizione piu'' vicina.\n');
            end
        end

    % =====================================================================
    % TASTO [G]: NAVIGAZIONE E AGGIORNAMENTO INCREMENTALE SLAM
    % =====================================================================
    elseif strcmp(tasto, 'g')
        if isempty(history.poses), return; end
        fprintf('\nClicca il punto di destinazione sulla mappa...\n');
        [gx, gy]  = ginput(1);
        targetPos = [gx, gy];

        dx      = gx - ROBOT_POSE(1);
        dy      = gy - ROBOT_POSE(2);
        dist    = hypot(dx, dy);
        ang_rel = wrapToPi(atan2(dy, dx) - ROBOT_POSE(3));
        t_rot   = abs(ang_rel) * 0.396;
        t_trans = dist * 2.591;

        posa_predetta = [gx; gy; wrapToPi(ROBOT_POSE(3) + ang_rel)];
        F = [1, 0, -dist*sin(posa_predetta(3));
             0, 1,  dist*cos(posa_predetta(3));
             0, 0,  1];

        % Rumore di processo scalato con l'entita' dello spostamento
        scale_q = max(0.25, dist / 0.50);
        EKF.P   = F * EKF.P * F' + EKF.Q * scale_q;
        EKF.P   = 0.5 * (EKF.P + EKF.P');

        if t_rot > 0.04
            write(u_out, sprintf('M,%.2f,%.2f,%.2f', -0.4*sign(ang_rel), 0.4*sign(ang_rel), t_rot), ...
                  "string", target_ip, 5005);
            pause(t_rot + 0.25);
        end

        if t_trans > 0.04
            write(u_out, sprintf('M,0.40,0.40,%.2f', t_trans), "string", target_ip, 5005);
            pause(t_trans + 0.25);
        end

        % Scansione a 3 giri (veloce e densa grazie allo zero-wait su Raspberry)
        scan = catturaScanConFlush(u_in, u_out, target_ip, 3);

        if ~isempty(scan.Cartesian)
            % Calcolo della posa RELATIVA rispetto all'ultimo nodo dello SLAM
            [~, p_prev_all] = scansAndPoses(slamObj);
            p_prev          = p_prev_all(end,:)';
            R_prev          = [cos(p_prev(3)), -sin(p_prev(3)); sin(p_prev(3)), cos(p_prev(3))];
            rel_xy          = (R_prev' * (posa_predetta(1:2) - p_prev(1:2)))';
            rel_th          = wrapToPi(posa_predetta(3) - p_prev(3));

            [isAccepted, ~] = addScan(slamObj, scan, [rel_xy, rel_th]);
            if isAccepted
                [~, p_all] = scansAndPoses(slamObj);
                z          = p_all(end,:)';

                % Innovazione e Gating probabilistico (Mahalanobis 3 DoF, 99.9% -> 16.27)
                innov          = z - posa_predetta;
                innov(3)       = wrapToPi(innov(3));
                S              = EKF.P + EKF.R;
                d_mahalanobis  = innov' * (S \ innov);

                if d_mahalanobis < 16.27
                    K          = EKF.P / S;
                    ROBOT_POSE = posa_predetta + K * innov;
                    ROBOT_POSE(3) = wrapToPi(ROBOT_POSE(3));

                    % Aggiornamento covarianza in Forma di Joseph (stabile e simmetrica)
                    I3    = eye(3);
                    IK    = I3 - K;
                    EKF.P = IK * EKF.P * IK' + K * EKF.R * K';
                    EKF.P = 0.5 * (EKF.P + EKF.P');

                    history.scans{end+1}   = scan;
                    history.poses(end+1,:) = ROBOT_POSE';
                    optimizePoseGraph(slamObj.PoseGraph);
                    [~, p_opt] = scansAndPoses(slamObj);
                    ROBOT_POSE = p_opt(end,:)';
                    history.poses = p_opt;
                    fprintf(' [SLAM+EKF] Aggiornato (D_M=%.2f) -> [%.2f, %.2f, %.1f deg]\n', ...
                            d_mahalanobis, ROBOT_POSE(1), ROBOT_POSE(2), rad2deg(ROBOT_POSE(3)));
                else
                    ROBOT_POSE = posa_predetta;
                    fprintf(' [GATE] Misura scartata (D_M=%.2f). Mantenuta odometria.\n', d_mahalanobis);
                end
            else
                ROBOT_POSE = posa_predetta;
            end
        else
            ROBOT_POSE = posa_predetta;
        end
        mostraMappa();
        if ishandle(hFig), figure(hFig); end

    % =====================================================================
    % TASTO [P]: ESPERIMENTO BENCHMARK ODOMETRICO (QUADRATO 0.5m x 0.5m)
    % =====================================================================
    elseif strcmp(tasto, 'p')
        if isempty(history.poses), return; end

        fprintf('\n======================================================\n');
        fprintf('AVVIO ESPERIMENTO: "Quadrato Odometrico" (0.5m x 0.5m)\n');
        fprintf('Assicurati di avere 50cm liberi in avanti e a sinistra!\n');
        fprintf('Il LiDAR e'' DISATTIVATO. Il robot naviga alla cieca...\n');
        fprintf('======================================================\n');

        posa_iniziale_teorica = ROBOT_POSE;

        for lato = 1:4
            fprintf('\n  -> Lato %d/4: Avanzamento di 0.5m...', lato);
            dist     = 0.5;
            t_avanti = dist * 2.591;
            v        = dist / t_avanti;

            write(u_out, sprintf('M,0.40,0.40,%.2f', t_avanti), "string", target_ip, 5005);
            pause(t_avanti + 0.35);

            theta  = ROBOT_POSE(3);
            dx     = v * t_avanti * cos(theta);
            dy     = v * t_avanti * sin(theta);
            F      = [1, 0, -v*t_avanti*sin(theta); 0, 1, v*t_avanti*cos(theta); 0, 0, 1];

            ROBOT_POSE    = ROBOT_POSE + [dx; dy; 0];
            ROBOT_POSE(3) = wrapToPi(ROBOT_POSE(3));
            EKF.P         = F * EKF.P * F' + EKF.Q;
            EKF.P         = 0.5 * (EKF.P + EKF.P');
            mostraMappa();

            fprintf('\n  -> Curva %d/4: Rotazione di 90 gradi...', lato);
            ang_rel = pi/2;
            t_rot   = abs(ang_rel) * 0.396;

            write(u_out, sprintf('M,%.2f,%.2f,%.2f', -0.4, 0.4, t_rot), "string", target_ip, 5005);
            pause(t_rot + 0.35);

            ROBOT_POSE    = ROBOT_POSE + [0; 0; ang_rel];
            ROBOT_POSE(3) = wrapToPi(ROBOT_POSE(3));
            EKF.P         = EKF.P + EKF.Q;
            EKF.P         = 0.5 * (EKF.P + EKF.P');
            mostraMappa();
        end

        fprintf('\nPercorso teorico chiuso. Posa odometrica: [%.2f, %.2f]\n', ROBOT_POSE(1), ROBOT_POSE(2));
        fprintf('ACCENSIONE LASER E SNAP FINALE (Correzione SLAM)...\n');
        write(u_out, 'M,0.0,0.0,0.1', "string", target_ip, 5005);
        pause(0.4);

        scan_finale = catturaScanConFlush(u_in, u_out, target_ip, 5);
        if ~isempty(scan_finale.Cartesian)
            [~, p_prev_all] = scansAndPoses(slamObj);
            p_prev          = p_prev_all(end,:)';
            R_prev          = [cos(p_prev(3)), -sin(p_prev(3)); sin(p_prev(3)), cos(p_prev(3))];
            rel_xy_p        = (R_prev' * (ROBOT_POSE(1:2) - p_prev(1:2)))';
            rel_th_p        = wrapToPi(ROBOT_POSE(3) - p_prev(3));
            [isAccepted, ~] = addScan(slamObj, scan_finale, [rel_xy_p, rel_th_p]);
            if isAccepted
                optimizePoseGraph(slamObj.PoseGraph);
                [~, p_all]  = scansAndPoses(slamObj);
                posa_reale  = p_all(end,:)';
                drift_error = norm(posa_iniziale_teorica(1:2) - posa_reale(1:2));

                fprintf('\n======================================================\n');
                fprintf('RISULTATI ESPERIMENTO DEAD-RECKONING:\n');
                fprintf(' - Posa Iniziale Reale:      [%.2f, %.2f]\n', posa_iniziale_teorica(1), posa_iniziale_teorica(2));
                fprintf(' - Posa Teorica (Odometria): [%.2f, %.2f]\n', ROBOT_POSE(1), ROBOT_POSE(2));
                fprintf(' - Posa Corretta (SLAM):     [%.2f, %.2f]\n', posa_reale(1), posa_reale(2));
                fprintf(' - DERIVA FISICA TOTALE:     %.3f metri!\n', drift_error);
                fprintf('======================================================\n');

                ROBOT_POSE = posa_reale;
                EKF.P      = diag([0.01, 0.01, deg2rad(1)^2]);
            else
                fprintf('\nDRIFT TROPPO ELEVATO! Lo SLAM non riconosce la stanza.\n');
            end
        end
        mostraMappa();
    end
end

function rebuildSLAM()
    global slamObj history
    slamObj = lidarSLAM(67, 8);
    slamObj.LoopClosureThreshold    = 250;
    slamObj.LoopClosureSearchRadius = 3.0;
    if isempty(history.scans), return; end
    addScan(slamObj, history.scans{1});
    for i = 2:length(history.scans)
        [~, p_curr] = scansAndPoses(slamObj);
        p_prev      = p_curr(end,:)';
        p_targ      = history.poses(i,:)';
        R_prev      = [cos(p_prev(3)), -sin(p_prev(3)); sin(p_prev(3)), cos(p_prev(3))];
        rel_xy      = (R_prev' * (p_targ(1:2) - p_prev(1:2)))';
        rel_th      = wrapToPi(p_targ(3) - p_prev(3));
        addScan(slamObj, history.scans{i}, [rel_xy, rel_th]);
    end
    if slamObj.PoseGraph.NumNodes > 1
        optimizePoseGraph(slamObj.PoseGraph);
    end
end

function mostraMappa()
    global slamObj ROBOT_POSE EKF h_map_points h_traj h_ellipse h_robot h_target targetPos
    try
        if slamObj.PoseGraph.NumNodes > 0
            [scans, poses] = scansAndPoses(slamObj);
            all_x = [];
            all_y = [];
            for i = 1:numel(scans)
                p      = poses(i,:);
                p_loc  = scans{i}.Cartesian;
                rotM   = [cos(p(3)), -sin(p(3)); sin(p(3)), cos(p(3))];
                p_glob = (rotM * p_loc')' + [p(1), p(2)];
                all_x  = [all_x; p_glob(:,1)]; %#ok<AGROW>
                all_y  = [all_y; p_glob(:,2)]; %#ok<AGROW>
            end
            set(h_map_points, 'XData', all_x, 'YData', all_y);
            set(h_traj, 'XData', poses(:,1), 'YData', poses(:,2));

            [V, D] = eig(EKF.P(1:2, 1:2));
            t      = linspace(0, 2*pi, 50);
            xy     = 2 * V * sqrt(abs(D)) * [cos(t); sin(t)] + ROBOT_POSE(1:2);
            set(h_ellipse, 'XData', xy(1,:), 'YData', xy(2,:));
            set(h_robot, 'XData', ROBOT_POSE(1), 'YData', ROBOT_POSE(2), ...
                         'UData', cos(ROBOT_POSE(3))*0.3, 'VData', sin(ROBOT_POSE(3))*0.3);
            if ~isempty(all_x)
                axis([min(all_x)-0.5, max(all_x)+0.5, min(all_y)-0.5, max(all_y)+0.5]);
            end
        end
    catch
    end
    if ~isempty(targetPos)
        set(h_target, 'XData', targetPos(1), 'YData', targetPos(2));
    end
    drawnow limitrate;
end

function scan = catturaScanConFlush(u_rx, u_tx, ip, target_giri)
    % Svuota eventuale buffer residuo
    while u_rx.NumDatagramsAvailable > 0
        read(u_rx, u_rx.NumDatagramsAvailable, "string");
    end

    write(u_tx, sprintf("SCAN,%d", target_giri), "string", ip, 5005);

    pts     = [];
    timeout = max(10.0, 3.0 + 2.2 * target_giri);
    t_start = tic;
    done    = false;

    while toc(t_start) < timeout
        if u_rx.NumDatagramsAvailable > 0
            data = read(u_rx, u_rx.NumDatagramsAvailable, "string");
            for i = 1:numel(data)
                str = strtrim(data(i).Data);
                if strcmpi(str, "DONE")
                    done = true;
                    break;
                elseif strcmpi(str, "EMG")
                    continue;
                end
                % Parsing C-nativo vettorizzato (100x piu' rapido di split + str2double)
                s_clean = strrep(char(str), ';', ' ');
                vals    = sscanf(s_clean, '%f,%f');
                n_pairs = floor(numel(vals) / 2);
                if n_pairs > 0
                    pts = [pts; reshape(vals(1:2*n_pairs), 2, n_pairs)']; %#ok<AGROW>
                end
            end
        end
        if done, break; end
        pause(0.004);
    end

    if size(pts, 1) > 20
        d   = hypot(pts(:,1), pts(:,2));
        pts = pts(d >= 0.05 & d <= 8.0, :);

        % Downsampling fine su griglia di 2 cm per accelerare addScan e PoseGraph
        x_idx    = round(pts(:,1) / 0.02);
        y_idx    = round(pts(:,2) / 0.02);
        [~, uid] = unique([x_idx, y_idx], 'rows');
        pts      = pts(uid, :);

        fprintf(' [LiDAR: %d pt] ', size(pts, 1));
        scan = lidarScan(pts);
    else
        scan = lidarScan([]);
    end
end
