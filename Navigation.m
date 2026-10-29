% =========================================================================
% TESI 2: NAVIGAZIONE AUTONOMA A* + SCAN-TO-MAP CORRELATIVO & EKF ADATTIVO
% =========================================================================
% Architettura di Localizzazione e Navigazione Infallibile:
%   1. Dual-Map Architecture:
%      - Mappa C-Space (con spazio ignoto bloccato + pulizia speckle) per A*
%      - Nuvola Muri Reali SLAM (2 cm, zero spazio ignoto) per Localizzazione
%   2. Scan-to-Map Globale a 2 Stadi + Recovery Anti-Kidnapping:
%      - Stadio 1: Correlative Branch-and-Bound Grid Matching (matchScansGrid)
%                  immune ai minimi locali anche con forti slittamenti angolari
%      - Stadio 2: Raffinamento NDT sub-centimetrico (matchScans CellSize 0.5m)
%      - Stadio 3: Verifica geometrica reale (Inlier Ratio & Trimmed RMSE) e
%                  Wide-Window Recovery automatico (+/- 90 deg, +/- 0.85 m)
%   3. Filtro EKF Ibrido Auto-Risanante in Forma di Joseph:
%      - Fusione completa 3-DoF con R adattiva basata su RMSE/Inlier reali
%      - Slip-Recovery automatico se le ruote slittano ma il LiDAR vede i muri
%      - Covariance Floor anti-starvation (il filtro non diventa mai "cieco")
%   4. Controllo Moto & UDP Senza Interferenze:
%      - Rotazioni sul posto prive di pacchetti "SCAN,1" in volo (zero blur)
%      - Sincronizzazione rigorosa del pacchetto "DONE" + C-level sscanf
% =========================================================================

function Navigation()
    global u_in u_out hFig ROBOT_POSE binMap_clean GLOBAL_WALLS
    global h_map_points h_traj h_ellipse h_robot h_target h_actual_traj actual_path_history
    global PLANNED_PATH ICP_LOG h_safety_cone EKF

    ICP_LOG      = [];
    PLANNED_PATH = [];

    % Inizializzazione matrici di covarianza EKF (calibrata per trazione 4WD skid-steering)
    EKF.P     = diag([0.010, 0.010, deg2rad(2.0)^2]);
    EKF.Q     = diag([0.025, 0.025, deg2rad(8.0)^2]);
    EKF.R     = diag([0.002, 0.002, deg2rad(1.5)^2]);
    EKF.P_min = diag([0.0025, 0.0025, deg2rad(1.2)^2]);

    [fileName, pathName] = uigetfile('*.mat', 'Seleziona Mappa Pre-Registrata');
    if isequal(fileName, 0), return; end

    fullPath    = fullfile(pathName, fileName);
    data_struct = load(fullPath);
    if ~isfield(data_struct, 'ROBOT_POSE')
        error('File di log non valido o corrotto (manca ROBOT_POSE).');
    end
    ROBOT_POSE = double(data_struct.ROBOT_POSE(:));

    % =====================================================================
    % COSTRUZIONE DUAL-MAP:
    %   1) binMap_clean -> per A* (muri + spazio ignoto = 1, spazio libero = 0)
    %   2) GLOBAL_WALLS -> SOLO veri punti muro LiDAR (zero spazio ignoto!)
    % =====================================================================
    [binMap_clean, GLOBAL_WALLS, ROBOT_POSE] = estraiMappaEMuriReali(data_struct, ROBOT_POSE);

    fprintf('==================================================================\n');
    fprintf(' FASE 2 ONLINE: Navigazione A* + Correlative Scan-to-Map EKF\n');
    fprintf(' Mappa caricata: %d punti muro reali estratti (risoluzione 2 cm).\n', size(GLOBAL_WALLS, 1));
    fprintf(' Posa iniziale:  [X = %.2f m, Y = %.2f m, Theta = %.1f deg]\n', ...
            ROBOT_POSE(1), ROBOT_POSE(2), rad2deg(ROBOT_POSE(3)));
    fprintf('  [N] -> Allineamento globale ad ampio raggio (5 giri, +/-90 deg)\n');
    fprintf('  [P] -> Clicca destinazione ed esegui navigazione autonoma\n');
    fprintf('  [Q] -> Genera Dashboard Report ed esporta risultati (.mat / .csv)\n');
    fprintf('==================================================================\n');

    actual_path_history = [];
    try delete(udpport("datagram", "LocalPort", 5007)); catch; end
    u_in  = udpport("datagram", "LocalPort", 5007, "Timeout", 5);
    u_out = udpport("datagram");

    hFig = figure('Name', 'ROM3 - Navigation (Correlative Scan-to-Map + Adaptive EKF)', ...
                  'Color', 'w', 'WindowKeyPressFcn', @gestoreEventi, ...
                  'Units', 'normalized', 'Position', [0.08 0.08 0.84 0.84]);
    hold on; grid on; axis equal;
    title('WORLD VIEW (Tracking Correlativo Scan-to-Map + EKF Adattivo & Re-Planning A*)');
    show(binMap_clean);

    % Visualizza i veri punti muro estratti dallo SLAM sopra la griglia
    plot(GLOBAL_WALLS(:,1), GLOBAL_WALLS(:,2), 'k.', 'MarkerSize', 3.0, ...
         'DisplayName', 'Muri Reali SLAM');

    h_map_points  = plot(NaN, NaN, 'm.', 'MarkerSize', 4.0, 'DisplayName', 'Scan LiDAR Live');
    h_actual_traj = plot(NaN, NaN, 'b-',  'LineWidth', 2.2, 'DisplayName', 'Traiettoria Reale');
    h_traj        = plot(NaN, NaN, 'r-o', 'LineWidth', 2.2, 'MarkerSize', 6, 'DisplayName', 'Traiettoria Pianificata A*');
    h_safety_cone = fill(NaN, NaN, 'g', 'FaceAlpha', 0.22, 'EdgeColor', 'g', ...
                         'LineWidth', 1.5, 'DisplayName', 'Cono Sicurezza (\pm10°)');
    h_ellipse     = plot(NaN, NaN, 'r:', 'LineWidth', 1.4, 'DisplayName', 'Incertezza EKF (2\sigma)');
    h_robot       = quiver(ROBOT_POSE(1), ROBOT_POSE(2), ...
                           cos(ROBOT_POSE(3))*0.3, sin(ROBOT_POSE(3))*0.3, ...
                           'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5, 'DisplayName', 'Robot');
    h_target      = plot(NaN, NaN, 'gx', 'MarkerSize', 15, 'LineWidth', 2.5, 'DisplayName', 'Goal');

    while ishandle(hFig)
        pause(0.1);
    end
end

% =========================================================================
% ESTRAZIONE DUAL-MAP: SEPARA LO SPAZIO IGNOTO DAI VERI MURI FISICI LIDAR
% =========================================================================
function [binMap_clean, global_walls, pose_out] = estraiMappaEMuriReali(data_struct, pose_in)
    pose_out = pose_in;
    wall_pts = [];

    if isfield(data_struct, 'slamObj')
        slamObj = data_struct.slamObj;
        [scans, poses] = scansAndPoses(slamObj);

        % Verifica automatica e riparazione di eventuali scan invertiti (180 deg)
        % prodotti dal vecchio bug di Mapping_originale.m
        if isfield(data_struct, 'history') && isfield(data_struct.history, 'poses') && ...
           size(data_struct.history.poses, 1) == numel(scans)
            hist_poses = data_struct.history.poses;
            needs_repair = false;
            for i = 1:numel(scans)
                if abs(wrapToPi(poses(i,3) - hist_poses(i,3))) > pi/2
                    scans{i} = lidarScan(-scans{i}.Cartesian);
                    needs_repair = true;
                end
            end
            if needs_repair
                fprintf('[AUTO-REPAIR] Ripristinata coerenza orientamento scan nel PoseGraph...\n');
                slamFixed = lidarSLAM(67, 8);
                slamFixed.LoopClosureThreshold    = 250;
                slamFixed.LoopClosureSearchRadius = 3.0;
                addScan(slamFixed, scans{1});
                for i = 2:numel(scans)
                    [~, p_curr] = scansAndPoses(slamFixed);
                    p_prev      = p_curr(end,:)';
                    p_targ      = hist_poses(i,:)';
                    R_prev      = [cos(p_prev(3)), -sin(p_prev(3)); sin(p_prev(3)), cos(p_prev(3))];
                    rel_xy      = (R_prev' * (p_targ(1:2) - p_prev(1:2)))';
                    rel_th      = wrapToPi(p_targ(3) - p_prev(3));
                    addScan(slamFixed, scans{i}, [rel_xy, rel_th]);
                end
                if slamFixed.PoseGraph.NumNodes > 1
                    optimizePoseGraph(slamFixed.PoseGraph);
                end
                [scans, poses] = scansAndPoses(slamFixed);
                pose_out       = poses(end,:)';
            end
        end

        % 1. Estrazione diretta della nuvola di punti LiDAR reale dai nodi SLAM
        for i = 1:numel(scans)
            sc_glob  = transformScan(scans{i}, poses(i,:));
            wall_pts = [wall_pts; sc_glob.Cartesian]; %#ok<AGROW>
        end

        % 2. Costruzione occupancyMap probabilistica per A* e rinforzo celle muro (> 0.65)
        map_raw = buildMap(scans, poses, 20, 5);
        occ_mat = occupancyMatrix(map_raw);
        [r_w, c_w] = find(occ_mat > 0.65);
        if ~isempty(r_w)
            wall_pts = [wall_pts; grid2world(map_raw, [r_w, c_w])];
        end

        % Per A* (binMap_clean): spazio libero esplorato (< 0.4) = 0, muri e ignoto (> 0.4) = 1
        % Pulizia morfologica dei singoli pixel isolati di rumore dentro lo spazio libero
        free_mask = occ_mat < 0.40;
        % Se un pixel non-libero e' completamente circondato da pixel liberi (>= 7 vicini liberi su 8),
        % e' un singolo punto di rumore isolato e viene ripulito per non bloccare A*
        free_neighbors = conv2(double(free_mask), [1 1 1; 1 0 1; 1 1 1], 'same');
        speckle_noise  = (~free_mask) & (occ_mat < 0.65) & (free_neighbors >= 7);
        occ_clean      = (occ_mat > 0.40) & (~speckle_noise);

        binMap_clean = binaryOccupancyMap(occ_clean, map_raw.Resolution);
        binMap_clean.GridLocationInWorld = map_raw.GridLocationInWorld;

    elseif isfield(data_struct, 'export_map')
        binMap_clean = data_struct.export_map;
        occ_bin      = occupancyMatrix(binMap_clean) > 0.4;
        % Estrae solo il bordo adiacente allo spazio libero (esclude il blocco pieno di spazio ignoto!)
        free_mask    = ~occ_bin;
        has_free_nbr = conv2(double(free_mask), ones(3,3), 'same') > 0;
        [r_b, c_b]   = find(occ_bin & has_free_nbr);
        wall_pts     = grid2world(binMap_clean, [r_b, c_b]);
    else
        error('Nessuna mappa valida trovata nel file selezionato.');
    end

    % De-duplicazione fine su griglia di 2 cm per preservare la densita' dei muri
    grid_res        = 0.02;
    x_idx           = round(wall_pts(:,1) / grid_res);
    y_idx           = round(wall_pts(:,2) / grid_res);
    [~, unique_idx] = unique([x_idx, y_idx], 'rows');
    global_walls    = double(wall_pts(unique_idx, :));
end

% =========================================================================
% GESTORE EVENTI DA TASTIERA ([N], [P], [Q])
% =========================================================================
function gestoreEventi(~, ev)
    global u_in u_out ROBOT_POSE hFig h_safety_cone EKF
    global h_traj h_target h_actual_traj actual_path_history binMap_clean
    global PLANNED_PATH ICP_LOG

    target_ip = "172.20.10.2";
    tasto     = lower(ev.Key);

    % =====================================================================
    % TASTO [Q]: ESPORTA REPORT DI SESSIONE E APRI DASHBOARD GRAFICO
    % =====================================================================
    if strcmp(tasto, 'q')
        fprintf('\n[REPORT] Generazione Report di Fine Corsa e Dashboard...\n');
        if isempty(actual_path_history)
            fprintf('Nessun dato di navigazione registrato da esportare.\n');
            return;
        end
        timestamp    = datestr(now, 'yyyymmdd_HHMMSS');
        filename_mat = sprintf('Nav_Session_%s.mat', timestamp);
        filename_csv = sprintf('ICP_Precision_Log_%s.csv', timestamp);

        save(filename_mat, 'actual_path_history', 'PLANNED_PATH', 'ICP_LOG', 'ROBOT_POSE');
        if ~isempty(ICP_LOG)
            T_log = array2table(ICP_LOG, ...
                'VariableNames', {'Correzione_Traslazione_m', 'Correzione_Rotazione_deg'});
            writetable(T_log, filename_csv);
        end

        figure('Name', 'Report di Navigazione (Tesi)', 'Color', 'w', ...
               'Units', 'normalized', 'Position', [0.12 0.12 0.75 0.75]);

        subplot(2, 2, [1 3]);
        hold on; grid on; axis equal;
        title('Confronto Traiettoria Ideale (A*) vs Traiettoria Eseguita (EKF-ICP)');
        show(binMap_clean);
        if ~isempty(PLANNED_PATH)
            plot(PLANNED_PATH(:,1), PLANNED_PATH(:,2), 'r--o', 'LineWidth', 2.2, ...
                 'DisplayName', 'Percorso Ideale (A* + LOS)');
        end
        plot(actual_path_history(:,1), actual_path_history(:,2), 'b-o', 'LineWidth', 2.2, ...
             'MarkerSize', 4, 'DisplayName', 'Percorso Reale (EKF + Scan-to-Map)');
        legend('Location', 'best');

        subplot(2, 2, 2);
        if ~isempty(ICP_LOG)
            plot(1:size(ICP_LOG,1), ICP_LOG(:,1), 'b-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
            grid on;
            title('Correzione Scan-to-Map: Traslazione (\Delta d)');
            xlabel('Step di Navigazione'); ylabel('Correzione [m]');
        end

        subplot(2, 2, 4);
        if ~isempty(ICP_LOG)
            plot(1:size(ICP_LOG,1), ICP_LOG(:,2), 'm-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'm');
            grid on;
            title('Correzione Scan-to-Map: Orientamento (\Delta\theta)');
            xlabel('Step di Navigazione'); ylabel('Correzione [gradi]');
        end

        fprintf('[REPORT OK] File salvati (%s, %s) e Dashboard visualizzato!\n', filename_mat, filename_csv);

    % =====================================================================
    % TASTO [N]: ALLINEAMENTO GLOBALE AD ALTA PRECISIONE (5 GIRI, AMPIO RAGGIO)
    % =====================================================================
    elseif strcmp(tasto, 'n')
        fprintf('\n[ALLINEAMENTO] Scansione di localizzazione ad alta densita'' (5 giri)...\n');
        flushUDP(u_in);
        scan = catturaScanConFlush(u_in, u_out, target_ip, 5);
        if isa(scan, 'lidarScan') && ~isempty(scan.Cartesian)
            [z_meas, icp_ok, match_info] = localizzaConMatematicaPura(scan, ROBOT_POSE, true);
            if icp_ok
                % Su comando manuale [N] agganciamo direttamente la posa verificata
                ROBOT_POSE = z_meas;
                EKF.P      = diag([0.005, 0.005, deg2rad(1.5)^2]);
                fprintf('[ALLINEAMENTO OK] Posa agganciata: [%.2f m, %.2f m, %.1f deg] (Inlier=%.0f%%, RMSE=%.1f cm)\n', ...
                        ROBOT_POSE(1), ROBOT_POSE(2), rad2deg(ROBOT_POSE(3)), ...
                        100*match_info.inlier_ratio, 100*match_info.rmse);
            end
        end
        aggiornaGrafica(scan);

    % =====================================================================
    % TASTO [P]: PIANIFICAZIONE A* E NAVIGAZIONE AUTONOMA STOP & GO
    % =====================================================================
    elseif strcmp(tasto, 'p')
        % 0. Riallineamento preliminare automatico della posa prima di pianificare
        fprintf('\n[PRE-CHECK] Verifica posa iniziale con scansione rapida...\n');
        flushUDP(u_in);
        scan_init = catturaScanAdattivo(u_in, u_out, target_ip);
        if isa(scan_init, 'lidarScan') && ~isempty(scan_init.Cartesian)
            [z_init, ok_init, info_init] = localizzaConMatematicaPura(scan_init, ROBOT_POSE, true);
            if ok_init
                aggiornaPosaIbrida(z_init, info_init);
            end
            aggiornaGrafica(scan_init);
        end

        fprintf('\nClicca la DESTINAZIONE FINALE sulla mappa...');
        [gx, gy] = ginput(1);
        goal_pt  = [gx, gy];

        cSpace = copy(binMap_clean);
        inflate(cSpace, 0.18);
        if checkOccupancy(cSpace, goal_pt) == 1
            fprintf('\n[WARN] Punto cliccato vicino a un bordo: cerco il punto libero piu'' vicino...\n');
            goal_pt = trovaPuntoLiberoPiuVicino(cSpace, goal_pt, 0.25);
            if isempty(goal_pt)
                fprintf('[ERRORE] Destinazione non valida (cade dentro un ostacolo).\n');
                return;
            end
        end

        simplified_path = calcolaPercorsoAStarLOS(cSpace, ROBOT_POSE(1:2)', goal_pt);
        if isempty(simplified_path)
            fprintf('\n[ERRORE] Nessun percorso libero verso la destinazione.\n');
            return;
        end
        PLANNED_PATH = simplified_path;

        if ishandle(h_traj)
            set(h_traj, 'XData', simplified_path(:,1), 'YData', simplified_path(:,2));
        end
        if ishandle(h_target)
            set(h_target, 'XData', goal_pt(1), 'YData', goal_pt(2));
        end

        actual_path_history = ROBOT_POSE(1:2)';
        fprintf('\n======================================================\n');
        fprintf('AVVIO NAVIGAZIONE (A* + Correlative Scan-to-Map EKF)\n');
        fprintf('======================================================\n');
        set(h_safety_cone, 'FaceColor', 'g', 'EdgeColor', 'g');

        wp_idx = 2;
        while wp_idx <= size(simplified_path, 1)
            if ~ishandle(hFig), break; end
            target_wp = simplified_path(wp_idx,:);
            dx        = target_wp(1) - ROBOT_POSE(1);
            dy        = target_wp(2) - ROBOT_POSE(2);

            if hypot(dx, dy) <= 0.15
                wp_idx = wp_idx + 1;
                continue;
            end
            emergenza_attivata = false;
            pts_emergenza      = [];

            % =============================================================
            % FASE 1: ROTAZIONE SUL POSTO (ALLINEAMENTO ANGOLARE PULITO)
            % =============================================================
            rot_attempts = 0;
            while rot_attempts < 6
                dx = target_wp(1) - ROBOT_POSE(1);
                dy = target_wp(2) - ROBOT_POSE(2);
                if hypot(dx, dy) <= 0.15, break; end

                ang_rel = wrapToPi(atan2(dy, dx) - ROBOT_POSE(3));
                if abs(ang_rel) <= deg2rad(4.0), break; end

                rot_attempts = rot_attempts + 1;
                chunk_ang    = sign(ang_rel) * min(abs(ang_rel), deg2rad(90));
                t_rot        = abs(chunk_ang) * 0.396;
                if t_rot > 0 && t_rot < 0.06, t_rot = 0.06; end

                fprintf('  [ROT] %+.0f deg (residuo: %+.0f deg)...\n', ...
                        rad2deg(chunk_ang), rad2deg(ang_rel));
                write(u_out, sprintf('M,%.2f,%.2f,%.2f', ...
                      -0.4*sign(chunk_ang), 0.4*sign(chunk_ang), t_rot), ...
                      "string", target_ip, 5005);

                % Durante la rotazione sul posto NON inviamo SCAN,1:
                % attendiamo il completamento esatto del moto + 0.30s di smorzamento
                % per avere una scansione LiDAR ferma al 100% e senza pacchetti residui
                pause(t_rot + 0.30);

                % Predizione EKF per rotazione sul posto
                ROBOT_POSE(3) = wrapToPi(ROBOT_POSE(3) + chunk_ang);
                Q_rot         = diag([0.004, 0.004, deg2rad(10.0)^2]);
                EKF.P         = EKF.P + Q_rot;
                EKF.P         = 0.5 * (EKF.P + EKF.P');

                flushUDP(u_in);
                scan = catturaScanAdattivo(u_in, u_out, target_ip);
                if isa(scan, 'lidarScan') && ~isempty(scan.Cartesian)
                    [z_meas, icp_ok, match_info] = localizzaConMatematicaPura(scan, ROBOT_POSE, false);
                    if icp_ok
                        aggiornaPosaIbrida(z_meas, match_info);
                    end
                end
                aggiornaGrafica(scan);

                % Se siamo allineati entro 5 gradi al target, procediamo subito ad avanzare
                ang_residuo = wrapToPi(atan2(target_wp(2) - ROBOT_POSE(2), target_wp(1) - ROBOT_POSE(1)) - ROBOT_POSE(3));
                if abs(ang_residuo) <= deg2rad(5.0)
                    break;
                end
            end

            % =============================================================
            % FASE 2: AVANZAMENTO RETTILINEO CON MONITORAGGIO CONO
            % =============================================================
            while true
                dx   = target_wp(1) - ROBOT_POSE(1);
                dy   = target_wp(2) - ROBOT_POSE(2);
                dist = hypot(dx, dy);
                if dist <= 0.15
                    wp_idx = wp_idx + 1;
                    break;
                end

                % Controllo deriva angolare cumulativa in corsa
                err_ang = wrapToPi(atan2(dy, dx) - ROBOT_POSE(3));
                if abs(err_ang) > deg2rad(16)
                    fprintf('  [ALLINEA] Deriva angolare (%.1f deg). Riallineamento sul posto...\n', rad2deg(err_ang));
                    break;
                end

                chunk_dist = min(dist, 0.30);
                t_avanti   = chunk_dist * 2.591;

                fprintf('  [AVANTI] %.2f m (distanza al waypoint: %.2f m)...\n', chunk_dist, dist);
                write(u_out, sprintf('M,0.40,0.40,%.2f', t_avanti), "string", target_ip, 5005);

                [ostacolo, t_eff, pts_emg] = attesaAttivaConSicurezza(t_avanti, target_ip);

                % Predizione cinematica EKF proporzionale al tempo realmente percorso
                fraz_avanti   = min(1.0, max(0.0, t_eff / max(t_avanti, 0.01)));
                dist_eff      = chunk_dist * fraz_avanti;
                th_prev       = ROBOT_POSE(3);
                ROBOT_POSE(1) = ROBOT_POSE(1) + dist_eff * cos(th_prev);
                ROBOT_POSE(2) = ROBOT_POSE(2) + dist_eff * sin(th_prev);

                F = [1, 0, -dist_eff * sin(th_prev);
                     0, 1,  dist_eff * cos(th_prev);
                     0, 0,  1];
                scale_q = max(0.35, dist_eff / 0.30);
                EKF.P   = F * EKF.P * F' + EKF.Q * scale_q;
                EKF.P   = 0.5 * (EKF.P + EKF.P');

                if ostacolo
                    emergenza_attivata  = true;
                    pts_emergenza       = pts_emg;
                    actual_path_history = [actual_path_history; ROBOT_POSE(1:2)']; %#ok<AGROW>
                    break;
                end

                % Pausa di arresto completo (0.30 s) per eliminare ogni motion-blur dallo scan
                pause(0.30);

                flushUDP(u_in);
                scan = catturaScanAdattivo(u_in, u_out, target_ip);
                if isa(scan, 'lidarScan') && ~isempty(scan.Cartesian)
                    [z_meas, icp_ok, match_info] = localizzaConMatematicaPura(scan, ROBOT_POSE, false);
                    if icp_ok
                        aggiornaPosaIbrida(z_meas, match_info);
                    end
                end

                actual_path_history = [actual_path_history; ROBOT_POSE(1:2)']; %#ok<AGROW>
                if ishandle(h_actual_traj)
                    set(h_actual_traj, 'XData', actual_path_history(:,1), ...
                                       'YData', actual_path_history(:,2));
                end
                aggiornaGrafica(scan);
            end

            if emergenza_attivata
                [nuovo_path, abort_nav] = gestisciRecuperoOstacolo(target_ip, cSpace, goal_pt, pts_emergenza);
                if abort_nav
                    fprintf('[STOP] Navigazione interrotta in sicurezza: ostacolo permanente.\n');
                    break;
                elseif ~isempty(nuovo_path)
                    simplified_path = nuovo_path;
                    PLANNED_PATH    = simplified_path;
                    wp_idx          = 2;
                end
                continue;
            end
        end

        fprintf('======================================================\n');
        fprintf('Target finale raggiunto! Motori fermi.\n');
        write(u_out, 'M,0,0,0.1', "string", target_ip, 5005);
    end
end

% =========================================================================
% RICERCA PUNTO LIBERO PIU' VICINO SE L'UTENTE CLICCA VICINO AL MURO
% =========================================================================
function pt_out = trovaPuntoLiberoPiuVicino(cSpace, pt_in, max_r)
    pt_out   = [];
    [gX, gY] = meshgrid(pt_in(1)-max_r : 0.03 : pt_in(1)+max_r, ...
                        pt_in(2)-max_r : 0.03 : pt_in(2)+max_r);
    cand     = [gX(:), gY(:)];
    valid    = cand(:,1) >= cSpace.XWorldLimits(1) & cand(:,1) <= cSpace.XWorldLimits(2) & ...
               cand(:,2) >= cSpace.YWorldLimits(1) & cand(:,2) <= cSpace.YWorldLimits(2);
    cand     = cand(valid, :);
    if isempty(cand), return; end
    occ      = checkOccupancy(cSpace, cand);
    free_pts = cand(occ == 0, :);
    if isempty(free_pts), return; end
    [~, idx] = min(hypot(free_pts(:,1) - pt_in(1), free_pts(:,2) - pt_in(2)));
    pt_out   = free_pts(idx, :);
end

% =========================================================================
% PIANIFICAZIONE A* + LINE-OF-SIGHT (LOS) SMOOTHING CON MARGINE SICURO
% =========================================================================
function simplified_path = calcolaPercorsoAStarLOS(cSpace, start_pt, goal_pt)
    simplified_path = [];
    cSpace_plan     = copy(cSpace);

    % Clearing locale intorno alla posa attuale del robot
    r_clear  = 0.25;
    [cX, cY] = meshgrid(start_pt(1)-r_clear : 0.04 : start_pt(1)+r_clear, ...
                        start_pt(2)-r_clear : 0.04 : start_pt(2)+r_clear);
    clear_pts = [cX(:), cY(:)];
    valid_idx = clear_pts(:,1) >= cSpace_plan.XWorldLimits(1) & ...
                clear_pts(:,1) <= cSpace_plan.XWorldLimits(2) & ...
                clear_pts(:,2) >= cSpace_plan.YWorldLimits(1) & ...
                clear_pts(:,2) <= cSpace_plan.YWorldLimits(2);
    setOccupancy(cSpace_plan, clear_pts(valid_idx,:), 0);
    setOccupancy(cSpace_plan, goal_pt, 0);

    planner  = plannerAStarGrid(cSpace_plan);
    path_idx = plan(planner, world2grid(cSpace_plan, start_pt), world2grid(cSpace_plan, goal_pt));
    if isempty(path_idx)
        return;
    end

    % Copia con lieve extra-margine (3 cm) per evitare che il LOS tagli troppo vicino agli spigoli
    cSpace_los = copy(cSpace_plan);
    inflate(cSpace_los, 0.03);
    setOccupancy(cSpace_los, clear_pts(valid_idx,:), 0);
    setOccupancy(cSpace_los, goal_pt, 0);

    path_xy         = grid2world(cSpace_plan, path_idx);
    simplified_path = path_xy(1,:);
    curr_idx        = 1;

    while curr_idx < size(path_xy, 1)
        found_next = false;
        for test_idx = size(path_xy,1) : -1 : (curr_idx+1)
            distAB   = norm(path_xy(test_idx,:) - path_xy(curr_idx,:));
            alpha    = linspace(0, 1, max(2, ceil(distAB / 0.01)))';
            line_pts = path_xy(curr_idx,:) + alpha .* (path_xy(test_idx,:) - path_xy(curr_idx,:));
            if max(checkOccupancy(cSpace_los, line_pts)) == 0
                simplified_path = [simplified_path; path_xy(test_idx,:)]; %#ok<AGROW>
                curr_idx        = test_idx;
                found_next      = true;
                break;
            end
        end
        if ~found_next
            for test_idx = size(path_xy,1) : -1 : (curr_idx+1)
                distAB   = norm(path_xy(test_idx,:) - path_xy(curr_idx,:));
                alpha    = linspace(0, 1, max(2, ceil(distAB / 0.01)))';
                line_pts = path_xy(curr_idx,:) + alpha .* (path_xy(test_idx,:) - path_xy(curr_idx,:));
                if max(checkOccupancy(cSpace_plan, line_pts)) == 0
                    simplified_path = [simplified_path; path_xy(test_idx,:)]; %#ok<AGROW>
                    curr_idx        = test_idx;
                    found_next      = true;
                    break;
                end
            end
        end
        if ~found_next
            curr_idx        = curr_idx + 1;
            simplified_path = [simplified_path; path_xy(curr_idx,:)]; %#ok<AGROW>
        end
    end
    simplified_path(end,:) = goal_pt;
end

% =========================================================================
% FLUSH BUFFER UDP SICURO
% =========================================================================
function flushUDP(u_rx)
    while u_rx.NumDatagramsAvailable > 0
        read(u_rx, u_rx.NumDatagramsAvailable, "string");
    end
end

% =========================================================================
% FUSIONE EKF ADATTIVA AUTO-RISANANTE IN FORMA DI JOSEPH
% =========================================================================
function aggiornaPosaIbrida(z_meas, match_info)
    global ROBOT_POSE EKF

    if nargin < 2 || isempty(match_info)
        match_info.inlier_ratio = 0.75;
        match_info.rmse         = 0.025;
    end

    % 1. Rumore di misura R adattivo basato sulla qualita' geometrica reale dello scan match
    sigma_xy = max(0.020, match_info.rmse / max(0.45, match_info.inlier_ratio));
    sigma_th = deg2rad(max(1.0, 2.5 * (1.0 - 0.5 * match_info.inlier_ratio)));
    R_adapt  = diag([sigma_xy^2, sigma_xy^2, sigma_th^2]);

    % 2. Aggiornamento Orientamento (protetto e pesato sulla confidenza)
    err_th = wrapToPi(z_meas(3) - ROBOT_POSE(3));
    if match_info.inlier_ratio >= 0.60
        % Alta certezza geometrica sui muri della stanza: aggancio diretto dell'angolo
        ROBOT_POSE(3) = wrapToPi(z_meas(3));
        EKF.P(3,3)    = max(EKF.P_min(3,3), R_adapt(3,3));
    elseif abs(err_th) <= deg2rad(30)
        % Fusione scalare di Kalman sull'orientamento
        k_th          = EKF.P(3,3) / (EKF.P(3,3) + R_adapt(3,3));
        ROBOT_POSE(3) = wrapToPi(ROBOT_POSE(3) + k_th * err_th);
        EKF.P(3,3)    = max(EKF.P_min(3,3), (1 - k_th) * EKF.P(3,3));
    end
    EKF.P(1:2, 3) = 0;
    EKF.P(3, 1:2) = 0;

    % 3. Gating probabilistico XY con Auto-Recovery dallo slittamento ruote
    innov_xy      = z_meas(1:2) - ROBOT_POSE(1:2);
    P_xy          = EKF.P(1:2, 1:2);
    R_xy          = R_adapt(1:2, 1:2);
    S_xy          = P_xy + R_xy;
    d_mahalanobis = innov_xy' * (S_xy \ innov_xy);

    % Caso A: Innovazione coerente con la covarianza predetta (Chi-Quadro 99.7% -> 11.34)
    if d_mahalanobis < 11.34
        K_xy            = P_xy / S_xy;
        ROBOT_POSE(1:2) = ROBOT_POSE(1:2) + K_xy * innov_xy;

        I2              = eye(2);
        IK              = I2 - K_xy;
        EKF.P(1:2, 1:2) = IK * P_xy * IK' + K_xy * R_xy * K_xy';
        fprintf('  [FUSIONE EKF] Posizione aggiornata (D_M^2=%.2f, Inlier=%.0f%%, RMSE=%.1fcm)\n', ...
                d_mahalanobis, 100*match_info.inlier_ratio, 100*match_info.rmse);

    % Caso B: Slittamento ruote fisico superiore alla predizione odometrica,
    %         MA il LiDAR vede chiaramente i muri della stanza (Inlier >= 60%, RMSE <= 4.5 cm)!
    elseif match_info.inlier_ratio >= 0.60 && match_info.rmse <= 0.045
        P_slip          = P_xy + diag([0.04, 0.04]);
        S_slip          = P_slip + R_xy;
        K_xy            = P_slip / S_slip;
        ROBOT_POSE(1:2) = ROBOT_POSE(1:2) + K_xy * innov_xy;

        I2              = eye(2);
        IK              = I2 - K_xy;
        EKF.P(1:2, 1:2) = IK * P_slip * IK' + K_xy * R_xy * K_xy';
        fprintf('  [SLIP-RECOVERY] Slittamento ruote corretto dal LiDAR (Inlier=%.0f%%, RMSE=%.1fcm)\n', ...
                100*match_info.inlier_ratio, 100*match_info.rmse);

    % Caso C: Vero outlier o corridoio privo di riferimenti longitudinali (Aperture Problem)
    else
        % Inflazione controllata della covarianza per impedire la starvation del filtro
        EKF.P(1:2, 1:2) = 1.30 * EKF.P(1:2, 1:2);
        fprintf('  [GATE MAHALANOBIS] Misura XY scartata (D_M^2=%.2f, Inlier=%.0f%%)\n', ...
                d_mahalanobis, 100*match_info.inlier_ratio);
    end

    % 4. Simmetrizzazione e Covariance Floor (impedisce al filtro di diventare sovra-confidente)
    EKF.P = 0.5 * (EKF.P + EKF.P');
    for d_i = 1:3
        if EKF.P(d_i, d_i) < EKF.P_min(d_i, d_i)
            EKF.P(d_i, d_i) = EKF.P_min(d_i, d_i);
        end
    end
end

% =========================================================================
% MONITORAGGIO ATTIVO CONO DI SICUREZZA FRONTALE (SENZA PACCHETTI ORFANI)
% =========================================================================
function [ostacolo_trovato, t_effettivo, pts_ostacolo] = attesaAttivaConSicurezza(t_movimento, target_ip)
    global u_in u_out h_safety_cone
    ostacolo_trovato = false;
    pts_ostacolo     = [];
    t_start          = tic;
    soglia_critica   = 0.20;

    while toc(t_start) < t_movimento
        % Invia una richiesta SCAN,1 solo se mancano almeno 0.25s alla fine del movimento
        if (t_movimento - toc(t_start)) < 0.22
            pause(max(0.01, t_movimento - toc(t_start)));
            break;
        end

        flushUDP(u_in);
        write(u_out, "SCAN,1", "string", target_ip, 5005);

        pts           = [];
        t_read        = tic;
        done_ricevuto = false;
        emg_hardware  = false;

        % Attende il completamento di SCAN,1 (o timeout breve)
        while toc(t_read) < 0.55
            if u_in.NumDatagramsAvailable > 0
                data = read(u_in, u_in.NumDatagramsAvailable, "string");
                for i = 1:numel(data)
                    str = strtrim(data(i).Data);
                    if strcmpi(str, "EMG")
                        emg_hardware = true;
                    elseif strcmpi(str, "DONE")
                        done_ricevuto = true;
                        break;
                    else
                        s_clean = strrep(char(str), ';', ' ');
                        vals    = sscanf(s_clean, '%f,%f');
                        n_pairs = floor(numel(vals) / 2);
                        if n_pairs > 0
                            pts = [pts; reshape(vals(1:2*n_pairs), 2, n_pairs)']; %#ok<AGROW>
                        end
                    end
                end
            end
            if emg_hardware || done_ricevuto, break; end
            if toc(t_start) >= t_movimento, break; end
            pause(0.004);
        end

        % Se il ciclo e' uscito per fine tempo movimento prima di ricevere DONE,
        % drena l'eventuale DONE residuo per non inquinare la scansione successiva
        if ~done_ricevuto && ~emg_hardware
            t_drain = tic;
            while toc(t_drain) < 0.45
                if u_in.NumDatagramsAvailable > 0
                    d_rem = read(u_in, u_in.NumDatagramsAvailable, "string");
                    if any(strcmpi(strtrim([d_rem.Data]), "DONE"))
                        break;
                    end
                end
                pause(0.005);
            end
        end

        % Controllo 1: Riflesso hardware on-board Raspberry Pi
        if emg_hardware
            t_effettivo = min(t_movimento, toc(t_start));
            fprintf('\n[EMERGENZA HARDWARE] Riflesso On-Board attivato a t=%.2fs!\n', t_effettivo);
            for k = 1:5
                write(u_out, 'M,0,0,0.1', "string", target_ip, 5005);
                pause(0.005);
            end
            set(h_safety_cone, 'FaceColor', 'r', 'EdgeColor', 'r');
            ostacolo_trovato = true;
            pts_ostacolo     = pts;
            return;
        end

        % Controllo 2: Cono di sicurezza frontale (+/- 10 gradi)
        if size(pts, 1) > 5
            angoli        = atan2(pts(:,2), pts(:,1));
            idx_frontali  = angoli >= -deg2rad(10) & angoli <= deg2rad(10);
            dist_frontali = hypot(pts(idx_frontali,1), pts(idx_frontali,2));

            if any(dist_frontali > 0.05 & dist_frontali < soglia_critica)
                t_effettivo = min(t_movimento, toc(t_start));
                min_d_cm    = min(dist_frontali(dist_frontali > 0.05)) * 100;
                fprintf('\n[EMERGENZA] Ostacolo nel cono a %.1f cm (t=%.2fs)! Arresto immediato!\n', ...
                        min_d_cm, t_effettivo);
                for k = 1:8
                    write(u_out, 'M,0,0,0.1', "string", target_ip, 5005);
                    pause(0.005);
                end
                set(h_safety_cone, 'FaceColor', 'r', 'EdgeColor', 'r');
                ostacolo_trovato = true;
                pts_ostacolo     = pts(idx_frontali, :);
                return;
            else
                set(h_safety_cone, 'FaceColor', 'g', 'EdgeColor', 'g');
            end
        end
        pause(0.02);
    end
    t_effettivo = t_movimento;
end

% =========================================================================
% GESTIONE RECUPERO OSTACOLO CON RE-PLANNING DINAMICO A* AUTOMATICO
% =========================================================================
function [nuovo_percorso, abort_nav] = gestisciRecuperoOstacolo(target_ip, cSpace, goal_pt, pts_emg)
    global u_in u_out h_safety_cone h_traj ROBOT_POSE
    nuovo_percorso = [];
    abort_nav      = false;

    fprintf('[RECUPERO] Percorso bloccato. Verifico se l''ostacolo e'' mobile o fisso...\n');
    ostacolo_presente   = true;
    t_recovery          = tic;
    REPLAN_AFTER_S      = 6.0;
    MAX_RECOVERY_WAIT_S = 25.0;
    replan_tentato      = false;

    while ostacolo_presente
        elapsed = toc(t_recovery);

        flushUDP(u_in);
        write(u_out, "SCAN,1", "string", target_ip, 5005);

        pts           = [];
        t_read        = tic;
        done_ricevuto = false;
        while toc(t_read) < 0.80
            if u_in.NumDatagramsAvailable > 0
                data = read(u_in, u_in.NumDatagramsAvailable, "string");
                for i = 1:numel(data)
                    str = strtrim(data(i).Data);
                    if strcmpi(str, "DONE")
                        done_ricevuto = true;
                        break;
                    elseif strcmpi(str, "EMG")
                        continue;
                    end
                    s_clean = strrep(char(str), ';', ' ');
                    vals    = sscanf(s_clean, '%f,%f');
                    n_pairs = floor(numel(vals) / 2);
                    if n_pairs > 0
                        pts = [pts; reshape(vals(1:2*n_pairs), 2, n_pairs)']; %#ok<AGROW>
                    end
                end
            end
            if done_ricevuto, break; end
            pause(0.005);
        end

        if size(pts, 1) > 5
            angoli        = atan2(pts(:,2), pts(:,1));
            idx_frontali  = angoli >= -deg2rad(10) & angoli <= deg2rad(10);
            dist_frontali = hypot(pts(idx_frontali,1), pts(idx_frontali,2));

            if ~any(dist_frontali > 0.05 & dist_frontali < 0.28)
                break;
            end
            pts_emg = pts(idx_frontali & hypot(pts(:,1), pts(:,2)) < 0.40, :);
        end

        if ~replan_tentato && elapsed >= REPLAN_AFTER_S
            replan_tentato = true;
            fprintf('[RE-PLANNING] Ostacolo fisso rilevato. Ricalcolo traiettoria alternativa A*...\n');

            flushUDP(u_in);
            scan_loc = catturaScanConFlush(u_in, u_out, target_ip, 5);
            if isa(scan_loc, 'lidarScan') && ~isempty(scan_loc.Cartesian)
                [z_meas, icp_ok, match_info] = localizzaConMatematicaPura(scan_loc, ROBOT_POSE, false);
                if icp_ok, aggiornaPosaIbrida(z_meas, match_info); end
                aggiornaGrafica(scan_loc);
            end

            if ~isempty(pts_emg)
                rotM     = [cos(ROBOT_POSE(3)), -sin(ROBOT_POSE(3));
                            sin(ROBOT_POSE(3)),  cos(ROBOT_POSE(3))];
                obs_glob = (rotM * pts_emg')' + ROBOT_POSE(1:2)';

                cSpace_dyn = copy(cSpace);
                for k = 1:size(obs_glob, 1)
                    [oX, oY]  = meshgrid(obs_glob(k,1)-0.20 : 0.03 : obs_glob(k,1)+0.20, ...
                                         obs_glob(k,2)-0.20 : 0.03 : obs_glob(k,2)+0.20);
                    in_circle = hypot(oX - obs_glob(k,1), oY - obs_glob(k,2)) <= 0.20;
                    pts_fill  = [oX(in_circle), oY(in_circle)];
                    v_idx     = pts_fill(:,1) >= cSpace_dyn.XWorldLimits(1) & ...
                                pts_fill(:,1) <= cSpace_dyn.XWorldLimits(2) & ...
                                pts_fill(:,2) >= cSpace_dyn.YWorldLimits(1) & ...
                                pts_fill(:,2) <= cSpace_dyn.YWorldLimits(2);
                    if any(v_idx)
                        setOccupancy(cSpace_dyn, pts_fill(v_idx,:), 1);
                    end
                end

                cand_path = calcolaPercorsoAStarLOS(cSpace_dyn, ROBOT_POSE(1:2)', goal_pt);
                if ~isempty(cand_path)
                    fprintf('[RE-PLANNING OK] Nuovo percorso alternativo trovato!\n');
                    nuovo_percorso = cand_path;
                    set(h_safety_cone, 'FaceColor', 'g', 'EdgeColor', 'g');
                    if ishandle(h_traj)
                        set(h_traj, 'XData', nuovo_percorso(:,1), 'YData', nuovo_percorso(:,2));
                    end
                    return;
                else
                    fprintf('[RE-PLANNING] Corridoio bloccato senza deviazioni possibili. Attendo rimozione...\n');
                end
            end
        end

        if elapsed > MAX_RECOVERY_WAIT_S
            fprintf('[SICUREZZA] Timeout (%ds) raggiunto con ostacolo ancora presente.\n', MAX_RECOVERY_WAIT_S);
            abort_nav = true;
            return;
        end
        pause(0.35);
    end

    fprintf('[RECUPERO OK] Ostacolo rimosso! Riallineamento posa Scan-to-Map...\n');
    set(h_safety_cone, 'FaceColor', 'g', 'EdgeColor', 'g');
    flushUDP(u_in);
    scan = catturaScanConFlush(u_in, u_out, target_ip, 5);
    if isa(scan, 'lidarScan') && ~isempty(scan.Cartesian)
        [z_meas, icp_ok, match_info] = localizzaConMatematicaPura(scan, ROBOT_POSE, false);
        if icp_ok, aggiornaPosaIbrida(z_meas, match_info); end
    end
    aggiornaGrafica(scan);
end

% =========================================================================
% LOCALIZZAZIONE GLOBALE SCAN-TO-MAP A 2 STADI + RECOVERY ANTI-KIDNAPPING
%   Stadio 1: Correlative Branch-and-Bound Grid Matching (matchScansGrid)
%   Stadio 2: Raffinamento NDT continuo (matchScans con CellSize 0.5m)
%   Stadio 3: Valutazione geometrica Inlier/RMSE e Recovery automatico +/-90°
% =========================================================================
function [estPose, success, match_info] = localizzaConMatematicaPura(scan, guessPose, is_wide_search)
    global GLOBAL_WALLS ICP_LOG
    if nargin < 3
        is_wide_search = false;
    end

    success                 = false;
    estPose                 = guessPose;
    match_info.inlier_ratio = 0.0;
    match_info.rmse         = 1.0;

    try
        % 1. Estrazione nuvola muri reali entro un raggio di 6.0 metri dalla posa stimata
        dx_glob   = GLOBAL_WALLS(:,1) - guessPose(1);
        dy_glob   = GLOBAL_WALLS(:,2) - guessPose(2);
        dist_sq   = dx_glob.^2 + dy_glob.^2;
        local_pts = GLOBAL_WALLS(dist_sq <= 36.0, :);
        if size(local_pts, 1) < 20
            local_pts = GLOBAL_WALLS;
        end
        if size(local_pts, 1) < 20, return; end

        % 2. Rototraslazione della nuvola muri reali nel frame locale predetto del robot
        th      = -guessPose(3);
        dx      = local_pts(:,1) - guessPose(1);
        dy      = local_pts(:,2) - guessPose(2);
        local_x = dx .* cos(th) - dy .* sin(th);
        local_y = dx .* sin(th) + dy .* cos(th);
        ref_pts = [local_x, local_y];
        refScan = lidarScan(ref_pts);

        % 3. Stadio 1: Ricerca correlativa Branch-and-Bound globale (immune ai minimi locali)
        if is_wide_search
            range_xy = [0.85, 0.85];
            range_th = deg2rad(90);
            res_grid = 22;
        else
            range_xy = [0.45, 0.45];
            range_th = deg2rad(45);
            res_grid = 25;
        end

        [rel_grid, ~] = matchScansGrid(scan, refScan, ...
            'MaxRange', 6.0, ...
            'Resolution', res_grid, ...
            'InitialPose', [0, 0, 0], ...
            'TranslationSearchRange', range_xy, ...
            'RotationSearchRange', range_th);

        % 4. Stadio 2: Raffinamento NDT sub-centimetrico inizializzato sul risultato correlativo
        [rel_fine, ~] = matchScans(scan, refScan, ...
            'InitialPose', rel_grid, ...
            'MaxIterations', 50, ...
            'CellSize', 0.5);

        % Valutazione geometrica oggettiva (distanza punto-muro reale)
        [inlier_g, rmse_g] = valutaQualitaScanToMap(scan, ref_pts, rel_grid);
        [inlier_f, rmse_f] = valutaQualitaScanToMap(scan, ref_pts, rel_fine);

        if (inlier_f / max(0.015, rmse_f)) >= (inlier_g / max(0.015, rmse_g))
            rel_best    = rel_fine;
            inlier_best = inlier_f;
            rmse_best   = rmse_f;
        else
            rel_best    = rel_grid;
            inlier_best = inlier_g;
            rmse_best   = rmse_g;
        end

        % 5. Stadio 3: Auto-Recovery ad ampio raggio se il robot ha subito un forte slittamento
        if ~is_wide_search && (inlier_best < 0.72 || rmse_best > 0.035)
            [rel_wide, ~] = matchScansGrid(scan, refScan, ...
                'MaxRange', 6.0, ...
                'Resolution', 22, ...
                'InitialPose', [0, 0, 0], ...
                'TranslationSearchRange', [0.85, 0.85], ...
                'RotationSearchRange', deg2rad(90));
            [rel_wide_f, ~] = matchScans(scan, refScan, ...
                'InitialPose', rel_wide, ...
                'MaxIterations', 50, ...
                'CellSize', 0.5);

            [inlier_w,  rmse_w]  = valutaQualitaScanToMap(scan, ref_pts, rel_wide);
            [inlier_wf, rmse_wf] = valutaQualitaScanToMap(scan, ref_pts, rel_wide_f);

            if (inlier_wf / max(0.015, rmse_wf)) >= (inlier_w / max(0.015, rmse_w))
                rel_cand    = rel_wide_f;
                inlier_cand = inlier_wf;
                rmse_cand   = rmse_wf;
            else
                rel_cand    = rel_wide;
                inlier_cand = inlier_w;
                rmse_cand   = rmse_w;
            end

            if (inlier_cand / max(0.015, rmse_cand)) > (inlier_best / max(0.015, rmse_best))
                rel_best    = rel_cand;
                inlier_best = inlier_cand;
                rmse_best   = rmse_cand;
                fprintf('  [RECOVERY SCAN-TO-MAP] Riaggancio ad ampio raggio attivato (Inlier=%.0f%%)\n', ...
                        100*inlier_best);
            end
        end

        dx_corr  = rel_best(1);
        dy_corr  = rel_best(2);
        dth_corr = rel_best(3);

        % 6. Gate di validazione geometrica basato sulla reale sovrapposizione coi muri
        max_trans_allowed = 0.75;
        if is_wide_search
            max_trans_allowed = 1.20;
        end
        if hypot(dx_corr, dy_corr) > max_trans_allowed || inlier_best < 0.38
            fprintf('  [SCAN-TO-MAP] Match scartato (corr=%.2fm, %.1f deg, inlier=%.0f%%)\n', ...
                    hypot(dx_corr, dy_corr), rad2deg(dth_corr), 100*inlier_best);
            return;
        end

        % 7. Proiezione della correzione relativa nel frame di riferimento globale
        th0        = guessPose(3);
        estPose    = zeros(3,1);
        estPose(1) = guessPose(1) + dx_corr*cos(th0) - dy_corr*sin(th0);
        estPose(2) = guessPose(2) + dx_corr*sin(th0) + dy_corr*cos(th0);
        estPose(3) = wrapToPi(th0 + dth_corr);

        match_info.inlier_ratio = inlier_best;
        match_info.rmse         = rmse_best;
        ICP_LOG = [ICP_LOG; hypot(dx_corr, dy_corr), rad2deg(dth_corr)];
        success = true;
    catch ME
        fprintf('  [SCAN-TO-MAP WARN] %s\n', ME.message);
    end
end

% =========================================================================
% VALUTAZIONE GEOMETRICA VETTORIZZATA DELLA SOVRAPPOSIZIONE SCAN-MURI REALI
% =========================================================================
function [inlier_ratio, rmse] = valutaQualitaScanToMap(scan, ref_pts, relPose)
    sc_t = transformScan(scan, relPose);
    pts  = sc_t.Cartesian;
    if isempty(pts) || isempty(ref_pts)
        inlier_ratio = 0.0;
        rmse         = 1.0;
        return;
    end
    % Calcolo vettorizzato della distanza euclidea minima punto-muro
    dx     = pts(:,1) - ref_pts(:,1)';
    dy     = pts(:,2) - ref_pts(:,2)';
    min_d2 = min(dx.^2 + dy.^2, [], 2);
    min_d  = sqrt(min_d2);

    inlier_ratio = mean(min_d < 0.10);
    inlier_mask  = min_d < 0.15;
    if any(inlier_mask)
        rmse = sqrt(mean(min_d2(inlier_mask)));
    else
        rmse = median(min_d);
    end
end

% =========================================================================
% AGGIORNAMENTO GRAFICO REAL-TIME
% =========================================================================
function aggiornaGrafica(scan)
    global ROBOT_POSE h_robot h_ellipse h_map_points h_safety_cone EKF
    if ~ishandle(h_robot), return; end

    set(h_robot, 'XData', ROBOT_POSE(1), 'YData', ROBOT_POSE(2), ...
                 'UData', cos(ROBOT_POSE(3))*0.3, 'VData', sin(ROBOT_POSE(3))*0.3);

    [V, D] = eig(EKF.P(1:2, 1:2));
    t_ell  = linspace(0, 2*pi, 50);
    xy     = 2 * V * sqrt(abs(D)) * [cos(t_ell); sin(t_ell)] + ROBOT_POSE(1:2);
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

% =========================================================================
% ACQUISIZIONE SCANSIONE ADATTIVA (COMPATIBILE CON VECCHIO E NUOVO SERVER)
% =========================================================================
function scan = catturaScanAdattivo(u_rx, u_tx, ip)
    scan = catturaScanConFlush(u_rx, u_tx, ip, 3);
    if isempty(scan) || size(scan.Cartesian, 1) < 60
        fprintf('  [SCAN ADATTIVO] Pochi punti con 3 giri, riprovo con 5 giri...\n');
        scan = catturaScanConFlush(u_rx, u_tx, ip, 5);
    end
end

function scan = catturaScanConFlush(u_rx, u_tx, ip, n_giri)
    if nargin < 4
        n_giri = 5;
    end

    flushUDP(u_rx);
    write(u_tx, sprintf("SCAN,%d", n_giri), "string", ip, 5005);

    pts  = [];
    % Timeout ampio (min 10s, 14s per 5 giri) per compatibilita' totale anche col vecchio script Raspberry,
    % ma uscita istantanea (< 0.4s) appena arriva il pacchetto "DONE"
    T    = max(10.0, 3.0 + 2.2 * n_giri);
    t0   = tic;
    done = false;

    while toc(t0) < T
        if u_rx.NumDatagramsAvailable > 0
            pkts = read(u_rx, u_rx.NumDatagramsAvailable, "string");
            for i = 1:numel(pkts)
                s = strtrim(pkts(i).Data);
                if strcmpi(s, "DONE")
                    done = true;
                    break;
                elseif strcmpi(s, "EMG")
                    continue;
                end
                % Decodifica vettorizzata C-level (sscanf)
                s_clean = strrep(char(s), ';', ' ');
                vals    = sscanf(s_clean, '%f,%f');
                n_pairs = floor(numel(vals) / 2);
                if n_pairs > 0
                    pts = [pts; reshape(vals(1:2*n_pairs), 2, n_pairs)']; %#ok<AGROW>
                end
            end
        end
        if done, break; end
        pause(0.003);
    end

    if ~done
        fprintf('  [WARN] Pacchetto DONE non ricevuto entro %.1fs.\n', T);
    end

    if size(pts, 1) > 20
        % Filtro distanza radiale [5cm, 6m]
        d   = hypot(pts(:,1), pts(:,2));
        pts = pts(d >= 0.05 & d <= 6.0, :);

        % Downsampling spaziale su griglia fine a 2 cm
        x_idx    = round(pts(:,1) / 0.02);
        y_idx    = round(pts(:,2) / 0.02);
        [~, uid] = unique([x_idx, y_idx], 'rows');
        pts      = pts(uid, :);

        fprintf('  [LiDAR] %d pt unici (%d giri).\n', size(pts, 1), n_giri);
        scan = lidarScan(pts);
    else
        fprintf('  [LiDAR] Punti insufficienti (%d). Scan scartato.\n', size(pts, 1));
        scan = [];
    end
end
