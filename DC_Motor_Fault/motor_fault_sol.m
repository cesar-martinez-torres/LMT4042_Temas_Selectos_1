%% DC Motor Fault Detection: Param Change at 5 s (All HIs + Saturation)
% Signals used: motor current and speed error under PI control.
% Detection: 3σ thresholds with persistence and specific rules per fault type.
% 0–5 s: par1/ctl1 (healthy, for thresholds)
% 5–30 s: par2/ctl2 (new params applied)

clear; clc; close all;

%% 1) Motor and Control Parameters (EDITABLE: healthy vs new params after 5 s)
% Healthy (0–5 s)
par1.R    = 1.2;      % Ohm
par1.L    = 0.002;    % H
par1.Ke   = 0.03;     % Vs/rad
par1.Kt   = 0.03;     % N·m/A
par1.J    = 2.5e-4;   % kg·m^2
par1.b0   = 6e-5;     % N·m·s/rad (viscous friction)
par1.Vlim = 6;        % V (driver limit)

ctl1.Kp = 0.08;
ctl1.Ki = 2.5;
ctl1.antiwindup = true;

% New parameters (5–30 s)  << EDIT aquí para “crear la falla” cambiando parámetros
par2.R    = 1.2;      % Ohm
par2.L    = 0.002;    % H
par2.Ke   = 0.03;     % Vs/rad
par2.Kt   = 0.03;     % N·m/A
par2.J    = 2.5e-4;   % kg·m^2
par2.b0   = 6e-5;     % N·m·s/rad
par2.Vlim = 6;        % V

ctl2.Kp = 0.08;
ctl2.Ki = 2.5;
ctl2.antiwindup = true;

% Speed reference (rad/s) ~ 300 rpm ~ 31.4 rad/s
w_ref = 35;  % constante

% Carga por segmento (puedes variar sólo la carga si quieres)
tau_load_healthy = 0.002;  % Nm (0–5 s)
tau_load_new     = 0.050;  % Nm (5–30 s)

% Fracción para considerar saturación del control (HI auxiliar)
sat_frac = 0.95;

%% 2) Time Scenario -> Healthy 0–5 s + New Params 5–30 s
fs   = 1000;              % Hz simulation
Tend = 30;                % total seconds
dt   = 1/fs;

% Two segments: [0,5] uses par1/ctl1; [5,30] uses par2/ctl2
seg = struct( ...
    't0',       [0 5], ...
    't1',       [5 30], ...
    'tau_load', [tau_load_healthy tau_load_new] ...
);

%% 3) Segment Simulation (Explicit Euler + PI) with per-segment params
x = [0; 0]; % [omega; i]
PI_state = 0;

tout = []; xout = []; uout = []; ierr = [];
i_torque = []; tau_load_t = []; b_t = []; vlim_t = [];

for k = 1:numel(seg.t0)
    % === Selección de parámetros por segmento ===
    if k==1
        p = par1; c = ctl1; 
    else
        p = par2; c = ctl2;
    end

    % Print de verificación (una vez por segmento)
    fprintf('\nSEGMENT %d: R=%.3f, L=%.4f, Ke=%.3f, Kt=%.3f, J=%.1e, b0=%.2e, Vlim=%.2f, Kp=%.3f, Ki=%.3f\n', ...
        k, p.R, p.L, p.Ke, p.Kt, p.J, p.b0, p.Vlim, c.Kp, c.Ki);

    tk = (seg.t0(k):dt:seg.t1(k))';
    if tk(end) < seg.t1(k), tk(end+1) = seg.t1(k); end

    tauL = seg.tau_load(k);
    bnow = p.b0;

    for ii = 1:numel(tk)
        % --- Control PI con los parámetros del segmento (c.Kp, c.Ki, p.Vlim) ---
        e       = w_ref - x(1);          
        PI_state= PI_state + c.Ki*dt*e;
        u_PI    = c.Kp*e + PI_state;
        V_cmd   = max(min(u_PI, p.Vlim), -p.Vlim);
        if c.antiwindup && (V_cmd ~= u_PI)
            PI_state = PI_state - (u_PI - V_cmd);
        end

        % --- Dinámica del motor con parámetros del segmento (p.*) ---
        di = (V_cmd - p.R*x(2) - p.Ke*x(1))/p.L;
        dw = (p.Kt*x(2) - bnow*x(1) - tauL)/p.J;
        x  = x + dt*[dw; di];

        % Logging
        tout       = [tout; tk(ii)];
        xout       = [xout; x'];
        uout       = [uout; V_cmd];
        ierr       = [ierr; e];
        i_torque   = [i_torque; p.Kt*x(2)];
        tau_load_t = [tau_load_t; tauL];
        b_t        = [b_t; bnow];
        vlim_t     = [vlim_t; p.Vlim];   % importante para sat_limit por ventana
    end
end

% Final signals
omega = xout(:,1);         % rad/s
i_m    = xout(:,2);        % A
err_w  = ierr;             % rad/s
V_cmd  = uout;             % V

%% 4) Windowed Condition Indicators (compute per-window saturation limit too)
win_s = 0.25;             % 250 ms
hop_s = 0.10;             % 100 ms
Nw = round(win_s*fs);
Nh = round(hop_s*fs);
idx_st = 1:Nh:(numel(tout)-Nw+1);

time_v = zeros(numel(idx_st),1);
rmsI   = zeros(numel(idx_st),1);      % RMS current
meanI  = zeros(numel(idx_st),1);      % mean current
rmsErr = zeros(numel(idx_st),1);      % RMS speed error
meanW  = zeros(numel(idx_st),1);      % mean speed
rmsW   = zeros(numel(idx_st),1);      % RMS speed
vcmd_winmean = zeros(numel(idx_st),1);% windowed mean(V_cmd)
sat_limit_vec = zeros(numel(idx_st),1);% per-window saturation limit (depends on Vlim)

for i = 1:numel(idx_st)
    idxs = idx_st(i):idx_st(i)+Nw-1;
    segI = i_m(idxs);
    segE = err_w(idxs);
    segW = omega(idxs);
    segU = V_cmd(idxs);

    rmsI(i)   = rms(segI);
    meanI(i)  = mean(segI);
    rmsErr(i) = rms(segE);
    meanW(i)  = mean(segW);
    rmsW(i)   = rms(segW);
    vcmd_winmean(i) = mean(segU);
    time_v(i) = tout(idxs(1) + floor(Nw/2));

    % Vlim de la ventana (centro) -> límite de saturación por ventana
    vlim_win = vlim_t(idxs(1) + floor(Nw/2));
    sat_limit_vec(i) = sat_frac * vlim_win;
end

%% 5) Thresholds from Healthy Segment (0–5 s)  -- computed from par1/ctl1 segment
healthy_mask = time_v < 5;
mu_rmsI  = mean(rmsI(healthy_mask));   sd_rmsI  = std(rmsI(healthy_mask));
mu_meanI = mean(meanI(healthy_mask));  sd_meanI = std(meanI(healthy_mask));
mu_rmsE  = mean(rmsErr(healthy_mask)); sd_rmsE  = std(rmsErr(healthy_mask));
mu_meanW = mean(meanW(healthy_mask));  sd_meanW = std(meanW(healthy_mask));
mu_rmsW  = mean(rmsW(healthy_mask));   sd_rmsW  = std(rmsW(healthy_mask));

k = 3; % 3-sigma
thr_rmsI_high  = mu_rmsI  + k*sd_rmsI;            % high current
thr_meanI_high = mu_meanI + k*sd_meanI;           % high mean current
thr_rmsE_high  = mu_rmsE  + k*sd_rmsE;            % high error
thr_meanW_low  = max(mu_meanW - k*sd_meanW, 0);   % low speed
thr_rmsW_low   = max(mu_rmsW - k*sd_rmsW, 0);     % low RMS(speed)

%% 6) Fault Detection Logic (rules)
% Overload: High I + high error, without extreme speed drop
% Friction: High I + moderate speed drop, without sustained high error
% Stall: High I + very low speed AND low RMS speed AND control near saturation
overI     = (rmsI > thr_rmsI_high) | (meanI > thr_meanI_high);
overErr   = (rmsErr > thr_rmsE_high);
lowW      = (meanW < thr_meanW_low);
lowRMSW   = (rmsW  < thr_rmsW_low);
sat       = abs(vcmd_winmean) > sat_limit_vec;   % per-window saturation threshold

raw_overload  = overI & overErr & ~lowW;
raw_friction  = overI & ~overErr & (meanW < (mu_meanW - 0.5*sd_meanW));
raw_stall     = overI & lowW & lowRMSW & sat;

% Persistence (M consecutive windows)
M = 3;
persist = @(sig) arrayfun(@(i) i>=M && all(sig(i-M+1:i)), 1:numel(sig))';

alarm_overload = persist(raw_overload);
alarm_friction = persist(raw_friction);
alarm_stall    = persist(raw_stall);

% Detection times (first occurrence)
t_over  = first_time(time_v, alarm_overload);
t_fric  = first_time(time_v, alarm_friction);
t_stall = first_time(time_v, alarm_stall);

%% 7) Plots 
figure('Name','Motor Signals','Position',[80 60 1100 800]);

subplot(4,1,1);
plot(tout, omega, 'LineWidth',1); grid on;
ylabel('\omega (rad/s)'); title('Speed (Healthy 0–5 s; New Parameters 5–30 s)');
xregion([5 30], 'New Params');
if ~isnan(t_over),  xline(t_over,  'g--', 'Detect: Overload'); end
if ~isnan(t_fric),  xline(t_fric,  'm--', 'Detect: Friction'); end
if ~isnan(t_stall), xline(t_stall, 'r--', 'Detect: Stall'); end

subplot(4,1,2);
plot(tout, i_m, 'LineWidth',1); grid on;
ylabel('i (A)'); title('Current'); 

subplot(4,1,3);
plot(tout, err_w, 'LineWidth',1); grid on;
ylabel('e_\omega (rad/s)'); title('Speed Error'); 

subplot(4,1,4);
plot(tout, V_cmd, 'LineWidth',1); hold on; grid on;
% Límites de saturación de referencia por tramo (visual)
yline( sat_frac*par1.Vlim, '--', sprintf('\\pm%.2f V (0–5 s)', sat_frac*par1.Vlim));
yline(-sat_frac*par1.Vlim, '--');
yline( sat_frac*par2.Vlim, ':',  sprintf('\\pm%.2f V (5–30 s)', sat_frac*par2.Vlim));
yline(-sat_frac*par2.Vlim, ':');
ylabel('V_{cmd} (V)'); xlabel('Time (s)'); title('Control Action and Saturation Levels');

% Show ALL HIs in one figure (5 panels)
figure('Name','All Health Indicators (HIs)','Position',[120 60 1100 900]);
tiledlayout(5,1);

nexttile;
plot(time_v, rmsI, 'LineWidth',1); hold on; 
yline(thr_rmsI_high,'--','thr RMS i'); grid on;
ylabel('RMS(i)'); title('HI1: RMS current');

nexttile;
plot(time_v, meanI, 'LineWidth',1); hold on; 
yline(thr_meanI_high,'--','thr mean i'); grid on;
ylabel('mean(i)'); title('HI2: Mean current');

nexttile;
plot(time_v, rmsErr, 'LineWidth',1); hold on; 
yline(thr_rmsE_high,'--','thr RMS err'); grid on;
ylabel('RMS(err)'); title('HI3: RMS speed error');

nexttile;
plot(time_v, meanW, 'LineWidth',1); hold on; 
yline(thr_meanW_low,'--','thr \omega low'); grid on;
ylabel('mean(\omega)'); title('HI4: Mean speed');

nexttile;
plot(time_v, rmsW, 'LineWidth',1); hold on; 
yline(thr_rmsW_low,'--','thr RMS(\omega) low'); grid on;
ylabel('RMS(\omega)'); xlabel('Time (s)'); title('HI5: RMS speed');

% Control Saturation Check (windowed mean of Vcmd vs per-window limit)
figure('Name','Control Saturation Check','Position',[160 80 900 320]);
plot(time_v, vcmd_winmean, 'LineWidth',1); hold on; grid on;
plot(time_v,  sat_limit_vec,'k--','LineWidth',1);  % show per-window limit
plot(time_v, -sat_limit_vec,'k--','LineWidth',1);
xlabel('Time (s)'); ylabel('mean(V_{cmd}) per window (V)');
title('Windowed mean of V_{cmd} vs per-window saturation limit');

%% 8) Report
fprintf('\n--- REPORT ---\n');
fprintf('Scenario: Healthy (par1/ctl1) 0–5 s; New Params (par2/ctl2) 5–30 s\n');
fprintf('Healthy load tau=%.3f Nm; New load tau=%.3f Nm\n', tau_load_healthy, tau_load_new);
fprintf(['Thresholds (3σ) from 0–5 s (healthy):\n' ...
         '  thr RMS(i)=%.3f A, thr mean(i)=%.3f A, thr RMS(err)=%.3f rad/s,\n' ...
         '  thr mean(omega)_low=%.2f rad/s, thr RMS(omega)_low=%.3f rad/s\n'], ...
    thr_rmsI_high, thr_meanI_high, thr_rmsE_high, thr_meanW_low, thr_rmsW_low);

fprintf('\nDetections (5–30 s):\n');
prn('Overload', t_over, [5 30]);
prn('Excessive friction', t_fric, [5 30]);
prn('Stall', t_stall, [5 30]);

%% ==== Auxiliary Functions ====
function tdet = first_time(t, alarm)
    idx = find(alarm,1,'first');
    if isempty(idx), tdet = NaN; else, tdet = t(idx); end
end

function prn(name, tdet, trange)
    if isnan(tdet)
        fprintf('  %-18s: NOT detected\n', name);
    else
        inwin = tdet>=trange(1) && tdet<=trange(2);
        fprintf('  %-18s: t = %.2f s %s\n', name, tdet, tern(inwin,'(in window)','(out of window)'));
    end
end

function s = tern(cond, a, b)
    if cond, s=a; else, s=b; end
end

function xregion(tr, labeltxt)
    yl = ylim; hold on;
    patch([tr(1) tr(2) tr(2) tr(1)], [yl[1] yl[1] yl[2] yl[2]], [0.9 0.9 0.95], ...
        'EdgeColor','none','FaceAlpha',0.35);
    text(mean(tr), yl(2)-0.08*(yl(2)-yl(1)), labeltxt, ...
        'HorizontalAlignment','center');
    uistack(findobj(gca,'Type','line'),'top'); % keep curve on top
end
