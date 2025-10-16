%% DC Motor Fault Detection: Overload, Excessive Friction, and Stall (Nominal + All HIs + Saturation)
% Signals used: motor current and speed error under PI control.
% Detection: 3σ thresholds with persistence and specific rules per fault type.

clear; clc; close all;

%% 1) Motor and Control Parameters
% Model:
%   V = R*i + L*di/dt + K_e*omega
%   J*domega/dt = K_t*i - b*omega - tau_load
par.R   = 1.2;        % Ohm
par.L   = 0.002;      % H
par.Ke  = 0.03;       % Vs/rad
par.Kt  = 0.03;       % N·m/A
par.J   = 2.5e-4;     % kg·m^2
par.b0  = 6e-5;       % N·m·s/rad (healthy viscous friction)
par.Vlim= 6;          % V (typical max voltage in small robots)

% Basic PI speed control
ctl.Kp = 0.08;
ctl.Ki = 2.5;
ctl.antiwindup = true;

% Speed reference (rad/s) ~ 300 rpm ~ 31.4 rad/s
w_ref = 35;  % constant; you can change it

%% 2) Time Scenario -> Nominal 30 s (NO faults)
fs   = 1000;              % Hz simulation
Tend = 30;                % total seconds
dt   = 1/fs;

% Single healthy segment for the whole run (0–30 s)
seg = struct( ...
    't0',       0, ...
    't1',       30, ...
    'tau_load', 0.002, ...      % nominal load torque
    'b',        par.b0 ...      % nominal viscous friction
);

%% 3) Segment Simulation (Explicit Euler + PI)
x = [0; 0]; % [omega; i]
PI_state = 0;

tout = []; xout = []; uout = []; ierr = [];
i_torque = []; tau_load_t = []; b_t = [];

for k = 1:numel(seg.t0)
    tk = (seg.t0(k):dt:seg.t1(k))';
    % Ensure last exact point of segment
    if tk(end) < seg.t1(k)
        tk(end+1) = seg.t1(k);
    end

    tauL = seg.tau_load(k);
    bnow = seg.b(k);

    for ii = 1:numel(tk)
        % PI Control
        e     = w_ref - x(1);          % speed error
        PI_state = PI_state + ctl.Ki*dt*e;
        u_PI  = ctl.Kp*e + PI_state;
        V_cmd = max(min(u_PI, par.Vlim), -par.Vlim);
        if ctl.antiwindup && (V_cmd ~= u_PI)
            PI_state = PI_state - (u_PI - V_cmd);
        end

        % Dynamics (Explicit Euler)
        di = (V_cmd - par.R*x(2) - par.Ke*x(1))/par.L;
        dw = (par.Kt*x(2) - bnow*x(1) - tauL)/par.J;
        x  = x + dt*[dw; di];

        % Logging
        tout       = [tout; tk(ii)];
        xout       = [xout; x'];
        uout       = [uout; V_cmd];
        ierr       = [ierr; e];
        i_torque   = [i_torque; par.Kt*x(2)];
        tau_load_t = [tau_load_t; tauL];
        b_t        = [b_t; bnow];
    end
end

% Final signals
omega = xout(:,1);         % rad/s
i_m    = xout(:,2);        % A
err_w  = ierr;             % rad/s
V_cmd  = uout;             % V

%% 4) Windowed Condition Indicators
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
vcmd_winmean = zeros(numel(idx_st),1);% mean control voltage (per window)  <-- NEW

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
end

%% 5) Thresholds from Healthy Segment (0–5 s)
healthy_mask = time_v < 5;
mu_rmsI  = mean(rmsI(healthy_mask));   sd_rmsI  = std(rmsI(healthy_mask));
mu_meanI = mean(meanI(healthy_mask));  sd_meanI = std(meanI(healthy_mask));
mu_rmsE  = mean(rmsErr(healthy_mask)); sd_rmsE  = std(rmsErr(healthy_mask));
mu_meanW = mean(meanW(healthy_mask));  sd_meanW = std(meanW(healthy_mask));
mu_rmsW  = mean(rmsW(healthy_mask));   sd_rmsW  = std(rmsW(healthy_mask));     % NEW

k = 3; % 3-sigma
thr_rmsI_high  = mu_rmsI  + k*sd_rmsI;            % high current
thr_meanI_high = mu_meanI + k*sd_meanI;           % high mean current
thr_rmsE_high  = mu_rmsE  + k*sd_rmsE;            % high error
thr_meanW_low  = max(mu_meanW - k*sd_meanW, 0);   % low speed
thr_rmsW_low   = max(mu_rmsW - k*sd_rmsW, 0);     % low RMS(speed)            % NEW

% Control saturation reference (not an HI but used in stall logic & plots)
sat_limit = 0.95*par.Vlim;                                                   % NEW

%% 6) Fault Detection Logic (rules)
% Overload: High I + high error, without extreme speed drop
% Friction: High I + moderate speed drop, without sustained high error
% Stall: High I + very low speed AND low RMS speed AND control near saturation
overI     = (rmsI > thr_rmsI_high) | (meanI > thr_meanI_high);
overErr   = (rmsErr > thr_rmsE_high);
lowW      = (meanW < thr_meanW_low);
lowRMSW   = (rmsW  < thr_rmsW_low);                    
sat       = abs(vcmd_winmean) > sat_limit;             

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
figure('Name','Motor Signals','Position',[80 60 1100 780]);

subplot(4,1,1);
plot(tout, omega, 'LineWidth',1); grid on;
ylabel('\omega (rad/s)'); title('Speed (Nominal Run, 0–30 s)');

subplot(4,1,2);
plot(tout, i_m, 'LineWidth',1); grid on;
ylabel('i (A)'); title('Current'); 

subplot(4,1,3);
plot(tout, err_w, 'LineWidth',1); grid on;
ylabel('e_\omega (rad/s)'); title('Speed Error'); 

subplot(4,1,4);
plot(tout, V_cmd, 'LineWidth',1); hold on; grid on;
yline( sat_limit, '--', '+0.95 V_{lim}');         % <-- show saturation limits
yline(-sat_limit, '--', '-0.95 V_{lim}');
ylabel('V_{cmd} (V)'); xlabel('Time (s)'); title('Control Action (±V_{lim})');

% Show ALL HIs in one figure (5 panels), with thresholds incl. rmsW
figure('Name','All Health Indicators (HIs)','Position',[120 60 1100 900]);
tiledlayout(5,1);

% HI #1: RMS current
nexttile;
plot(time_v, rmsI, 'LineWidth',1); hold on; 
yline(thr_rmsI_high,'--','thr RMS i'); grid on;
ylabel('RMS(i)'); title('HI1: RMS current');

% HI #2: Mean current
nexttile;
plot(time_v, meanI, 'LineWidth',1); hold on; 
yline(thr_meanI_high,'--','thr mean i'); grid on;
ylabel('mean(i)'); title('HI2: Mean current');

% HI #3: RMS speed error
nexttile;
plot(time_v, rmsErr, 'LineWidth',1); hold on; 
yline(thr_rmsE_high,'--','thr RMS err'); grid on;
ylabel('RMS(err)'); title('HI3: RMS speed error');

% HI #4: Mean speed
nexttile;
plot(time_v, meanW, 'LineWidth',1); hold on; 
yline(thr_meanW_low,'--','thr \omega low'); grid on;
ylabel('mean(\omega)'); title('HI4: Mean speed');

% HI #5: RMS speed  (with low threshold)
nexttile;
plot(time_v, rmsW, 'LineWidth',1); hold on; 
yline(thr_rmsW_low,'--','thr RMS(\omega) low'); grid on;
ylabel('RMS(\omega)'); xlabel('Time (s)'); title('HI5: RMS speed');

% Control Saturation Check 
figure('Name','Control Saturation Check','Position',[160 80 900 320]);         % NEW
plot(time_v, vcmd_winmean, 'LineWidth',1); hold on; grid on;
yline( sat_limit, '--', '+0.95 V_{lim}');
yline(-sat_limit, '--', '-0.95 V_{lim}');
xlabel('Time (s)'); ylabel('mean(V_{cmd}) per window (V)');
title('Windowed mean of V_{cmd} vs saturation limit');

%% 8) Report
fprintf('\n--- REPORT ---\n');
fprintf('Thresholds (3σ) calculated in 0–5 s (healthy)\n');
fprintf(['  thr RMS(i)=%.3f A, thr mean(i)=%.3f A, thr RMS(err)=%.3f rad/s, ' ...
         'thr mean(omega)_low=%.2f rad/s, thr RMS(omega)_low=%.3f rad/s, sat_limit=%.2f V\n'], ...
    thr_rmsI_high, thr_meanI_high, thr_rmsE_high, thr_meanW_low, thr_rmsW_low, sat_limit);

fprintf('\nDetections:\n');
prn('Overload', t_over, [5 10]);
prn('Excessive friction', t_fric, [10 15]);
prn('Stall', t_stall, [15 20]);

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
    patch([tr(1) tr(2) tr(2) tr(1)], [yl(1) yl(1) yl(2) yl(2)], [0.9 0.9 0.95], ...
        'EdgeColor','none','FaceAlpha',0.35);
    text(mean(tr), yl(2)-0.08*(yl(2)-yl(1)), labeltxt, ...
        'HorizontalAlignment','center');
    uistack(findobj(gca,'Type','line'),'top'); % keep curve on top
end
