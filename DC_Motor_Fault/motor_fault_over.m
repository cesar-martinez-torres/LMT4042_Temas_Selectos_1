%% DC Motor Fault Detection: Overload only (Healthy 0–5 s; Overload 5–30 s)
% Signals used: motor current and speed error under PI control.
% Detection: 3σ-like thresholds with persistence and specific rules per fault type.

clear; clc; close all;

%% 1) Motor and Control Parameters
%   V = R*i + L*di/dt + K_e*omega
%   J*domega/dt = K_t*i - b*omega - tau_load
par.R   = 1.2;        % Ohm
par.L   = 0.002;      % H
par.Ke  = 0.03;       % Vs/rad
par.Kt  = 0.03;       % N·m/A
par.J   = 2.5e-4;     % kg·m^2
par.b0  = 6e-5;       % N·m·s/rad (healthy viscous friction)
par.Vlim= 6;          % V (typical max voltage in small robots)

% PI speed control
ctl.Kp = 0.08;
ctl.Ki = 2.5;
ctl.antiwindup = true;

% Speed reference (rad/s) ~ 300 rpm ~ 31.4 rad/s
w_ref = 35;  % constant

%% 2) Time Scenario -> Healthy 0–5 s; Overload 5–30 s
fs   = 1000;              % Hz simulation
Tend = 30;                % total seconds
dt   = 1/fs;

% Two segments: healthy (0–5 s) and overload (5–30 s)
seg = struct( ...
    't0',       [0     5], ...
    't1',       [5     30], ...
    'tau_load', [0.002 0.050], ...     % Nm (overload chosen to raise I & error)
    'b',        [par.b0 par.b0] ...    % keep viscous friction nominal
);

%% 3) Segment Simulation (Explicit Euler + PI)
x = [0; 0]; % [omega; i]
PI_state = 0;

tout = []; xout = []; uout = []; ierr = [];
i_torque = []; tau_load_t = []; b_t = [];

for k = 1:numel(seg.t0)
    tk = (seg.t0(k):dt:seg.t1(k))';
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

for i = 1:numel(idx_st)
    idxs = idx_st(i):idx_st(i)+Nw-1;
    segI = i_m(idxs);
    segE = err_w(idxs);
    segW = omega(idxs);

    rmsI(i)   = rms(segI);
    meanI(i)  = mean(segI);
    rmsErr(i) = rms(segE);
    meanW(i)  = mean(segW);
    rmsW(i)   = rms(segW);
    time_v(i) = tout(idxs(1) + floor(Nw/2));
end

%% 5) Thresholds from Healthy Segment (0–5 s)
healthy_mask = time_v < 5;

mu_rmsI  = mean(rmsI(healthy_mask));   sd_rmsI  = std(rmsI(healthy_mask));
mu_meanI = mean(meanI(healthy_mask));  sd_meanI = std(meanI(healthy_mask));
mu_rmsE  = mean(rmsErr(healthy_mask)); sd_rmsE  = std(rmsErr(healthy_mask));
mu_meanW = mean(meanW(healthy_mask));  sd_meanW = std(meanW(healthy_mask));

% Separate k's to shape each threshold class
k_I    = 3.0;   % current thresholds (robust)
k_E    = 2.2;   % error threshold (more sensitive to guarantee overErr)
k_Wlow = 5.0;   % low-speed threshold (lenient -> avoid lowW=true)

thr_rmsI_high  = mu_rmsI  + k_I*sd_rmsI;            % high current
thr_meanI_high = mu_meanI + k_I*sd_meanI;           % high mean current
thr_rmsE_high  = mu_rmsE  + k_E*sd_rmsE;            % high error
thr_meanW_low  = max(mu_meanW - k_Wlow*sd_meanW, 0);% low speed (very low)

%% 6) Fault Detection Logic (rules)
% Overload: High I + high error, without extreme speed drop
% Friction: High I + moderate speed drop, without sustained high error
% Stall: High I + very low speed
overI   = (rmsI > thr_rmsI_high) | (meanI > thr_meanI_high);
overErr = (rmsErr > thr_rmsE_high);
lowW    = (meanW < thr_meanW_low);

raw_overload  = overI & overErr & ~lowW;
raw_friction  = overI & ~overErr & (meanW < (mu_meanW - 0.5*sd_meanW));
raw_stall     = overI & lowW;

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

%% 7) Plots  (Healthy 0–5 s; shaded Overload 5–30 s)
figure('Name','Motor Signals','Position',[80 60 1100 750]);

subplot(4,1,1);
plot(tout, omega, 'LineWidth',1); grid on;
ylabel('\omega (rad/s)'); title('Speed: Overload from 5 s to 30 s');
xregion([5 30], 'Overload');
if ~isnan(t_over),  xline(t_over, 'g--', 'Detect: Overload'); end

subplot(4,1,2);
plot(tout, i_m, 'LineWidth',1); grid on;
ylabel('i (A)'); title('Current'); 

subplot(4,1,3);
plot(tout, err_w, 'LineWidth',1); grid on;
ylabel('e_\omega (rad/s)'); title('Speed Error'); 

subplot(4,1,4);
plot(tout, V_cmd, 'LineWidth',1); grid on;
ylabel('V_{cmd} (V)'); xlabel('Time (s)'); title('Control Action (±V_{lim})');

figure('Name','Indicators and Thresholds','Position',[120 100 1100 700]);
subplot(3,1,1);
plot(time_v, rmsI,'LineWidth',1); hold on; yline(thr_rmsI_high,'--','thr RMS i'); grid on;
ylabel('RMS(i)'); title('Windowed Indicators');
subplot(3,1,2);
plot(time_v, rmsErr,'LineWidth',1); hold on; yline(thr_rmsE_high,'--','thr RMS err'); grid on;
ylabel('RMS(err)');
subplot(3,1,3);
plot(time_v, meanW,'LineWidth',1); hold on; yline(thr_meanW_low,'--','thr \omega low'); grid on;
ylabel('mean(\omega)'); xlabel('Time (s)');

%% 8) Report
fprintf('\n--- REPORT ---\n');
fprintf('Thresholds calculated in 0–5 s (healthy)\n');
fprintf('  thr RMS(i)=%.3f A, thr mean(i)=%.3f A, thr RMS(err)=%.3f rad/s, thr mean(omega)_low=%.2f rad/s\n', ...
    thr_rmsI_high, thr_meanI_high, thr_rmsE_high, thr_meanW_low);

fprintf('\nDetections:\n');
prn('Overload', t_over, [5 30]);
prn('Excessive friction', t_fric, [10 30]); % should be NOT detected
prn('Stall', t_stall, [15 30]);             % should be NOT detected

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
