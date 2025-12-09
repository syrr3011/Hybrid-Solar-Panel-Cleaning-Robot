clear; clc; close all;

%% 상수
M = 0.18 * 0.1047;
v_robot = 0.2;

%% 변수 범위
r_min = 0.01; r_max = 0.05;
L_min = 0.20; L_max = 0.26;
w_min = 10 * 2*pi/60;
w_max = 21 * 2*pi/60;

%% 초기값
r = 0.045; L = 0.26; w = 1.0;

%% 하이퍼파라미터
eta = 1e-3;             
max_iter = 100000;      
tol_f = 1e-6;           
tol_x = 1e-5;           
tol_g = 1e-4;           
eta_min = 1e-5;         
eta_max = 5e-2;         

%% 정규화 기준 샘플
r_samp = linspace(r_min, r_max, 15);
L_samp = linspace(L_min, L_max, 15);
w_samp = linspace(w_min, w_max, 15);
H_vals = []; P_vals = []; W_vals = [];

for ri = r_samp
    for Li = L_samp
        for wi = w_samp
            v_tip = 2*pi*ri*wi;
            % 효율항에 L 감쇠항 추가 (1 + 12*L^2)
            H_vals(end+1) = (2*pi*ri*Li*abs(v_tip - v_robot)) / ...
                ((1 + (v_tip/v_robot)^2) * (1 + 12*Li^2));
            P_vals(end+1) = ri^3 * Li * wi;
            W_vals(end+1) = abs(4*v_robot*ri^2*Li - ri^3*Li*wi);
        end
    end
end

H_min = min(H_vals); H_max = max(H_vals);
P_min = min(P_vals); P_max = max(P_vals);
W_min = min(W_vals); W_max = max(W_vals);

normalize = @(x,xmin,xmax) (x - xmin) / (xmax - xmin);

%% 목적함수 정의
obj = @(r,L,w) 0.35 * ( normalize(r^3*L*w , P_min, P_max) ) ...
             + 0.35 * ( normalize(abs(4*v_robot*r^2*L - r^3*L*w), W_min, W_max) ) ...
             + 0.3  * ( 1 - normalize( ...
                (2*pi*r*L*abs(2*pi*r*w - v_robot)) / ...
                ((1 + (2*pi*r*w/v_robot)^2)*(1 + 12*L^2)), ...
                H_min, H_max));

%% 로그 초기화
J = []; r_log = []; L_log = []; w_log = []; eta_log = [];

%% --- 경사하강법 반복 ---
for i = 1:max_iter
    h = 1e-5;
    % 수치미분 (중심차분)
    df_dr = (obj(r+h,L,w) - obj(r-h,L,w)) / (2*h);
    df_dL = (obj(r,L+h,w) - obj(r,L-h,w)) / (2*h);
    df_dw = (obj(r,L,w+h) - obj(r,L,w-h)) / (2*h);
    grad_norm = sqrt(df_dr^2 + df_dL^2 + df_dw^2);
    
    % 변수 업데이트
    r_new = r - eta * df_dr;
    L_new = L - eta * df_dL;
    w_new = w - eta * df_dw;

    %% --- 제약조건 처리 ---
    % 기본 범위 제약
    r_new = max(min(r_new, r_max), r_min);
    L_new = max(min(L_new, L_max), L_min);
    w_new = max(min(w_new, w_max), w_min);

    % 완화된 비율 제약 (3.5r ≤ L ≤ 6.5r), 단 최소길이 0.22 고정
    if L_new < max(0.22, 3.5*r_new)
        L_new = max(0.22, 3.5*r_new);
    elseif L_new > 6.5*r_new
        L_new = 6.5*r_new;
    end

    %% --- 물리적 제약 ---
    % 끝단속도 제약 (v_tip ≤ 1.0 m/s)
    v_tip = 2*pi*r_new*w_new;
    if v_tip > 1.0
        w_new = 1.0 / (2*pi*r_new);
        fprintf('[제약] 끝단속도 초과 → w_new 조정 (%.4f rad/s)\n', w_new);
    end

    % 전력상한 제약 (P ≤ 12 W)
    rho = 0.18 * 0.1047; mu = 0.3; N = 10;
    P = 0.5*rho*pi*r_new^4*L_new*w_new^2 + mu*N*r_new*w_new;
    if P > 12
        w_new = w_new * sqrt(12 / P);
        fprintf('[제약] 전력 초과 → w_new 축소 (%.4f rad/s)\n', w_new);
    end

    %% 변화량 계산
    delta_x = sqrt((r_new - r)^2 + (L_new - L)^2 + (w_new - w)^2);

    %% 업데이트
    r = r_new; L = L_new; w = w_new;
    J(i) = obj(r,L,w);
    r_log(i)=r; L_log(i)=L; w_log(i)=w; eta_log(i)=eta;

    %% --- 적응형 학습률 ---
    if i>1
        if J(i) > J(i-1)
            eta = max(eta*0.5, eta_min);
        else
            eta = min(eta*1.05, eta_max);
        end
    end

    %% --- 수렴 조건 ---
    if i>1
        delta_f = abs(J(i)-J(i-1));
        if (delta_f<tol_f)||(delta_x<tol_x)||(grad_norm<tol_g)
            fprintf('수렴 조건 충족 (%d iter)\n',i);
            break;
        end
    end
end

%% --- 결과 출력 ---
fprintf('\n===== 최적화 결과 =====\n');
fprintf('r = %.4f m\n', r);
fprintf('L = %.4f m\n', L);
fprintf('w = %.4f rad/s (%.2f rpm)\n', w, w*60/(2*pi));
fprintf('f = %.6f\n', J(i));

%% --- 결과 시각화 ---
figure;
subplot(1,2,1)
plot(J, 'LineWidth', 1.8);
xlabel('반복 횟수'); ylabel('목적함수 f');
title('수렴 곡선'); grid on;

subplot(1,2,2)
plot(eta_log, 'LineWidth', 1.5);
xlabel('반복 횟수'); ylabel('학습률(eta)');
title('적응형 학습률 변화'); grid on;

figure;
plot3(r_log, L_log, w_log, '.-','LineWidth',1.5);
xlabel('r [m]'); ylabel('L [m]'); zlabel('w [rad/s]');
title('경사하강법 탐색 경로 (r-L-w)');
grid on;
