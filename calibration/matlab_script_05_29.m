%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calibration Parameter (R_cnv) Estimation

%% 1. CSV 데이터 읽기
filename = 'checkboard_Points.csv';
T = readtable(filename, detectImportOptions(filename));

%% 2. 영상·카메라 파라미터
res_x = 640;  res_y = 360;
cx = res_x/2; cy = res_y/2;

fov_h = deg2rad(65.77);           % 가로 FOV
fov_v = deg2rad(38.10);           % 세로 FOV
fx    = (res_x/2)/tan(fov_h/2);   % pixel focal length
fy    = (res_y/2)/tan(fov_v/2);

h_cam = 0.145;                    % 카메라 높이[m]
roll_cam = deg2rad( 32);          % +32° : 카메라 X축 기준 하향

%% 3. 회전 행렬
% 3‑1) Body축 → (Pitch 0) Camera축 매핑
R_axes = [ 0  -1   0 ;            % X_body →  Z_cam
           0   0  -1 ;            % Y_body → -X_cam
           1   0   0 ];           % Z_body → -Y_cam
%       (Body→Camera, 열:Body 벡터를 행:Camera 축으로 변환)

% 3‑2) Camera X축(+우) 기준 Roll(+20°)  (Camera 좌표계 회전)
c = cos(roll_cam);  s = sin(roll_cam);
R_roll = [ 1  0   0 ;
           0  c  -s ;
           0  s   c ];

% 3‑3) Body→Camera 전체 행렬 (축 변환 → Roll)
R_bc = R_roll * R_axes;           % Body → Camera
% Camera→Body는 R_cb = R_bc'

%% 4. Body 점 → Camera 점
Xb = T.X / 100;                % Body +X (전방)
Yb = T.Y / 100;                % Body +Y (왼편)
Zb = zeros(height(T),1);          % 지면

P_body = [Xb, Yb, Zb].';          % 3×N
C_body = [0;0;h_cam];             % 카메라 중심 좌표(Body)

P_rel  = P_body - C_body;         % 카메라 원점으로 평행이동
P_cam  = R_bc * P_rel;            % Body → Camera

Xc =  P_cam(1,:);                 % Camera +X (우)
Yc =  P_cam(2,:);                 % Camera +Y (하)
Zc =  P_cam(3,:);                 % Camera +Z (전방)

%% 5. 핀홀 투영
in_front = Zc > 0;                % 전방에 있는 점
u_est = NaN(size(Zc));
v_est = NaN(size(Zc));
u_est(in_front) = cx + fx .* ( Xc(in_front) ./ Zc(in_front) );
v_est(in_front) = cy + fy .* ( Yc(in_front) ./ Zc(in_front) );

%% 6. 실측‑추정값 정리 및 Least-Squared Error (LSE) 보정 파라미터 도출
u_meas = T{:,1};
v_meas = T{:,2};

N = size(u_meas, 1);

% Homogeneous Coordinates 좌표 생성: meas - 측정값, est - 추정값
meas = [u_meas'; v_meas'; ones(1, N)];
est = [u_est; v_est; ones(1, N)];

% 추정값 -> 측정값으로 변환하는 LSE 보정 Matrix 도출
R_cnv = meas * pinv(est);

% 보정된 픽셀 데이터 생성
calib = R_cnv * est;
u_calib = calib(1, :);
v_calib = calib(2, :);


%% 7. 오차 계산
du = u_est.' - u_meas;
dv = v_est.' - v_meas;

duc = u_calib' - u_meas;
dvc = v_calib' - v_meas;

% Mean-Squared Error 계산
fprintf("MSE of Raw Pixel Points: %f\n", sqrt(sum(du.*du + dv.*dv)/N));
fprintf("MSE of Calibrated Pixel Points: %f\n", sqrt(sum(duc.*duc + dvc.*dvc)/N));

%% 8. 시각화 (흰색 배경)
figure;
imshow(ones(res_y,res_x,3));  hold on;
plot(u_meas, v_meas, 'bo', 'MarkerSize',5, 'DisplayName','Measured');
plot(u_est , v_est , 'r+', 'MarkerSize',5, 'LineWidth',1.5, 'DisplayName','Estimated');
plot(u_calib , v_calib , 'g*', 'MarkerSize',5, 'LineWidth',1.5, 'DisplayName','Calibrated');
set(gca,'YDir','reverse'); axis([0 res_x 0 res_y]);
xlabel('Pixel X'); ylabel('Pixel Y');
title('Measured vs Estimated Pixel Positions');
legend('Location','best');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Body Frame Estimation

%% 9. 측정‑>이론 픽셀 역보정  (R_cnv^{-1} 활용)  -----------------
R_inv = pinv(R_cnv);                 % R_cnv ≈ meas = R_cnv*est  ⇒  est ≈ R_inv*meas
meas_corr_h = R_inv * meas;          % 동차 좌표 (3×N)
u_corr = meas_corr_h(1,:)./meas_corr_h(3,:);   % 정규화
v_corr = meas_corr_h(2,:)./meas_corr_h(3,:);

% 결과 테이블에 보정픽셀 추가
Result.u_corr = u_corr.';
Result.v_corr = v_corr.';

%% 10. 역보정 픽셀로 바닥면 3‑D 위치 복원  -------------------------
% 10‑1) 역보정 픽셀 → 카메라 방향 벡터
dx_cam_c = (u_corr - cx)/fx;
dy_cam_c = (v_corr - cy)/fy;
dir_cam_c = [dx_cam_c ; dy_cam_c ; ones(1,N)];      % 3×N

% 10‑2) 카메라 → Body 방향 벡터
R_cb = R_bc.';                        % Camera → Body
dir_body_c = R_cb * dir_cam_c;        % 3×N

% 10‑3) 바닥면(Z=0) 교점 계산
dir_z_c = dir_body_c(3,:);            % Z 성분 (<0)
t_c = -h_cam ./ dir_z_c;              % 스칼라 교차 파라미터
Xb_pos_c = t_c.' .* dir_body_c(1,:).';  % Body‑X
Yb_pos_c = t_c.' .* dir_body_c(2,:).';  % Body‑Y
Zb_pos_c = zeros(N,1);                % 바닥

pos_body_corr = [Xb_pos_c, Yb_pos_c, Zb_pos_c];

dxp = Xb_pos_c - Xb;
dyp = Yb_pos_c - Yb;
% Mean-Squared Error 계산
fprintf("MSE of Raw Distance: %f\n", sqrt(sum(dxp.*dxp + dyp.*dyp)/N));

%% 11. 2‑D Floor‑Plane 비교 플롯 ----------------------------------
figure;
plot(Xb, Yb, 'bo', 'MarkerSize',6, 'DisplayName','Measured Floor (file)');
hold on;
plot(Xb_pos_c, Yb_pos_c, 'g*', 'MarkerSize',6, 'DisplayName','Estimated Floor (corrected)');
axis equal; grid on;
xlabel('X_b  [m]');  ylabel('Y_b  [m]');
title('Measured vs Corrected Floor Points  (Body Frame, Z=0)');
legend('Location','best');
%%npy 생성
addpath('/Users/gadeuk/Documents/MATLAB/npy-matlab-master/npy-matlab');
writeNPY(R_cnv, 'R_cnv.npy');