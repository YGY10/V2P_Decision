function [target_y, target_v, out_state, has_signal, danger_level, action, collision_info] = ...
    fcn(no_signal_zone_x, ego_y, lane0_y, lane1_y, lane2_y, last_target_y, ...
        obs_x_set_gcs, obs_y_set_gcs, obs_v_set_gcs, obs_heading_set_gcs, ...
        obs_ID_set, ego_x, ego_v, ego_heading, ego_yawrate, last_target_v, time)


%% ------------------------------------------------------
% Persistent Variables
%% ------------------------------------------------------
persistent first_run stored_target_y stored_target_v state block_until_time
action = 0;
collision_info.obs_id = int32(-1);
collision_info.time = -1;

if isempty(first_run)
    first_run = true;
    stored_target_y = 0;
    stored_target_v = 0;
    state = 0;
    block_until_time = 0;
end

%% Parameters
dt = 0.05;
recommond_speed = 15;
lane_width = 3.5;
max_acc = 2;
max_de_acc = -3.5;

ego_heading = wrapToPi(-deg2rad(ego_heading - 90));
lane_y_all = [lane0_y, lane1_y, lane2_y];

%% Signal availability
has_signal = (ego_x < no_signal_zone_x);

%% =============== CASE 1: LOST SIGNAL ===================
if ~has_signal
    stored_target_v = 0;
    stored_target_y = 0;
    state = 0;

    % Danger level 依旧保留
    danger_level = calc_lost_signal_danger( ...
        ego_x, ego_y, ego_heading, ...
        obs_x_set_gcs, obs_y_set_gcs, obs_v_set_gcs, obs_heading_set_gcs, ...
        obs_ID_set, lane_y_all, lane_width);

    % ⭐ 新增：调用 analyze_danger
    [danger_ns, tx_ns] = analyze_danger( ...
        ego_x, ego_y, ego_v, ego_heading, ...
        obs_x_set_gcs, obs_y_set_gcs, obs_v_set_gcs, ...
        obs_heading_set_gcs, obs_ID_set, lane_y_all, lane_width);

    % 找最危险 lane（TTC 最小）
    ttc_list = tx_ns(:,1);
    [min_ttc, idx_lane] = min(ttc_list);

    if min_ttc < 99
        collision_info.obs_id = int32(danger_ns(idx_lane,2));
        collision_info.time   = min_ttc;
    else
        collision_info.obs_id = int32(-1);
        collision_info.time   = 999;
    end

    % 输出保持
    target_v = stored_target_v;
    target_y = stored_target_y;
    out_state = state;
    action = 0;
    return;
end

%% =============== FIRST RUN ===================
if first_run
    [~, idx_lane] = min(abs(ego_y - lane_y_all));
    stored_target_y = lane_y_all(idx_lane);
    stored_target_v = recommond_speed;

    first_run = false;

    target_y = stored_target_y;
    target_v = stored_target_v;
    out_state = state;
    danger_level = 0;
    action = 0;
    return;
end

%% =============== NORMAL MODE ===================
stored_target_y = last_target_y;
stored_target_v = last_target_v;

%% Danger Analysis
[danger, tx_lane] = analyze_danger( ...
    ego_x, ego_y, ego_v, ego_heading, ...
    obs_x_set_gcs, obs_y_set_gcs, obs_v_set_gcs, obs_heading_set_gcs, obs_ID_set, ...
    lane_y_all, lane_width);

% Determine lane index (scalar)
lane_idx = 1;
min_val = abs(stored_target_y - lane_y_all(1));
for ii = 2:3
    if abs(stored_target_y - lane_y_all(ii)) < min_val
        min_val = abs(stored_target_y - lane_y_all(ii));
        lane_idx = ii;
    end
end

is_danger = danger(lane_idx, 1);
t_lane  = tx_lane(lane_idx, 1);
x_lane  = tx_lane(lane_idx, 2);
if is_danger
    collision_info.obs_id = int32(danger(lane_idx, 2));
    collision_info.time = t_lane;
end

%% --------------------- State Machine ---------------------
switch state

case 0   % KEEP
    action = 0;
    block_until_time = time;

    if is_danger
        state = 1;
    else
        stored_target_v = min(stored_target_v + max_acc*dt, recommond_speed);
    end

case 1   % BRAKE
    if is_danger
        d_brake = ego_v^2 / (2*abs(max_de_acc));
        dx_conflict = x_lane - ego_x;

        if d_brake < dx_conflict - 6
            stored_target_v = last_target_v + max_de_acc * dt;
            action = 2;

            obs_id = danger(lane_idx,2);
            idx_obs = find(obs_ID_set == obs_id, 1);
            if isempty(idx_obs), idx_obs = 1; end

            vy = obs_v_set_gcs(idx_obs) * sin(obs_heading_set_gcs(idx_obs));

            block_until_time = time + min(t_lane, (lane_width/2)/max(abs(vy),0.01));

        else
            % change lane
            
            if lane_idx == 1
                cand = 2;
            elseif lane_idx == 3
                cand = 2;
            else
                cand = [1, 3];
            end
        
            % 从候选车道里找一个安全车道
            new_lane = lane_idx; % 默认保持
            for ci = 1:length(cand)
                if ~danger(cand(ci),1)
                    new_lane = cand(ci);
                    break;
                end
            end

            stored_target_y = lane_y_all(new_lane);
            state = 2;
            action = 3;
        end

        % failsafe: too slow but still dangerous
        obs_id = danger(lane_idx, 2);
        obs_v_risk = 999;
        % 手写一个确定的标量查找
        for ii = 1:length(obs_ID_set)
            if obs_ID_set(ii) == obs_id
                obs_v_risk = obs_v_set_gcs(ii);
                break;
            end
        end
        if (ego_v < 5 || obs_v_risk < 0.2)&& is_danger
            new_lane = lane_idx;
            found = false;

            for li = 1:3
                if danger(li,1)==false
                    new_lane = li;
                    found = true;
                    break;
                end
            end

            if ~found
                action = 4;
            else
                stored_target_y = lane_y_all(new_lane);
                state = 2;
            end
        end

    else
        state = 3;
        action = 1;
    end

case 2   % CHANGE LANE
    block_until_time = time + 0.5;
    stored_target_v = last_target_v;
    state = 3;
    action = 3;

case 3   % BLOCK
    if time >= block_until_time
        state = 0;
    end
    stored_target_v = last_target_v;
    action = 1;

end

%% Outputs
target_y = stored_target_y;
target_v = stored_target_v;
out_state = state;
danger_level = 0;

end  % END fcn


%% =====================================================================
%%                        analyze_danger
%% =====================================================================
function [danger, tx_lane] = analyze_danger( ...
    ego_x, ego_y, ego_v, ego_heading, ...
    obs_x, obs_y, obs_v, obs_h, obs_ID, lane_y, lane_width)

N = length(obs_ID);

danger = zeros(3,2);           % [is_danger, obs_id]
tx_lane = 99 * ones(3,2);      % [ttc, x_collision]

TIME_THRESHOLD   = 4;

for i = 1:N
    if isnan(obs_ID(i)), continue; end

    %% === 计算真实碰撞点 ===
    [ok, ttc, cx, cy] = ray_intersect(...
        ego_x, ego_y, 0, ego_v, ...
        obs_x(i), obs_y(i), obs_h(i), obs_v(i));

    % 不发生碰撞
    if ~ok, continue; end

    % 碰撞时间过长 → 不危险
    if ttc > TIME_THRESHOLD + 1
        continue;
    end


    %% === 判断碰撞点属于哪个 lane ===
    for j = 1:3

        if abs(cy - lane_y(j)) < lane_width/2

            danger(j,1) = 1;            % 标记危险
            danger(j,2) = obs_ID(i);    % 责任障碍物
            tx_lane(j,1) = ttc;         % 真实 TTC
            tx_lane(j,2) = cx;          % 碰撞 X 坐标
        end
    end
end

end





%% =====================================================================
%%                     calc_lost_signal_danger
%% =====================================================================
function danger_level = calc_lost_signal_danger(ego_x, ego_y, ego_heading, ...
    obs_x, obs_y, obs_v, obs_h, obs_ID, lane_y_all, lane_width)

danger_level = 0;

N = length(obs_ID);
if N == 0
    return;
end

% Find nearest lane
lane_idx = 1;
min_dist = abs(ego_y - lane_y_all(1));
for j = 2:3
    if abs(ego_y - lane_y_all(j)) < min_dist
        min_dist = abs(ego_y - lane_y_all(j));
        lane_idx = j;
    end
end

t_min = 1e9;  % scalar

for k = 1:N
    if isnan(obs_ID(k))
        continue;
    end

    vy = obs_v(k) * sin(obs_h(k));
    vx = obs_v(k) * cos(obs_h(k));

    % ----- Compute t_lane -----
    if abs(obs_y(k) - lane_y_all(lane_idx)) < lane_width/2
        t_lane = 0;

    elseif abs(vy) < 0.01
        continue;

    else
        t_lane = (lane_y_all(lane_idx) - obs_y(k)) / vy;
        if t_lane <= 0
            continue;
        end
    end

    if t_lane < t_min
        t_min = t_lane;
    end
end

% ----- Level mapping -----
if t_min < 3
    danger_level = 3;
elseif t_min < 5
    danger_level = 2;
elseif t_min < 10
    danger_level = 1;
end

end


%% =====================================================================
%%                       ray_intersect
%% =====================================================================
function [is_possible, collision_time, collision_x, collision_y] = ...
    ray_intersect(x1,y1,h1,v1, x2,y2,h2,v2)

% 默认返回
is_possible = false;
collision_time = Inf;
collision_x = NaN;
collision_y = NaN;

% Heading unit vectors
H1 = [cos(h1), sin(h1)];
H2 = [cos(h2), sin(h2)];

% If any vehicle is stationary, still OK
V1 = v1*H1;
V2 = v2*H2;

% Relative position
rx = x2 - x1;
ry = y2 - y1;

% Relative velocity
vx_rel = V2(1) - V1(1);
vy_rel = V2(2) - V1(2);

% Solve linear system:
% x1 + V1*t = x2 + V2*t2  → (V1 - V2)*t = r
A = [V1(1)-V2(1), -(V1(1)-V2(1));
     V1(2)-V2(2), -(V1(2)-V2(2))];

% But easier: directly solve t1, t2 for ray intersection:
den = H1(1)*H2(2) - H1(2)*H2(1);
if abs(den) < 1e-9
    return; % almost parallel
end

t1 = ( rx*H2(2) - ry*H2(1) ) / den; % ego
t2 = ( rx*H1(2) - ry*H1(1) ) / den; % obs

% Must be forward time
if t1 < 0 || t2 < 0
    if abs(y2 -y1) < 2
        collision_y = y1;
        collision_x = x1 + H1(1)*t1;
        collision_time = (collision_x - x1) / v1;
        is_possible = true;
    end
    return;
end

% Convert param t1 to real physical time
% param_t1 is distance along ray direction, so:
if v1 < 0.01
    return;
end

collision_time = t1 / v1;

% Compute collision point
collision_x = x1 + H1(1)*t1;
collision_y = y1 + H1(2)*t1;

% Mark valid
is_possible = true;
end





