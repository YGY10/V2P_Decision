function plot_obstacles_gcs(has_signal, action, danger_level, ...
    obs_x, obs_y, obs_v, obs_heading, obs_id, ...
    host_x, host_y, host_v, host_heading, target_y, collision_info)

coder.extrinsic('figure','clf','hold','grid','axis','xlim','ylim',...
    'xlabel','ylabel','title','plot','quiver','text','fill','legend','sprintf');

%% Persistent for UI
persistent figCreated;
persistent prev_action;

%% Persistent for trajectory memory
persistent traj_ids;        % [1×MAX_OBJ]  保存每个轨迹对应的ID
persistent traj_len;        % [1×MAX_OBJ]  每个轨迹当前长度
persistent traj_history;    % cell: 每个 cell 是 [MAX_POINTS×2] 的数组

MAX_OBJ    = 50;       % 最大障碍物数量
MAX_POINTS = 200;      % 每个障碍物最多保存 200 个点

%% Init persistent trajectory cache
if isempty(traj_ids)
    traj_ids     = zeros(1,MAX_OBJ);          % 0 表示空槽
    traj_len     = zeros(1,MAX_OBJ);          % 每个轨迹当前长度
    traj_history = cell(1,MAX_OBJ);

    % Init trajectory buffers
    for i = 1:MAX_OBJ
        traj_history{i} = zeros(MAX_POINTS,2);   % 每个障碍物的轨迹缓存
    end
end

if isempty(prev_action)
    prev_action = 0;
end

%% Init figure
if isempty(figCreated)
    figure(200);
    figCreated = true;
end

figure(200);
clf; hold on; axis equal;

%% UI colors
set(gcf,'Color',[0.08 0.10 0.12]);
set(gca,'Color',[0.10 0.12 0.14]);
set(gca,'XColor',[0.6 0.7 0.8],'YColor',[0.6 0.7 0.8]);
grid on; set(gca,'GridAlpha',0.15,'GridColor',[0.5 0.6 0.7]);

%% ranges
forward_range = 120;
back_range = 20;
side_range = 70;

xlim([host_x - back_range, host_x + forward_range]);
ylim([host_y - side_range, host_y + side_range]);

xlabel('X (m)','Color',[0.7 0.8 0.9]);
ylabel('Y (m)','Color',[0.7 0.8 0.9]);
title('Local View','Color',[0.8 0.9 1],'FontWeight','bold');

%% Draw Ego
Lh = 5; Wh = 2;
host_heading_rad = deg2rad(-host_heading + 90);

rect = [ Lh/2 Wh/2; Lh/2 -Wh/2; -Lh/2 -Wh/2; -Lh/2 Wh/2 ];
R = [cos(host_heading_rad) -sin(host_heading_rad); sin(host_heading_rad) cos(host_heading_rad)];
rect_rot = (R*rect')'; 
rect_final = rect_rot + [host_x, host_y];

fill(rect_final(:,1)-0.2, rect_final(:,2)-0.2,[0 0 0],'FaceAlpha',0.25,'EdgeColor','none');
fill(rect_final(:,1), rect_final(:,2),[0.15 0.45 0.95],'FaceAlpha',0.55,'EdgeColor',[0.5 0.7 1],'LineWidth',2);
text(host_x-18, host_y, 'Ego','Color',[0.6 0.8 1],'FontSize',12,'FontWeight','bold');
text(host_x-15, host_y+5.5, sprintf('%.1f m/s', host_v),'Color',[0.7 0.9 1],'FontSize',12);

%% Draw Obstacles + Trajectories
for i = 1:length(obs_x)
    if isnan(obs_x(i)), continue; end

    ox = obs_x(i); 
    oy = obs_y(i);
    v  = obs_v(i);
    hd = obs_heading(i);
    oid = obs_id(i);

    %% ============================
    % ⭐ 更新/写入轨迹缓存
    %% ============================
    idx = 0;

    % 查找已有 ID
    for k = 1:MAX_OBJ
        if traj_ids(k) == oid
            idx = k;
            break;
        end
    end

    % 若无 → 找空槽
    if idx == 0
        for k = 1:MAX_OBJ
            if traj_ids(k) == 0
                traj_ids(k) = oid;
                idx = k;
                break;
            end
        end
    end

    if idx == 0
        continue; % 没槽了，跳过
    end

    % 写入轨迹点
    L = traj_len(idx) + 1;
    if L > MAX_POINTS
        L = MAX_POINTS;
    end

    traj_len(idx) = L;
    traj_history{idx}(L,:) = [ox oy];

    %% ============================
    % ⭐ 绘制已有轨迹
    %% ============================
    Ldraw = traj_len(idx);
    traj = traj_history{idx}(1:Ldraw,:);

    plot(traj(:,1), traj(:,2), '-', ...
        'Color',[0.3 0.7 1 0.30], 'LineWidth',1.2);

    %% Vel arrow
    quiver(ox, oy, v*cos(hd), v*sin(hd),1.3,'Color',[0.55 0.75 1],'LineWidth',1.2);
    text(ox+10, oy, sprintf('%.1f m/s', v),'Color',[0.75 0.85 1],'FontSize',10);

    %% ⭐ 碰撞信息
    if ~isempty(collision_info) && ...
       (oid == collision_info.obs_id) && ...
        collision_info.time < 99

        text(ox+25, oy, sprintf('t=%.1f s', collision_info.time), ...
            'Color',[1 0.9 0.3],'FontSize',10,'FontWeight','bold');
    end

    %% Draw obstacle
    if oid==42 || oid==54 || oid==38
        % --- Cars
        Lc=5; Wc=2;
        car_rect=[Lc/2 Wc/2; Lc/2 -Wc/2; -Lc/2 -Wc/2; -Lc/2 Wc/2];
        R=[cos(hd) -sin(hd); sin(hd) cos(hd)];
        rr=(R*car_rect')'+[ox oy];

        fill(rr(:,1)-0.1,rr(:,2)-0.1,[0 0 0],'FaceAlpha',0.28,'EdgeColor','none');
        fill(rr(:,1),rr(:,2),[0.15 0.85 0.75],'FaceAlpha',0.55,'EdgeColor',[0.4 1 0.85],'LineWidth',2);
        text(ox, oy-4.5, sprintf('Car %d', oid),'Color',[0.6 1 0.9],'FontSize',11);

    else
        % --- Pedestrians
        plot(ox, oy, 'o','MarkerSize',2,'MarkerEdgeColor',[1 0.5 0.5], ...
            'MarkerFaceColor',[1 0.2 0.2],'LineWidth',1.5);

        text(ox+2.5, oy, sprintf('%d', oid),'Color',[1 0.6 0.6],'FontSize',11);

        % 距离显示
        dist_to_ego = hypot(ox-host_x, oy-host_y);
        text(ox+2.5, oy-3.5, sprintf('%.1f m', dist_to_ego), ...
            'Color',[1 0.8 0.5],'FontSize',10,'FontWeight','bold');
    end
end

%% Ref line
if ~isnan(target_y) && has_signal
    plot([host_x, host_x+forward_range], ...
         [target_y,target_y], '--','Color',[0.6 0.7 1],'LineWidth',1.2);
end

%% Status bar
if ~has_signal
    s = sprintf('\\color{red}No Signal  Danger Level %d', danger_level);
else
    show_action = action;
    if action==1, show_action = prev_action; end

    switch show_action
        case 0, s='\\color{green}Normal Drive';
        case 2, s='\\color{cyan}Decelerate';
        case 3, s='\\color{magenta}Lane Change';
        otherwise, s='\\color{green}Signal OK';
    end

    if action~=1
        prev_action = action;
    end
end

patch([host_x-back_range host_x+forward_range host_x+forward_range host_x-back_range], ...
      [host_y+side_range host_y+side_range host_y+side_range-6 host_y+side_range-6], ...
      [0.1 0.12 0.15],'FaceAlpha',0.45,'EdgeColor','none');

text(host_x-back_range+2, host_y+side_range-2, s, ...
    'Interpreter','tex','FontSize',13,'FontWeight','bold','Color',[0.9 0.95 1]);

legend('Velocity','Host','TextColor',[0.7 0.8 1]);

end
