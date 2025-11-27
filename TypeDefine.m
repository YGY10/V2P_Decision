function TypeDefine()
% ============================================
%      定义最简 Bus：collision_info_bus
%      字段：
%         1) obs_id : int32
%         2) time   : double
% ============================================

% --------- obs_id ----------
ele(1) = Simulink.BusElement;
ele(1).Name       = 'obs_id';
ele(1).DataType   = 'int32';
ele(1).Dimensions = 1;

% --------- time ------------
ele(2) = Simulink.BusElement;
ele(2).Name       = 'time';
ele(2).DataType   = 'double';
ele(2).Dimensions = 1;

% --------- Create Bus --------
collision_info_bus = Simulink.Bus;
collision_info_bus.Elements = ele;

% --------- 放到 Base Workspace --------
assignin('base','collision_info_bus',collision_info_bus);

end
