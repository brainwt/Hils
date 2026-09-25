function mdl = build_HILS_Controller(varargin)
%BUILD_HILS_CONTROLLER  HILS_Controller.slx 를 스크립트로 자동 생성한다.
%
%   mdl = build_HILS_Controller()                          % 기본: PLC 에뮬레이터 모드
%   mdl = build_HILS_Controller('PlcIo', 'modbus', 'Host', '192.168.0.10', 'Port', 502)
%
%   옵션 (name/value)
%     'ModelName'   모델 이름                         (기본 'HILS_Controller')
%     'PlcIo'       'emulator' | 'modbus'              (기본 'emulator')
%                   emulator: 가짜 PLC/챔버/히트펌프 MATLAB Function (실물 없이 폐루프 시험)
%                   modbus  : Industrial Communication Toolbox Modbus Client Read/Write 블록
%     'Host','Port' Modbus TCP 서버 주소 (기본 config/hils_config.json 값)
%     'StopTime'    (기본 '86400')
%     'Pacing'      true 이면 simulation pacing(1 sim s ~ 1 wall s). modbus 모드 기본 true
%     'Save'        true 이면 matlab/HILS_Controller.slx 로 저장 (기본 true)
%
%   모델 구조 (계획서 3, 9 절)
%     [Targets 60 s]  Q_zone_target/T_zone_target/T_out_target (tunable 변수)
%                     또는 내부 Virtual Building(2R2C) -> Switch(USE_EXTERNAL_TARGET)
%     [Supervisor 60 s]  Sequence 카운터 + 목표 ZOH
%     [Rate Transition 60 s -> 1 s]
%     [Realization 1 s]  heartbeat/지연감시/안전/상태머신/PI (hils_supervisor_step)
%        -> Rate Limiter -> Saturation (2차 보호) -> Encode (40100~40105)
%     [PLC I/O]  emulator 또는 Modbus Client Write/Read (40001~40010)
%     [Decode + Logging]  To Workspace (hils_meas, hils_cmd, hils_status)
%
%   필요: Simulink (R2021a 이상 권장). modbus 모드는 Industrial Communication
%   Toolbox 의 Modbus Client Read/Write 블록 (R2024b 이상).

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'lib'));
P = hils_params();

o = struct('ModelName', 'HILS_Controller', 'PlcIo', 'emulator', ...
           'Host', P.modbus.host, 'Port', P.modbus.port, 'StopTime', '86400', ...
           'Pacing', [], 'Save', true);
for k = 1:2:numel(varargin)
    o.(varargin{k}) = varargin{k + 1};
end
if isempty(o.Pacing)
    o.Pacing = strcmpi(o.PlcIo, 'modbus');
end
mdl = o.ModelName;
Tb = P.timing.Ts_building;
Ts = P.timing.Ts_realization;

% 상수 파라미터 함수 최신화 (MATLAB Function 블록이 사용)
hils_write_params_const();

% tunable 변수 (Simulation 객체 setVariable 로 실행 중 변경)
assignin('base', 'Q_zone_target', 0);
assignin('base', 'T_zone_target', P.emulator.hp_setpoint);
assignin('base', 'T_out_target', 0);
assignin('base', 'USE_EXTERNAL_TARGET', 0);   % 0: 내부 가상건물, 1: 외부(EnergyPlus/MATLAB master)

% ---------------------------------------------------------------- 모델 생성
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
          'FixedStep', num2str(Ts), 'StopTime', o.StopTime, ...
          'SaveOutput', 'on', 'ReturnWorkspaceOutputs', 'on', ...
          'EnableMultiTasking', 'off');      % single-tasking: 60 s -> 1 s 전달에 추가 지연 없음
% MATLAB Function 블록이 호출하는 lib/ 함수 경로
set_param(mdl, 'InitFcn', ...
    'addpath(fullfile(fileparts(get_param(bdroot, ''FileName'')), ''lib''));');
if o.Pacing
    try
        set_param(mdl, 'EnablePacing', 'on', 'PacingRate', 1);
    catch err
        warning('HILS:pacing', 'simulation pacing 설정 실패: %s', err.message);
    end
end

% ---------------------------------------------------------------- Targets (60 s)
add_const(mdl, 'Q_zone_target', 'Q_zone_target', [30 40], Tb);
add_const(mdl, 'T_zone_target', 'T_zone_target', [30 100], Tb);
add_const(mdl, 'T_out_target', 'T_out_target', [30 160], Tb);
add_const(mdl, 'USE_EXTERNAL_TARGET', 'USE_EXTERNAL_TARGET', [30 230], Tb);

add_mfb(mdl, 'Virtual_Building', [180 280 330 360], Tb, script_building());
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_ext'], 'Inputs', '3', ...
          'Position', [200 40 205 180]);
add_block('simulink/Signal Routing/Switch', [mdl '/Target_Select'], ...
          'Criteria', 'u2 > Threshold', 'Threshold', '0.5', 'Position', [400 150 450 290]);
add_line(mdl, 'Q_zone_target/1', 'Mux_ext/1');
add_line(mdl, 'T_zone_target/1', 'Mux_ext/2');
add_line(mdl, 'T_out_target/1', 'Mux_ext/3');
add_line(mdl, 'Mux_ext/1', 'Target_Select/1', 'autorouting', 'on');
add_line(mdl, 'USE_EXTERNAL_TARGET/1', 'Target_Select/2', 'autorouting', 'on');
add_line(mdl, 'Virtual_Building/1', 'Target_Select/3', 'autorouting', 'on');

% ---------------------------------------------------------------- Supervisor (60 s)
add_mfb(mdl, 'Supervisor_60s', [520 170 660 250], Tb, script_supervisor60());
add_line(mdl, 'Target_Select/1', 'Supervisor_60s/1', 'autorouting', 'on');
% Rate Transition: Integrity/Deterministic 을 끄면 single-tasking 에서 ZOH (지연 없음).
% 기본값(deterministic)은 slow->fast 에서 1 slow-주기(60 s) 지연을 넣으므로 사용하지 않는다.
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_slow2fast'], ...
          'OutPortSampleTime', num2str(Ts), 'Integrity', 'off', 'Deterministic', 'off', ...
          'Position', [700 190 740 230]);
add_line(mdl, 'Supervisor_60s/1', 'RT_slow2fast/1');

% ---------------------------------------------------------------- Realization (1 s)
add_block('simulink/Sources/Digital Clock', [mdl '/Clock_1s'], 'SampleTime', num2str(Ts), ...
          'Position', [700 300 740 330]);
add_mfb(mdl, 'Realization_1s', [800 170 980 350], Ts, script_realization());
add_line(mdl, 'RT_slow2fast/1', 'Realization_1s/1');
add_line(mdl, 'Clock_1s/1', 'Realization_1s/3', 'autorouting', 'on');

% 명령 벡터 [Q_cmd T_sp Enable Seq SIM_hb Tout_sp] 에서 Q_cmd 만 2차 보호
add_block('simulink/Signal Routing/Demux', [mdl '/Cmd_Demux'], 'Outputs', '[1 5]', ...
          'Position', [1020 170 1025 250]);
add_line(mdl, 'Realization_1s/1', 'Cmd_Demux/1');
rc = P.realization_controller;
add_block('simulink/Discontinuities/Rate Limiter', [mdl '/Rate_Limiter'], ...
          'RisingSlewLimit', num2str(rc.rate_limit_W_per_s), ...
          'FallingSlewLimit', num2str(-rc.rate_limit_W_per_s), ...
          'InitialCondition', '0', 'Position', [1060 160 1100 190]);
add_block('simulink/Discontinuities/Saturation', [mdl '/Saturation'], ...
          'UpperLimit', num2str(rc.Q_cmd_max), 'LowerLimit', num2str(rc.Q_cmd_min), ...
          'Position', [1130 160 1170 190]);
add_block('simulink/Signal Routing/Mux', [mdl '/Cmd_Mux'], 'Inputs', '[1 5]', ...
          'Position', [1200 170 1205 250]);
add_line(mdl, 'Cmd_Demux/1', 'Rate_Limiter/1');
add_line(mdl, 'Rate_Limiter/1', 'Saturation/1');
add_line(mdl, 'Saturation/1', 'Cmd_Mux/1');
add_line(mdl, 'Cmd_Demux/2', 'Cmd_Mux/2');
add_mfb(mdl, 'Encode_Cmd', [1240 180 1340 240], Ts, script_encode_cmd());
add_line(mdl, 'Cmd_Mux/1', 'Encode_Cmd/1');

% ---------------------------------------------------------------- PLC I/O
E0 = hils_plc_emulator_init(P);
switch lower(o.PlcIo)
    case 'emulator'
        add_mfb(mdl, 'PLC_Emulator', [1400 180 1520 240], Ts, script_plc_emulator());
        add_line(mdl, 'Encode_Cmd/1', 'PLC_Emulator/1');
        meas_src = 'PLC_Emulator/1';
    case 'modbus'
        meas_src = add_modbus_io(mdl, P, o);
    otherwise
        error('HILS:PlcIo', 'PlcIo must be ''emulator'' or ''modbus''');
end
% 1 스텝 지연: 측정은 직전 PLC 스캔 결과 (대수루프 방지, Python master 와 동일 순서)
add_block('simulink/Discrete/Unit Delay', [mdl '/Meas_Delay'], ...
          'InitialCondition', mat2str(E0.meas_words(:)), 'SampleTime', num2str(Ts), ...
          'Position', [1560 190 1600 230]);
add_line(mdl, meas_src, 'Meas_Delay/1', 'autorouting', 'on');
add_line(mdl, 'Meas_Delay/1', 'Realization_1s/2', 'autorouting', 'on');

% ---------------------------------------------------------------- Decode + building feedback
add_mfb(mdl, 'Decode_Meas', [1400 380 1520 440], Ts, script_decode_meas());
add_line(mdl, 'Meas_Delay/1', 'Decode_Meas/1', 'autorouting', 'on');
add_block('simulink/Signal Routing/Selector', [mdl '/Sel_T_indoor'], ...
          'InputPortWidth', '10', 'Indices', '1', 'Position', [1400 480 1440 510]);
add_line(mdl, 'Decode_Meas/1', 'Sel_T_indoor/1', 'autorouting', 'on');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_fast2slow'], ...
          'OutPortSampleTime', num2str(Tb), 'Integrity', 'off', 'Deterministic', 'off', ...
          'Position', [100 300 140 340]);
add_line(mdl, 'Sel_T_indoor/1', 'RT_fast2slow/1', 'autorouting', 'on');
add_line(mdl, 'RT_fast2slow/1', 'Virtual_Building/1', 'autorouting', 'on');

% ---------------------------------------------------------------- Logging
add_tows(mdl, 'Log_meas', 'hils_meas', [1600 380 1680 410]);
add_tows(mdl, 'Log_cmd', 'hils_cmd', [1400 100 1480 130]);
add_tows(mdl, 'Log_status', 'hils_status', [1020 330 1100 360]);
add_tows(mdl, 'Log_target', 'hils_target', [520 330 600 360]);
add_line(mdl, 'Decode_Meas/1', 'Log_meas/1', 'autorouting', 'on');
add_line(mdl, 'Cmd_Mux/1', 'Log_cmd/1', 'autorouting', 'on');
add_line(mdl, 'Realization_1s/2', 'Log_status/1', 'autorouting', 'on');
add_line(mdl, 'Supervisor_60s/1', 'Log_target/1', 'autorouting', 'on');

annotate(mdl, o);
if o.Save
    save_system(mdl, fullfile(here, [mdl '.slx']));
    fprintf('saved %s\n', fullfile(here, [mdl '.slx']));
end
end

% =====================================================================
function add_const(mdl, name, value, pos, Ts)
add_block('simulink/Sources/Constant', [mdl '/' name], 'Value', value, ...
          'SampleTime', num2str(Ts), 'Position', [pos pos + [90 30]]);
end

function add_tows(mdl, name, var, pos)
add_block('simulink/Sinks/To Workspace', [mdl '/' name], 'VariableName', var, ...
          'SaveFormat', 'Structure With Time', 'Position', pos);
end

function add_mfb(mdl, name, pos, Ts, script)
%ADD_MFB  MATLAB Function 블록 추가 + 스크립트/이산 샘플시간 설정.
blk = [mdl '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
try
    % R2019b+ 권장 API
    cfg = get_param(blk, 'MATLABFunctionConfiguration');
    cfg.FunctionScript = script;
    cfg.UpdateMethod = 'Discrete';
    cfg.SampleTime = num2str(Ts);
catch
    % 구버전: Stateflow API
    ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
    ch.Script = script;
    ch.ChartUpdate = 'DISCRETE';
    ch.SampleTime = num2str(Ts);
end
end

function src = add_modbus_io(mdl, P, o)
%ADD_MODBUS_IO  Industrial Communication Toolbox Modbus Client Write/Read 블록 배치.
%   블록 라이브러리 경로와 대화상자 파라미터 이름은 릴리스마다 다를 수 있어
%   라이브러리에서 이름으로 검색한다. 찾지 못하면 오류와 함께 수동 설정 방법을 안내.
[rdLib, wrLib] = find_modbus_blocks();
rd = [mdl '/Modbus_Read_40001'];
wr = [mdl '/Modbus_Write_40100'];
add_block(wrLib, wr, 'Position', [1400 180 1520 240]);
add_block(rdLib, rd, 'Position', [1400 280 1520 340]);
[s_in, n_in] = deal(P.registers.T_indoor.addr - P.modbus.base_address, 10);
[s_out, n_out] = deal(P.registers.Q_load_cmd.addr - P.modbus.base_address, 6);
Ts = P.timing.Ts_plc;
try_set(rd, {'DeviceAddress', 'IPAddress', 'Address'}, o.Host);
try_set(wr, {'DeviceAddress', 'IPAddress', 'Address'}, o.Host);
try_set(rd, {'Port'}, num2str(o.Port));
try_set(wr, {'Port'}, num2str(o.Port));
try_set(rd, {'ServerID', 'DeviceID', 'UnitID'}, num2str(P.modbus.unit_id));
try_set(wr, {'ServerID', 'DeviceID', 'UnitID'}, num2str(P.modbus.unit_id));
try_set(rd, {'Target', 'TargetType', 'RegisterType'}, 'Holding Registers');
try_set(wr, {'Target', 'TargetType', 'RegisterType'}, 'Holding Registers');
try_set(rd, {'StartAddress', 'Address'}, num2str(s_in));      % 0-based 오프셋 (40001 -> 0)
try_set(wr, {'StartAddress', 'Address'}, num2str(s_out));     % 40100 -> 99
try_set(rd, {'Count', 'NumberOfValues'}, num2str(n_in));
try_set(rd, {'Precision', 'DataType'}, 'uint16');
try_set(wr, {'Precision', 'DataType'}, 'uint16');
try_set(rd, {'SampleTime'}, num2str(Ts));
try_set(wr, {'SampleTime'}, num2str(Ts));
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/To_uint16'], ...
          'OutDataTypeStr', 'uint16', 'Position', [1360 195 1390 225]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/To_double'], ...
          'OutDataTypeStr', 'double', 'Position', [1530 295 1560 325]);
add_line(mdl, 'Encode_Cmd/1', 'To_uint16/1');
add_line(mdl, 'To_uint16/1', 'Modbus_Write_40100/1');
add_line(mdl, 'Modbus_Read_40001/1', 'To_double/1');
src = 'To_double/1';
fprintf(['Modbus 블록 배치 완료: %s:%d  read 오프셋 %d x%d, write 오프셋 %d x%d\n' ...
         '  -> 블록 대화상자에서 주소/개수/데이터형을 한 번 확인하세요 (릴리스별 파라미터 이름 차이).\n'], ...
        o.Host, o.Port, s_in, n_in, s_out, n_out);
end

function [rdLib, wrLib] = find_modbus_blocks()
rdLib = ''; wrLib = '';
cands = {'icommlib', 'icomm_modbus_lib', 'icommmodbuslib', 'modbuslib'};
for i = 1:numel(cands)
    try
        load_system(cands{i});
    catch
        continue;
    end
    r = find_system(cands{i}, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                    'Regexp', 'on', 'Name', 'Modbus.*Read');
    w = find_system(cands{i}, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                    'Regexp', 'on', 'Name', 'Modbus.*Write');
    if ~isempty(r) && ~isempty(w)
        rdLib = r{1}; wrLib = w{1};
        return;
    end
end
error('HILS:modbusLib', ['Modbus Client Read/Write 블록을 찾지 못했습니다. ' ...
      'Industrial Communication Toolbox(R2024b+) 설치 여부를 확인하고, ' ...
      'Library Browser 에서 블록 경로를 확인해 find_modbus_blocks() 의 후보 목록에 추가하세요.']);
end

function try_set(blk, names, value)
dp = get_param(blk, 'DialogParameters');
for i = 1:numel(names)
    if isfield(dp, names{i})
        try
            set_param(blk, names{i}, value);
        catch err
            warning('HILS:modbusParam', '%s.%s 설정 실패: %s', blk, names{i}, err.message);
        end
        return;
    end
end
end

function annotate(mdl, o)
txt = sprintf(['HILS Controller (auto-generated by build_HILS_Controller.m)\n' ...
               'PLC I/O: %s | Building/Supervisor 60 s, Realization/PLC 1 s\n' ...
               'Tunable: Q_zone_target, T_zone_target, T_out_target, USE_EXTERNAL_TARGET\n' ...
               'hils_status = [state fault pending ack_age latency e valid q_ref kp integ]'], o.PlcIo);
try
    a = Simulink.Annotation([mdl '/HILS Controller']);
    a.Text = txt;
    a.Position = [30 540 700 620];
catch err
    warning('HILS:annotation', 'annotation 생략: %s', err.message);
end
end

% ===================================================================== MATLAB Function 스크립트
function s = script_building()
s = sprintf([ ...
'function y = Virtual_Building(T_indoor)\n' ...
'%%#codegen\n' ...
'%% 가상건물 2R2C (EnergyPlus/FMU 대체). y = [Q_target; T_target; T_out]\n' ...
'persistent B t\n' ...
'P = hils_params_const();\n' ...
'if isempty(B), B = struct(''Tm'', 18); t = 0; end\n' ...
'[B, out] = hils_building_step(B, P.virtual_building, P.timing.Ts_building, t, T_indoor);\n' ...
't = t + P.timing.Ts_building;\n' ...
'y = [out.Q_target; P.emulator.hp_setpoint; out.T_out];\n' ...
'end\n']);
end

function s = script_supervisor60()
s = sprintf([ ...
'function y = Supervisor_60s(target)\n' ...
'%%#codegen\n' ...
'%% 60 s 층: 새 건물 목표마다 Sequence 증가. y = [seq; Q_target; T_target; T_out]\n' ...
'persistent seq\n' ...
'if isempty(seq), seq = 0; end\n' ...
'seq = mod(seq, 65535) + 1;\n' ...
'y = [seq; target(1); target(2); target(3)];\n' ...
'end\n']);
end

function s = script_realization()
s = sprintf([ ...
'function [cmd, st] = Realization_1s(target, meas_words, t)\n' ...
'%%#codegen\n' ...
'%% 1 s 층: heartbeat/지연감시/안전/상태머신/PI. hils_supervisor_step 과 동일.\n' ...
'%% cmd = [Q_load_cmd T_chamber_SP Enable Sequence SIM_heartbeat T_outdoor_SP]''\n' ...
'%% st  = [state fault pending ack_age latency e valid q_ref kp integ]''\n' ...
'persistent S\n' ...
'P = hils_params_const();\n' ...
'if isempty(S), S = hils_supervisor_init(P); end\n' ...
'if target(1) ~= S.seq\n' ...
'    S = hils_supervisor_building(S, P, target(2), target(3), target(4));\n' ...
'    S.seq = target(1);\n' ...
'end\n' ...
'm = hils_decode_meas(meas_words, P);\n' ...
'[S, c, r] = hils_supervisor_step(S, P, t, m);\n' ...
'cmd = [c.Q_load_cmd; c.T_chamber_SP; c.Enable; c.Sequence; c.SIM_heartbeat; c.T_outdoor_SP];\n' ...
'st = [r.state; r.fault; double(r.pending); r.ack_age; r.latency; r.e; double(r.valid); ' ...
'r.q_ref; r.kp; r.integ];\n' ...
'end\n']);
end

function s = script_encode_cmd()
s = sprintf([ ...
'function w = Encode_Cmd(cmd)\n' ...
'%%#codegen\n' ...
'%% 명령 -> 레지스터 40100~40105 (uint16 값, double 형)\n' ...
'P = hils_params_const();\n' ...
'c = struct(''Q_load_cmd'', cmd(1), ''T_chamber_SP'', cmd(2), ''Enable'', cmd(3), ' ...
'''Sequence'', cmd(4), ''SIM_heartbeat'', cmd(5), ''T_outdoor_SP'', cmd(6));\n' ...
'w = hils_encode_cmd(c, P);\n' ...
'end\n']);
end

function s = script_decode_meas()
s = sprintf([ ...
'function y = Decode_Meas(w)\n' ...
'%%#codegen\n' ...
'%% 레지스터 40001~40010 -> 공학단위\n' ...
'%% y = [T_indoor RH T_outdoor P_HP Q_HP HP_status Q_load_meas Ack PLC_status PLC_hb]''\n' ...
'P = hils_params_const();\n' ...
'm = hils_decode_meas(w, P);\n' ...
'y = [m.T_indoor; m.RH_indoor; m.T_outdoor; m.P_HP; m.Q_HP; m.HP_status; ' ...
'm.Q_load_meas; m.AckSequence; m.PLC_status; m.PLC_heartbeat];\n' ...
'end\n']);
end

function s = script_plc_emulator()
s = sprintf([ ...
'function w = PLC_Emulator(cmd_words)\n' ...
'%%#codegen\n' ...
'%% 가짜 PLC + 챔버 + 히트펌프(native 제어). 실물 연결 시 Modbus 블록으로 교체.\n' ...
'persistent E\n' ...
'P = hils_params_const();\n' ...
'if isempty(E), E = hils_plc_emulator_init(P); end\n' ...
'[E, w] = hils_plc_emulator_step(E, P, cmd_words);\n' ...
'end\n']);
end
