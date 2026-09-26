function mdl = build_HILS_Controller(varargin)
%BUILD_HILS_CONTROLLER  HILS_Controller.slx (air-enthalpy 결합) 를 스크립트로 자동 생성한다.
%
%   mdl = build_HILS_Controller()                                   % 겨울, PLC 에뮬레이터
%   mdl = build_HILS_Controller('Season', 'summer')
%   mdl = build_HILS_Controller('PlcIo', 'modbus', 'Host', '192.168.0.10', 'Port', 502)
%
%   옵션 (name/value)
%     'ModelName'  (기본 'HILS_Controller')
%     'Season'     'winter' | 'summer'  -> 초기조건·가상 존 기상 (히트펌프 모드는 제품 리모컨)
%     'PlcIo'      'emulator' | 'modbus'
%     'Host','Port' Modbus TCP 서버 (기본 config/hils_config.json)
%     'StopTime'   (기본 '86400')
%     'Pacing'     true 이면 1 sim s ~ 1 wall s. modbus 모드 기본 true
%     'Save'       (기본 true)
%     'ExportScripts' 폴더 경로: Simulink 없이 MATLAB Function 블록 스크립트만 .m 파일로 내보냄
%                  (tests/run_all_tests.m 의 t_simulink_block_scripts 가 Octave 에서 실행·검증)
%
%   루프 (1 스텝 = 1 s, 건물 구간 = 60 s)
%     PLC 40001~40015 (토출/리턴 T·RH, 노즐 풍량, 대기압, 챔버, 실외, P_HP, 상태, Ack)
%       -> Meas_Delay -> HILS_Core_1s
%            1 s : air-enthalpy 열량 계산·구간 누적, heartbeat/지연/추종/인터록, 상태머신,
%                  설정값 변화율 제한
%            60 s: 구간 평균 (Q_sens, m_w) -> 가상 존 step -> 존 공기 상태(k+1) -> Sequence++
%                  (USE_EXTERNAL_ZONE = 1 이면 외부 master/EnergyPlus 가 준 존 상태 사용)
%       -> Encode_Cmd -> PLC 40100~40106 (T_room_SP, RH_room_SP, 실외 SP, Enable, Seq, heartbeat)
%     60 s / 1 s 두 시간층은 HILS_Core_1s 안에서 hils_run_offline.m (Octave·Python 교차검증 완료)
%     과 같은 순서로 실행된다. 블록을 60 s / 1 s 로 나누면 zone <-> supervisor 사이에 대수루프가
%     생겨 지연을 넣어야 하고, 그러면 검증된 실행 순서와 달라지므로 한 블록으로 묶었다.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'lib'));
o = struct('ModelName', 'HILS_Controller', 'Season', 'winter', 'PlcIo', 'emulator', ...
           'Host', '', 'Port', [], 'StopTime', '86400', 'Pacing', [], 'Save', true, ...
           'ExportScripts', '');
for k = 1:2:numel(varargin)
    o.(varargin{k}) = varargin{k + 1};
end
P = hils_params([], o.Season);
if ~isempty(o.ExportScripts)
    hils_write_params_const(o.Season);
    blocks = {'HILS_Core_1s', script_core(); 'Encode_Cmd', script_encode_cmd(); ...
              'Decode_Meas', script_decode_meas(); 'PLC_Emulator', script_plc_emulator()};
    for i = 1:size(blocks, 1)
        fid = fopen(fullfile(o.ExportScripts, [blocks{i, 1} '.m']), 'w');
        fprintf(fid, '%s\n', blocks{i, 2});
        fclose(fid);
    end
    mdl = blocks(:, 1);
    return;
end
if isempty(o.Host), o.Host = P.modbus.host; end
if isempty(o.Port), o.Port = P.modbus.port; end
if isempty(o.Pacing), o.Pacing = strcmpi(o.PlcIo, 'modbus'); end
mdl = o.ModelName;
Ts = P.timing.Ts_realization;

hils_write_params_const(o.Season);          % MATLAB Function 블록용 상수 파라미터

% tunable 변수 (Simulation 객체 setVariable)
assignin('base', 'T_zone_ext', P.zone.T0);
assignin('base', 'RH_zone_ext', P.zone.RH0);
assignin('base', 'T_out_ext', P.wx.T_mean);
assignin('base', 'RH_out_ext', P.wx.RH);
assignin('base', 'USE_EXTERNAL_ZONE', 0);   % 0: 내장 가상 존, 1: 외부(EnergyPlus/MATLAB master)

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
          'FixedStep', num2str(Ts), 'StopTime', o.StopTime, ...
          'SaveOutput', 'on', 'ReturnWorkspaceOutputs', 'on', 'EnableMultiTasking', 'off');
set_param(mdl, 'InitFcn', ...
    'addpath(fullfile(fileparts(get_param(bdroot, ''FileName'')), ''lib''));');
if o.Pacing
    try
        set_param(mdl, 'EnablePacing', 'on', 'PacingRate', 1);
    catch err
        warning('HILS:pacing', 'simulation pacing 설정 실패: %s', err.message);
    end
end

% ------------------------------------------------ 외부 존 상태 (tunable)
names = {'T_zone_ext', 'RH_zone_ext', 'T_out_ext', 'RH_out_ext'};
for i = 1:4
    add_block('simulink/Sources/Constant', [mdl '/' names{i}], 'Value', names{i}, ...
              'SampleTime', 'inf', 'Position', [30 30 + 50 * (i - 1) 130 60 + 50 * (i - 1)]);
end
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_ext'], 'Inputs', '4', 'Position', [170 30 175 210]);
for i = 1:4, add_line(mdl, [names{i} '/1'], sprintf('Mux_ext/%d', i)); end
add_block('simulink/Sources/Constant', [mdl '/USE_EXTERNAL_ZONE'], 'Value', 'USE_EXTERNAL_ZONE', ...
          'SampleTime', 'inf', 'Position', [30 240 150 270]);
add_block('simulink/Sources/Digital Clock', [mdl '/Clock_1s'], 'SampleTime', num2str(Ts), ...
          'Position', [30 300 90 330]);

% ------------------------------------------------ HILS core (1 s)
add_mfb(mdl, 'HILS_Core_1s', [300 60 520 340], Ts, script_core());
add_line(mdl, 'Clock_1s/1', 'HILS_Core_1s/2', 'autorouting', 'on');
add_line(mdl, 'Mux_ext/1', 'HILS_Core_1s/3', 'autorouting', 'on');
add_line(mdl, 'USE_EXTERNAL_ZONE/1', 'HILS_Core_1s/4', 'autorouting', 'on');
add_mfb(mdl, 'Encode_Cmd', [580 70 700 130], Ts, script_encode_cmd());
add_line(mdl, 'HILS_Core_1s/1', 'Encode_Cmd/1');

% ------------------------------------------------ PLC I/O
E0 = hils_plc_emulator_init(P);
switch lower(o.PlcIo)
    case 'emulator'
        add_mfb(mdl, 'PLC_Emulator', [760 70 900 130], Ts, script_plc_emulator());
        add_line(mdl, 'Encode_Cmd/1', 'PLC_Emulator/1');
        meas_src = 'PLC_Emulator/1';
    case 'modbus'
        meas_src = add_modbus_io(mdl, P, o);
    otherwise
        error('HILS:PlcIo', 'PlcIo must be ''emulator'' or ''modbus''');
end
% 1 스텝 지연: 측정 = 직전 PLC 스캔 결과 (대수루프 방지, 검증된 실행 순서와 동일)
add_block('simulink/Discrete/Unit Delay', [mdl '/Meas_Delay'], ...
          'InitialCondition', mat2str(E0.meas_words(:)), 'SampleTime', num2str(Ts), ...
          'Position', [960 80 1000 120]);
add_line(mdl, meas_src, 'Meas_Delay/1', 'autorouting', 'on');
add_line(mdl, 'Meas_Delay/1', 'HILS_Core_1s/1', 'autorouting', 'on');

% ------------------------------------------------ 기록
add_mfb(mdl, 'Decode_Meas', [760 380 900 440], Ts, script_decode_meas());
add_line(mdl, 'Meas_Delay/1', 'Decode_Meas/1', 'autorouting', 'on');
logs = {'Log_meas', 'hils_meas', 'Decode_Meas/1', [960 395 1040 425]; ...
        'Log_cmd', 'hils_cmd', 'HILS_Core_1s/1', [580 170 660 200]; ...
        'Log_status', 'hils_status', 'HILS_Core_1s/2', [580 230 660 260]; ...
        'Log_air', 'hils_air', 'HILS_Core_1s/3', [580 290 660 320]; ...
        'Log_zone', 'hils_zone', 'HILS_Core_1s/4', [580 350 660 380]};
for i = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [mdl '/' logs{i, 1}], 'VariableName', logs{i, 2}, ...
              'SaveFormat', 'Structure With Time', 'Position', logs{i, 4});
    add_line(mdl, logs{i, 3}, [logs{i, 1} '/1'], 'autorouting', 'on');
end

annotate(mdl, o);
if o.Save
    save_system(mdl, fullfile(here, [mdl '.slx']));
    fprintf('saved %s\n', fullfile(here, [mdl '.slx']));
end
end

% =====================================================================
function add_mfb(mdl, name, pos, Ts, script)
blk = [mdl '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
try
    cfg = get_param(blk, 'MATLABFunctionConfiguration');     % R2019b+
    cfg.FunctionScript = script;
    cfg.UpdateMethod = 'Discrete';
    cfg.SampleTime = num2str(Ts);
catch
    ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);  % 구버전
    ch.Script = script;
    ch.ChartUpdate = 'DISCRETE';
    ch.SampleTime = num2str(Ts);
end
end

function src = add_modbus_io(mdl, P, o)
%ADD_MODBUS_IO  Industrial Communication Toolbox Modbus Client Write/Read 블록 배치.
[rdLib, wrLib] = find_modbus_blocks();
rd = [mdl '/Modbus_Read_40001'];
wr = [mdl '/Modbus_Write_40100'];
add_block(wrLib, wr, 'Position', [760 70 900 130]);
add_block(rdLib, rd, 'Position', [760 170 900 230]);
s_in = P.registers.T_supply.addr - P.modbus.base_address;    n_in = 15;
s_out = P.registers.T_room_SP.addr - P.modbus.base_address;  n_out = 7;
Ts = P.timing.Ts_plc;
for b = {rd, wr}
    try_set(b{1}, {'DeviceAddress', 'IPAddress', 'Address'}, o.Host);
    try_set(b{1}, {'Port'}, num2str(o.Port));
    try_set(b{1}, {'ServerID', 'DeviceID', 'UnitID'}, num2str(P.modbus.unit_id));
    try_set(b{1}, {'Target', 'TargetType', 'RegisterType'}, 'Holding Registers');
    try_set(b{1}, {'Precision', 'DataType'}, 'uint16');
    try_set(b{1}, {'SampleTime'}, num2str(Ts));
end
try_set(rd, {'StartAddress', 'Address'}, num2str(s_in));     % 0-based (40001 -> 0)
try_set(wr, {'StartAddress', 'Address'}, num2str(s_out));    % 40100 -> 99
try_set(rd, {'Count', 'NumberOfValues'}, num2str(n_in));
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/To_uint16'], ...
          'OutDataTypeStr', 'uint16', 'Position', [720 85 745 115]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/To_double'], ...
          'OutDataTypeStr', 'double', 'Position', [915 185 940 215]);
add_line(mdl, 'Encode_Cmd/1', 'To_uint16/1');
add_line(mdl, 'To_uint16/1', 'Modbus_Write_40100/1');
add_line(mdl, 'Modbus_Read_40001/1', 'To_double/1');
src = 'To_double/1';
fprintf(['Modbus 블록 배치: %s:%d  read 오프셋 %d x%d, write 오프셋 %d x%d\n' ...
         '  -> 블록 대화상자에서 주소/개수/데이터형을 한 번 확인하세요.\n'], ...
        o.Host, o.Port, s_in, n_in, s_out, n_out);
end

function [rdLib, wrLib] = find_modbus_blocks()
cands = {'icommlib', 'icomm_modbus_lib', 'icommmodbuslib', 'modbuslib'};
for i = 1:numel(cands)
    try
        load_system(cands{i});
    catch
        continue;
    end
    r = find_system(cands{i}, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Regexp', 'on', 'Name', 'Modbus.*Read');
    w = find_system(cands{i}, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Regexp', 'on', 'Name', 'Modbus.*Write');
    if ~isempty(r) && ~isempty(w)
        rdLib = r{1}; wrLib = w{1};
        return;
    end
end
error('HILS:modbusLib', ['Modbus Client Read/Write 블록을 찾지 못했습니다. Industrial Communication ' ...
      'Toolbox(R2024b+) 설치를 확인하고, Library Browser 의 블록 경로를 find_modbus_blocks() 후보에 추가하세요.']);
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
txt = sprintf(['HILS Controller (air-enthalpy coupling, auto-generated)\n' ...
               'Season: %s | PLC I/O: %s | zone step 60 s, supervisor/PLC 1 s\n' ...
               'Tunable: T_zone_ext RH_zone_ext T_out_ext RH_out_ext USE_EXTERNAL_ZONE\n' ...
               'hils_status = [state fault pending ack_age latency valid eT eRH T_ref RH_ref]'], ...
              o.Season, o.PlcIo);
try
    a = Simulink.Annotation([mdl '/HILS Controller']);
    a.Text = txt;
    a.Position = [30 480 700 560];
catch err
    warning('HILS:annotation', 'annotation 생략: %s', err.message);
end
end

% ===================================================================== MATLAB Function 스크립트
function s = script_core()
s = strjoin({ ...
'function [cmd, st, air, zone] = HILS_Core_1s(meas_words, t, ext, use_ext)'
'%#codegen'
'% 1 s: air-enthalpy -> 안전/상태머신 -> 설정값.  60 s: 구간 평균 열량 -> 가상 존 -> 존 상태(k+1)'
'% cmd  = [T_room_SP RH_room_SP T_outdoor_SP RH_outdoor_SP Enable Sequence SIM_heartbeat]'''
'% st   = [state fault pending ack_age latency valid eT eRH T_ref RH_ref]'''
'% air  = [Q_sens Q_lat Q_tot m_w] (이번 1 s 측정),  zone = [T_z RH_z T_out RH_out Q_sens_int m_w_int]'''
'persistent S Z zs q'
'P = hils_params_const();'
'if isempty(S)'
'    S = hils_supervisor_init(P);'
'    Z = hils_zone_init(P.zone, P.emulator.P_atm);'
'    zs = hils_zone_state(Z, P.wx);'
'    q = struct(''Q_sens'', 0, ''Q_lat'', 0, ''Q_tot'', 0, ''m_w'', 0, ''n'', 0);'
'end'
'm = hils_decode_meas(meas_words, P);'
'nb = round(P.timing.Ts_building / P.timing.Ts_realization);'
'k = round(t / P.timing.Ts_realization);'
'if mod(k, nb) == 0'
'    if k > 0'
'        [S, q] = hils_supervisor_interval(S);'
'        if use_ext < 0.5'
'            [Z, zs] = hils_zone_step(Z, P.zone, P.wx, P.timing.Ts_building, P.timing.Ts_zone_sub, q.Q_sens, q.m_w);'
'        end'
'    end'
'    if use_ext > 0.5'
'        zs.T_z = ext(1); zs.RH_z = ext(2); zs.T_out = ext(3); zs.RH_out = ext(4);'
'    end'
'    S = hils_supervisor_building(S, P, zs.T_z, zs.RH_z, zs.T_out, zs.RH_out);'
'end'
'[S, c, r] = hils_supervisor_step(S, P, t, m);'
'cmd = [c.T_room_SP; c.RH_room_SP; c.T_outdoor_SP; c.RH_outdoor_SP; c.Enable; c.Sequence; c.SIM_heartbeat];'
'st = [r.state; r.fault; double(r.pending); r.ack_age; r.latency; double(r.valid); r.eT; r.eRH; r.T_ref; r.RH_ref];'
'air = [r.Q_sens; r.Q_lat; r.Q_tot; r.m_w];'
'zone = [zs.T_z; zs.RH_z; zs.T_out; zs.RH_out; q.Q_sens; q.m_w];'
'end'}, newline);
end

function s = script_encode_cmd()
s = strjoin({ ...
'function w = Encode_Cmd(cmd)'
'%#codegen'
'% 명령 -> 레지스터 40100~40106'
'P = hils_params_const();'
'c = struct(''T_room_SP'', cmd(1), ''RH_room_SP'', cmd(2), ''T_outdoor_SP'', cmd(3), ...'
'           ''RH_outdoor_SP'', cmd(4), ''Enable'', cmd(5), ''Sequence'', cmd(6), ''SIM_heartbeat'', cmd(7));'
'w = hils_encode_cmd(c, P);'
'end'}, newline);
end

function s = script_decode_meas()
s = strjoin({ ...
'function y = Decode_Meas(w)'
'%#codegen'
'% 레지스터 40001~40015 -> 공학단위 (15x1, 레지스터 순서)'
'P = hils_params_const();'
'm = hils_decode_meas(w, P);'
'y = [m.T_supply; m.RH_supply; m.T_return; m.RH_return; m.V_air; m.P_atm; m.T_chamber; ...'
'     m.RH_chamber; m.T_outdoor; m.RH_outdoor; m.P_HP; m.HP_status; m.AckSequence; m.PLC_status; m.PLC_heartbeat];'
'end'}, newline);
end

function s = script_plc_emulator()
s = strjoin({ ...
'function w = PLC_Emulator(cmd_words)'
'%#codegen'
'% 가짜 PLC + 실내측 챔버(국부 PI) + 히트펌프 실내기(native). 실물 연결 시 Modbus 블록으로 교체.'
'persistent E'
'P = hils_params_const();'
'if isempty(E), E = hils_plc_emulator_init(P); end'
'[E, w] = hils_plc_emulator_step(E, P, cmd_words);'
'end'}, newline);
end
