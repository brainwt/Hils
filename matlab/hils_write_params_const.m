function hils_write_params_const(season, outFile)
%HILS_WRITE_PARAMS_CONST  hils_config.json -> lib/hils_params_const.m (구조체 리터럴) 생성.
%   MATLAB Function 블록(코드생성)에서는 jsondecode/fileread 를 쓸 수 없으므로,
%   JSON 을 바꾼 뒤에는 이 함수를 다시 실행해 상수 파라미터 함수를 재생성한다.
%   (matlab/tests/run_all_tests.m 가 두 파라미터의 일치 여부를 검사한다)
%   season: 'winter'(기본) | 'summer'  -> 초기조건·히트펌프 모드가 상수에 들어간다
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'lib'));
if nargin < 1 || isempty(season), season = 'winter'; end
if nargin < 2
    outFile = fullfile(here, 'lib', 'hils_params_const.m');
end
P = hils_params([], season);
lines = {'function P = hils_params_const()', ...
         '%HILS_PARAMS_CONST  자동 생성 파일 - 직접 수정하지 말 것 (hils_write_params_const.m 로 재생성).', ...
         '%#codegen'};
lines = emit(P, 'P', lines);
lines{end + 1} = 'end';
fid = fopen(outFile, 'w');
fprintf(fid, '%s\n', lines{:});
fclose(fid);
fprintf('written %s\n', outFile);
end

function lines = emit(v, name, lines)
if isstruct(v)
    f = fieldnames(v);
    for i = 1:numel(f)
        lines = emit(v.(f{i}), [name '.' f{i}], lines);
    end
elseif ischar(v)
    % 주석 문자열(_comment)은 생략
elseif islogical(v)
    if v, s = 'true'; else, s = 'false'; end
    lines{end + 1} = sprintf('%s = %s;', name, s);
else
    lines{end + 1} = sprintf('%s = %.17g;', name, v);
end
end
