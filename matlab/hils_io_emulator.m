function io = hils_io_emulator(P)
%HILS_IO_EMULATOR  hils_io_modbus 와 같은 인터페이스의 메모리 내 PLC 에뮬레이터 I/O.
%   io.write(words) 가 PLC 1 스캔을 실행한다 (결정적 오프라인 시험, Octave 지원).
%   주의: 익명함수 @() E.x 는 생성 시점의 E 를 복사하므로, 상태 공유는 nested function 으로.
E = hils_plc_emulator_init(P);
io.read    = @read_;
io.write   = @write_;
io.state   = @state_;
io.set     = @set_;
io.close   = @() [];
    function w = read_()
        w = E.meas_words;
    end
    function write_(w)
        [E, ~] = hils_plc_emulator_step(E, P, w);
    end
    function s = state_()
        s = E;
    end
    function set_(name, value)
        E.(name) = value;          % 장애 주입: 'estop', 'freeze_heartbeat'
    end
end
