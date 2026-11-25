"""USB-only bare-board diagnostic runner. Logs real measurements; no simulator."""
import argparse
import binascii
import csv
import datetime as dt
import json
from pathlib import Path
import secrets
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / '.tools' / 'python-libs'))
import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--seconds', type=int, default=180)
    parser.add_argument('--skip-eeprom', action='store_true')
    args = parser.parse_args()
    stamp = dt.datetime.now(dt.timezone(dt.timedelta(hours=5, minutes=30))).strftime('%Y-%m-%d_%H-%M-%S_IST')
    out = ROOT / 'runs' / stamp
    out.mkdir(parents=True)
    raw = (out / 'serial.log').open('w', encoding='utf-8', buffering=1)
    csvfile = (out / 'telemetry.csv').open('w', newline='', encoding='utf-8')
    writer = csv.writer(csvfile)
    writer.writerow(['host_monotonic_s', 'sequence', 'board_ms', 'ticks', 'a0_adc', 'vcc_estimate_mv_uncalibrated', 'missed_ticks', 'max_work_us', 'max_lateness_us'])
    report = {'source': 'REAL_USB_HARDWARE', 'port': args.port, 'baud': 115200,
              'output_directory': str(out), 'requested_seconds': args.seconds,
              'frames': 0, 'bad_crc': 0, 'malformed_frames': 0,
              'unexpected_boots': 0, 'sequence_errors': 0, 'echo_sent': 0,
              'echo_received': 0, 'self_tests': [], 'reset_boots': 0,
              'limitations': ['No external load or motor tested', 'No connected I2C sensor tested',
                              'VCC estimate uses uncalibrated internal bandgap',
                              'GPIO and ADC tests use internal pad readback, not external wiring',
                              'RAM test covers a reserved 512-byte buffer, not all 2 KB']}
    port = serial.Serial(args.port, 115200, timeout=0.25, write_timeout=1)

    def send(text):
        raw.write(f'{time.perf_counter():.6f} TX {text}\n')
        port.write((text + '\n').encode('ascii'))

    def receive():
        data = port.readline()
        if not data:
            return None
        raw.write(f'{time.perf_counter():.6f} RX {data!r}\n')
        try:
            text = data.decode('ascii').strip()
            payload, checksum = text.rsplit('*', 1)
            if len(checksum) != 4 or binascii.crc_hqx(payload.encode('ascii'), 0xFFFF) != int(checksum, 16):
                report['bad_crc'] += 1
                return None
            report['frames'] += 1
            return payload
        except (UnicodeError, ValueError):
            report['malformed_frames'] += 1
            return None

    def await_prefix(prefix, timeout=10):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            msg = receive()
            if msg and msg.startswith(prefix):
                return msg
        raise RuntimeError(f'Timed out waiting for {prefix}')

    def reset_and_boot():
        port.dtr = False
        time.sleep(0.1)
        port.reset_input_buffer()
        port.dtr = True
        await_prefix('BOOT,', 5)

    try:
        # Wait for any reset from opening the port before deliberate reset.
        time.sleep(2)
        reset_and_boot()
        print('Diagnostic firmware booted.', flush=True)
        for attempt in range(3):
            if attempt:
                reset_and_boot()
            report['reset_boots'] += 1
            send('TEST')
            await_prefix('TEST_BEGIN')
            test = []
            deadline = time.perf_counter() + 10
            while time.perf_counter() < deadline:
                msg = receive()
                if msg == 'TEST_END':
                    break
                if msg:
                    test.append(msg)
            else:
                raise RuntimeError('Self-test did not complete')
            report['self_tests'].append(test)
            print(f'Self-test {attempt + 1}: ' + '; '.join(test), flush=True)

        if not args.skip_eeprom:
            send('EEPROM')
            await_prefix('EEPROM_BEGIN')
            report['eeprom'] = await_prefix('EEPROM,', 30)
            print(report['eeprom'], flush=True)

        send('START')
        await_prefix('STARTED')
        start = time.perf_counter()
        next_echo = start
        next_progress = start + 30
        pending = set()
        rows = []
        last_seq = None
        last_board = None
        while time.perf_counter() - start < args.seconds:
            now = time.perf_counter()
            if now >= next_echo:
                token = secrets.token_hex(8)
                pending.add('ECHO,' + token)
                send('ECHO,' + token)
                report['echo_sent'] += 1
                next_echo = now + 0.5
            msg = receive()
            received = time.perf_counter()
            if msg:
                if msg.startswith('DATA,'):
                    values = [int(v) for v in msg.split(',')[1:]]
                    if len(values) != 8:
                        raise RuntimeError(f'Wrong telemetry length: {msg}')
                    seq, board_ms = values[:2]
                    if last_seq is not None and seq != last_seq + 1:
                        report['sequence_errors'] += 1
                    if last_board is not None and board_ms <= last_board:
                        report['sequence_errors'] += 1
                    last_seq, last_board = seq, board_ms
                    row = [received] + values
                    writer.writerow(row)
                    rows.append(row)
                elif msg in pending:
                    pending.remove(msg)
                    report['echo_received'] += 1
                elif msg.startswith('BOOT,'):
                    report['unexpected_boots'] += 1
                elif msg.startswith(('ERROR,', 'STOPPED')):
                    raise RuntimeError(f'Unexpected device status during soak: {msg}')
            if now >= next_progress:
                print(f'Soak {now - start:.0f}s: {len(rows)} samples, '
                      f'{report["bad_crc"]} CRC errors, {report["sequence_errors"]} sequence errors.', flush=True)
                next_progress += 30
        send('STOP')
        await_prefix('STOPPED')
        report['unanswered_echoes'] = len(pending)
        if len(rows) < 2:
            raise RuntimeError('Insufficient telemetry')
        duration = rows[-1][0] - rows[0][0]
        board_duration = (rows[-1][2] - rows[0][2]) / 1000
        report.update(telemetry_samples=len(rows), host_duration_s=duration,
                      board_duration_s=board_duration,
                      telemetry_hz=(len(rows) - 1) / duration,
                      control_hz=(rows[-1][3] - rows[0][3]) / duration,
                      clock_difference_percent=100 * (board_duration / duration - 1),
                      missed_ticks=max(r[6] for r in rows),
                      max_work_us=max(r[7] for r in rows),
                      max_lateness_us=max(r[8] for r in rows),
                      a0_min=min(r[4] for r in rows), a0_max=max(r[4] for r in rows),
                      vcc_estimate_mv_min=min(r[5] for r in rows),
                      vcc_estimate_mv_max=max(r[5] for r in rows))
        checks = []
        for test in report['self_tests']:
            by_kind = {}
            for entry in test:
                parts = entry.split(',')
                by_kind.setdefault(parts[0], []).append(list(map(int, parts[1:])))
            checks += [by_kind.get('RAM') == [[512, 0]], by_kind.get('GPIO') == [[18, 0]]]
            adc = by_kind.get('ADC_PIN', [])
            checks.append(len(adc) == 6 and all(lo <= 5 and hi >= 1018 for _, lo, hi in adc))
            internal = by_kind.get('ADC_INTERNAL', [])
            checks.append(len(internal) == 1 and internal[0][0] <= 5 and 180 <= internal[0][1] <= 280)
            pwm = by_kind.get('PWM_D9', [])
            checks.append(len(pwm) == 1 and 3800 <= pwm[0][0] <= 4050 and 430 <= pwm[0][1] <= 510 and pwm[0][2] > 300)
        checks += [report['bad_crc'] == 0, report['malformed_frames'] == 0,
                   report['unexpected_boots'] == 0, report['sequence_errors'] == 0,
                   report['unanswered_echoes'] == 0, report['missed_ticks'] == 0,
                   report['max_work_us'] < 5000, abs(report['clock_difference_percent']) < 2,
                   48 <= report['telemetry_hz'] <= 52,
                   args.skip_eeprom or report.get('eeprom') == 'EEPROM,1024,0,255']
        report['automated_checks_passed'] = all(checks)
        print(json.dumps(report, indent=2), flush=True)
    except Exception as exc:
        report['error'] = str(exc)
        report['automated_checks_passed'] = False
        raise
    finally:
        try:
            send('STOP')
            time.sleep(0.1)
        finally:
            port.close()
            raw.close()
            csvfile.close()
            (out / 'summary.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
            print(f'Results saved: {out}', flush=True)


if __name__ == '__main__':
    main()
