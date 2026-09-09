"""Run nonce-bound, simulation-only Quest trials and preserve raw evidence."""
import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid

from tools.performance.measurements import evaluate, health

PACKAGE = 'com.questnewton'
NO_WINDOW = getattr(subprocess, 'CREATE_NO_WINDOW', 0)


class Sweep:
    def __init__(self, args):
        self.args = args
        self.root = args.output.resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.device = None
        self.children = []
        self.thermal_stop = False

    def call(self, arguments, timeout=30, checked=True):
        command = [str(self.args.metavr)]
        if self.device:
            command += ['-d', self.device]
        result = subprocess.run(command + arguments, capture_output=True, text=True,
                                encoding='utf-8', errors='replace', timeout=timeout,
                                creationflags=NO_WINDOW)
        text = result.stdout + result.stderr
        if checked and result.returncode:
            raise RuntimeError(f'Meta {arguments[:2]} failed: {text.strip()}')
        return text.strip()

    def discover(self):
        # Every device workflow begins with fresh discovery. IDs never enter reports.
        self.device = None
        devices = json.loads(self.call(['device', 'list', '--format', 'json']))
        candidates = [item for item in devices if item.get('model') == 'Quest 3S' and item.get('state') == 'device']
        if len(candidates) != 1:
            raise RuntimeError('Expected one authorized Quest 3S')
        self.device = candidates[0]['id']

    def stopped(self):
        self.call(['app', 'stop', PACKAGE])
        rows = self.call(['shell', 'ps', '-A', '-w', '-o', 'CMD'])
        if not re.search(r'^\s*CMD\s*$', rows, re.M) or re.search(r'^\s*com\.questnewton(?::\S+)?\s*$', rows, re.M):
            raise RuntimeError('Could not verify app process absence')

    def app_json(self, name):
        raw = self.call(['shell', 'run-as', PACKAGE, 'cat', 'files/' + name], checked=False)
        try:
            return json.loads(raw)
        except (ValueError, TypeError):
            return None

    def settings(self):
        return self.call(['shell', 'run-as', PACKAGE, 'cat', 'files/simulation-settings-v1.txt'], checked=False)

    def snapshot_health(self, directory, name):
        raw = {}
        for label, command in [('battery', ['dumpsys', 'battery']),
                               ('thermal', ['dumpsys', 'thermalservice']),
                               ('memory', ['dumpsys', 'meminfo', PACKAGE])]:
            raw[label] = self.call(['shell', *command], checked=False)
            (directory / f'{name}-{label}.txt').write_text(raw[label], encoding='utf-8')
        result = health(raw['battery'], raw['thermal'], raw['memory'])
        (directory / f'{name}-health.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        return result

    def trace(self, directory, run_id):
        if not self.args.trace_config:
            return None
        config = self.args.trace_config.read_text(encoding='utf-8')
        duration_ms = (self.args.seconds + self.args.warmup + 10) * 1000
        config, replacements = re.subn(r'duration_ms:\s*\d+', f'duration_ms: {duration_ms}', config)
        if replacements != 1:
            raise RuntimeError('Expected one trace duration field')
        local = directory / 'trace-config.pbtxt'
        local.write_text(config, encoding='utf-8')
        remote_config = f'/data/misc/perfetto-configs/{run_id}.pbtxt'
        remote_trace = f'/data/misc/perfetto-traces/{run_id}.pftrace'
        self.call(['files', 'push', str(local), remote_config])
        log = (directory / 'capture.log').open('w', encoding='utf-8')
        process = subprocess.Popen([str(self.args.metavr), '-d', self.device, 'shell', 'perfetto',
                                    '--txt', '-c', remote_config, '-o', remote_trace],
                                   stdout=log, stderr=subprocess.STDOUT, creationflags=NO_WINDOW)
        self.children.append(process)
        return process, log, remote_trace

    def trial(self, rate, index):
        run_id = f'b{rate}_{uuid.uuid4().hex[:12]}'
        directory = self.root / f'{index:02d}-{self.args.workload}-{rate}hz-{run_id[-6:]}'
        directory.mkdir()
        self.stopped()
        started = dt.datetime.now(dt.timezone.utc).isoformat()
        command = ['shell', 'am', 'start', '-W', '-n', PACKAGE + '/.MainActivity',
                   '--es', 'quest_newton_mode', 'benchmark', '--es', 'quest_newton_run_id', run_id,
                   '--ei', 'quest_newton_benchmark_hz', str(rate),
                   '--ei', 'quest_newton_benchmark_seconds', str(self.args.seconds),
                   '--ei', 'quest_newton_benchmark_warmup_seconds', str(self.args.warmup),
                   '--es', 'quest_newton_benchmark_workload', self.args.workload]
        launch = self.call(command)
        (directory / 'launch.log').write_text(launch, encoding='utf-8')
        if re.search(r'^\s*(Error|Exception):', launch, re.M):
            raise RuntimeError('Activity launch error')
        pid = self.call(['shell', 'pidof', PACKAGE])
        if not re.fullmatch(r'\d+', pid):
            raise RuntimeError('App PID unavailable')
        print(json.dumps(dict(event='started', rate=rate, run_id=run_id, directory=str(directory))), flush=True)
        deadline = time.monotonic() + 180
        trace = None
        before = None
        last_phase = None
        while True:
            if (self.root / 'cancel').exists():
                raise KeyboardInterrupt('Sweep cancellation requested')
            status = self.app_json('full-benchmark-status.json')
            if status and status.get('run_id') == run_id and str(status.get('pid')) == pid:
                (directory / 'status.json').write_text(json.dumps(status, indent=2), encoding='utf-8')
                phase = status.get('phase')
                if phase != last_phase:
                    print(json.dumps(dict(event='phase', rate=rate, phase=phase)), flush=True)
                    last_phase = phase
                if phase in ('warmup', 'measuring') and before is None:
                    if phase == 'measuring':
                        raise RuntimeError('Missed warmup health sampling; increase warmup to exclude host collection from measurement')
                    before = self.snapshot_health(directory, 'before')
                    # Stop on a real device thermal warning, not a guessed sensor limit.
                    if before['thermal_status'] is not None and before['thermal_status'] >= 3:
                        self.thermal_stop = True
                        raise RuntimeError('Device reports severe thermal throttling; sweep stopped')
                    trace = self.trace(directory, run_id)
                    ready = self.app_json('full-benchmark-status.json')
                    if not ready or ready.get('run_id') != run_id or str(ready.get('pid')) != pid or ready.get('phase') != 'warmup':
                        raise RuntimeError('Health/trace setup reached the measurement boundary; increase warmup and rerun')
                    (directory / 'warmup-collection-complete.json').write_text(json.dumps(ready, indent=2), encoding='utf-8')
                    deadline = time.monotonic() + self.args.warmup + self.args.seconds + 60
                if phase in ('complete', 'failed'):
                    if phase == 'complete' and before is None:
                        raise RuntimeError('Completed trial has no verified warmup health/setup boundary; rerun with longer warmup')
                    break
            if time.monotonic() > deadline:
                raise TimeoutError(f'Benchmark {rate} Hz did not finish')
            time.sleep(.5)
        result = self.app_json(f'full-benchmark-{run_id}.json')
        final_deadline = time.monotonic() + 10
        while result is None and time.monotonic() < final_deadline:
            time.sleep(.2)
            result = self.app_json(f'full-benchmark-{run_id}.json')
        if not result or result.get('run_id') != run_id or str(result.get('pid')) != pid or result.get('requested_physics_hz') != rate:
            raise RuntimeError('Final benchmark identity missing or stale')
        (directory / 'native.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        after = self.snapshot_health(directory, 'after')
        if trace:
            process, log, remote = trace
            while process.poll() is None:
                if (self.root / 'cancel').exists():
                    raise KeyboardInterrupt('Sweep cancellation requested')
                if time.monotonic() > deadline + 30:
                    raise TimeoutError('Perfetto capture did not terminate')
                time.sleep(.5)
            log.close()
            if process.returncode:
                raise RuntimeError('Perfetto capture failed; see capture.log')
            self.call(['files', 'pull', remote, str(directory / 'trace.pftrace')], timeout=90)
        evaluation = evaluate(result)
        summary = dict(started_utc=started, directory=directory.name, run_id=run_id, native=result,
                       evaluation=evaluation, health_before=before, health_after=after, traced=bool(trace))
        (directory / 'trial.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        with (self.root / 'trials.jsonl').open('a', encoding='utf-8') as out:
            out.write(json.dumps(summary) + '\n')
        print(json.dumps(dict(event='result', rate=rate, **evaluation)), flush=True)
        self.stopped()
        if after['thermal_status'] is not None and after['thermal_status'] >= 3:
            self.thermal_stop = True
            raise RuntimeError('Device reports severe thermal throttling; sweep stopped')
        return summary

    def run(self):
        self.discover()
        original_settings = self.settings()
        (self.root / 'saved-settings-before.txt').write_text(original_settings, encoding='utf-8')
        original_proximity = json.loads(self.call(['device', 'proximity', '--status', '--format', 'json']))
        (self.root / 'proximity-before.json').write_text(json.dumps(original_proximity, indent=2), encoding='utf-8')
        was_running = self.args.restore_live or bool(re.fullmatch(r'\d+', self.call(['shell', 'pidof', PACKAGE], checked=False)))
        changed_proximity = original_proximity.get('enabled') is True
        try:
            if changed_proximity:
                self.call(['device', 'proximity', '--disable', '--duration-ms', '1800000'])
            summaries = []
            for index, rate in enumerate(self.args.rates, 1):
                summaries.append(self.trial(rate, index))
            return summaries
        finally:
            cleanup_errors = []
            for child in self.children:
                if child.poll() is None:
                    try:
                        child.terminate()
                        child.wait(timeout=10)
                    except Exception as error:
                        cleanup_errors.append(str(error))
            try:
                self.stopped()
            except Exception as error:
                cleanup_errors.append(str(error))
            if changed_proximity:
                try:
                    self.call(['device', 'proximity', '--enable'])
                except Exception as error:
                    cleanup_errors.append(str(error))
            try:
                if self.settings() != original_settings:
                    cleanup_errors.append('Saved settings changed unexpectedly; original preserved in evidence directory')
            except Exception as error:
                cleanup_errors.append(str(error))
            if was_running and not self.args.leave_stopped and not self.thermal_stop and not (self.root / 'cancel').exists():
                try:
                    self.call(['app', 'launch', '--cold-start', PACKAGE])
                except Exception as error:
                    cleanup_errors.append(str(error))
            print(json.dumps(dict(event='cleanup', errors=cleanup_errors,
                                  restored_when_successful=['app stop', 'proximity', 'saved-settings check', 'original live mode'])), flush=True)
            if cleanup_errors:
                if sys.exc_info()[0] is None:
                    raise RuntimeError('; '.join(cleanup_errors))
                print('Additional cleanup errors: '+ '; '.join(cleanup_errors), file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metavr', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--rates', default='100,200,250,300,350,400,500,1000', type=lambda x: [int(v) for v in x.split(',')])
    parser.add_argument('--seconds', type=int, default=30)
    parser.add_argument('--warmup', type=int, default=8)
    parser.add_argument('--workload', choices=['motion', 'rest'], default='motion')
    parser.add_argument('--trace-config', type=Path)
    parser.add_argument('--leave-stopped', action='store_true')
    parser.add_argument('--restore-live', action='store_true', help='Restore live mode after a preceding benchmark APK installation')
    args = parser.parse_args()
    if not all(50 <= x <= 2000 for x in args.rates) or not 5 <= args.seconds <= 600 or not 0 <= args.warmup <= 60:
        parser.error('Rates 50..2000, seconds 5..600, warmup 0..60 required')
    Sweep(args).run()


if __name__ == '__main__':
    main()
