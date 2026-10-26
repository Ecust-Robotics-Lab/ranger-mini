"""Linux session ownership, dependency scheduler and guarded process cleanup.

Only descendants of this session's guardian are managed. No command-name
matching, global process killing, automatic restart or saved control permit.
"""
from dataclasses import dataclass
import ctypes
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


BOOT_ID = Path('/proc/sys/kernel/random/boot_id').read_text().strip()

def atomic_json(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def identity(pid):
    try:
        raw = Path(f'/proc/{pid}/stat').read_text()
        values = raw[raw.rfind(')')+2:].split()
        return {'pid': int(pid), 'start_ticks': int(values[19]),
                'ppid': int(values[1]), 'state': values[0],
                'boot_id': BOOT_ID}
    except (OSError, ValueError, IndexError):
        return None


def same_process(record):
    now = identity(record['pid']) if record else None
    return bool(now and now['state'] != 'Z' and now['start_ticks'] == record['start_ticks']
                and now['boot_id'] == record['boot_id'])


def descendants(pid):
    records = {}
    for item in Path('/proc').iterdir():
        if item.name.isdigit():
            record = identity(int(item.name))
            if record and record['state'] != 'Z':
                records[record['pid']] = record
    owned = {pid}
    while True:
        children = {p for p, r in records.items() if r['ppid'] in owned}
        updated = owned | children
        if updated == owned:
            return [r for p, r in records.items() if p in owned and p != pid]
        owned = updated


def notify(record, sig):
    if same_process(record):
        try:
            os.kill(record['pid'], sig)
        except ProcessLookupError:
            pass


def subreaper():
    if ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'PR_SET_CHILD_SUBREAPER failed')


def reap():
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
            if pid == 0:
                return
        except ChildProcessError:
            return


def cleanup_tree(root_pid, leaders=(), grace=8.):
    # Notify launch/supervisor leaders once; let them forward SIGINT normally.
    leaders = list(leaders) or [r for r in descendants(root_pid) if r['ppid'] == root_pid]
    for record in reversed(leaders):
        notify(record, signal.SIGINT)
    end = time.monotonic()+grace
    while time.monotonic() < end:
        if not descendants(root_pid):
            reap()
            return []
        time.sleep(.05)
    for sig, delay in [(signal.SIGTERM, 2.), (signal.SIGKILL, 2.)]:
        for record in descendants(root_pid):
            notify(record, sig)
        end = time.monotonic()+delay
        while time.monotonic() < end:
            reap()
            if not descendants(root_pid):
                return []
            time.sleep(.05)
    reap()
    return descendants(root_pid)


@dataclass
class Component:
    name: str
    argv: list
    depends: tuple = ()
    requires: tuple = ()
    ready_key: str = 'process'
    critical: bool = True
    start_requires: tuple = ()
    resource: str = ''


class JsonHealth:
    def __init__(self, path):
        self.path = Path(path)

    def __call__(self):
        try:
            data = json.loads(self.path.read_text())
            if 0 <= time.monotonic()-data['monotonic'] <= 1.:
                return data['checks']
        except (OSError, KeyError, ValueError, TypeError):
            pass
        return {}


def read_session(directory):
    try:
        current = json.loads((Path(directory)/'current.json').read_text())
        run = Path(current['run_directory'])
        if run.parent.resolve() != Path(directory).resolve():
            raise ValueError('session pointer leaves its directory')
        return run, json.loads((run/'state.json').read_text())
    except FileNotFoundError:
        return None, None


def session_live(state):
    return state and (same_process(state.get('guardian')) or same_process(state.get('owner')) or
                      any(same_process(r) for r in state.get('owned', [])))


def stop_session(directory, timeout=35.):
    run, state = read_session(directory)
    if state is None:
        return True
    # No signal is sent to a reused PID or a process from another boot.
    if same_process(state.get('guardian')):
        notify(state['guardian'], signal.SIGTERM)
    elif same_process(state.get('owner')):
        notify(state['owner'], signal.SIGTERM)
    else:
        # Both supervisors died: only previously recorded exact identities
        # may be touched. Unknown processes are deliberately left alone.
        for r in state.get('owned', []):
            notify(r, signal.SIGTERM)
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        _, latest = read_session(directory)
        if not session_live(latest):
            return True
        time.sleep(.05)
    return False


def validate(components):
    by_name = {c.name: c for c in components}
    if len(by_name) != len(components):
        raise ValueError('duplicate component names')
    resolved = set()
    ordered = []
    while len(resolved) < len(components):
        candidates = [c.name for c in components if c.name not in resolved and set(c.depends) <= resolved]
        if not candidates:
            raise ValueError('missing or cyclic component dependency')
        resolved.update(candidates)
        ordered.extend(by_name[name] for name in candidates)
    return ordered


def run_guarded(directory, components, health_factory=JsonHealth, startup_timeout=60.,
                fail_on_health_loss=False, health_loss_grace=1., cleanup_grace=16.):
    """Run in the foreground. Guardian survives scheduler death and reaps orphans.

    An unexpected guardian death delivers SIGTERM to the scheduler. Concurrent
    SIGKILL of both supervisors cannot be made recoverable without an external
    service manager; recorded identities remain available for manual stop.
    """
    components = validate(components)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    lock = (directory/'session.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('session is already active')
    _, previous = read_session(directory)
    if session_live(previous):
        raise RuntimeError('previous session still has live owned processes; stop it first')
    resource_locks = []
    resource_dir = directory.parent/'resources'
    resource_dir.mkdir(exist_ok=True)
    for resource in sorted({c.resource for c in components if c.resource}):
        if not resource.replace('_', '').isalnum():
            raise ValueError('invalid resource name')
        handle = (resource_dir/(resource+'.lock')).open('a')
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f'resource already owned by another session: {resource}')
        resource_locks.append(handle)
    run = Path(tempfile.mkdtemp(prefix='run-', dir=directory))
    guardian = identity(os.getpid())
    subreaper()
    state = {'run_directory': str(run), 'guardian': guardian, 'owner': None,
             'state': 'STARTING', 'reason': '', 'owned': [], 'components': {}}
    atomic_json(run/'state.json', state)
    atomic_json(directory/'current.json', {'run_directory': str(run)})
    owner = os.fork()
    if owner == 0:
        result = 2
        try:
            # Close the parent-death race: set the signal, then verify ppid.
            libc = ctypes.CDLL(None, use_errno=True)
            if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
                raise OSError(ctypes.get_errno(), 'PR_SET_PDEATHSIG failed')
            if os.getppid() != guardian['pid']:
                os._exit(2)
            subreaper()
            result = _schedule(run, state, components, health_factory, startup_timeout,
                               fail_on_health_loss, health_loss_grace, cleanup_grace)
        finally:
            os._exit(result)
    stopping = [False]
    stop_started = [None]
    def stop_guardian(*_):
        if not stopping[0]:
            stopping[0] = True
            stop_started[0] = time.monotonic()
            notify(identity(owner), signal.SIGTERM)
    old_handlers = {s: signal.signal(s, stop_guardian) for s in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]}
    try:
        while True:
            pid, status = os.waitpid(owner, os.WNOHANG)
            if pid:
                break
            if stop_started[0] is not None and time.monotonic()-stop_started[0] > cleanup_grace+5.:
                notify(identity(owner), signal.SIGKILL)
            time.sleep(.05)
        code = os.waitstatus_to_exitcode(status)
        # A dead scheduler's escaped/setsid children are now our descendants.
        leftovers = cleanup_tree(os.getpid(), grace=.2)
        latest = json.loads((run/'state.json').read_text())
        if code != 0:
            latest['state'] = 'FAILED'
            latest['reason'] = (latest.get('reason', '')+'; ' if latest.get('reason') else '') + f'scheduler exited {code}; guardian cleanup'
        latest['owned'] = leftovers
        latest['guardian_cleanup_complete'] = not leftovers
        atomic_json(run/'state.json', latest)
        return 0 if code == 0 and not leftovers else 2
    finally:
        for s, handler in old_handlers.items():
            signal.signal(s, handler)
        lock.close()
        for handle in resource_locks:
            handle.close()


def _schedule(run, state, components, health_factory, timeout, fail_on_loss, loss_grace, grace):
    stopping = [False]
    def request_stop(*_):
        stopping[0] = True
    for s in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]:
        signal.signal(s, request_stop)
    state['owner'] = identity(os.getpid())
    processes, records, logs = {}, {}, []
    health = health_factory(run/'health.json')
    started = time.monotonic()
    running_once = False
    preflight_passed = not any(c.name == 'probe' for c in components)
    lost_since = None
    code = 0
    def save():
        state['owned'] = descendants(os.getpid())
        state['updated_monotonic'] = time.monotonic()
        atomic_json(run/'state.json', state)
    def event(kind, **details):
        with (run/'events.jsonl').open('a') as f:
            f.write(json.dumps({'monotonic': time.monotonic(), 'kind': kind, **details})+'\n')
    try:
        print(f'SESSION_DIR={run}', flush=True)
        while not stopping[0]:
            checks = health()
            if not preflight_passed and checks.get('probe_live'):
                occupied = sorted({key for c in components if c.critical for key in c.start_requires
                                   if not checks.get(key, False)})
                if occupied:
                    raise RuntimeError('preflight rejects occupied interface: '+', '.join(occupied))
                preflight_passed = True
            ready = {}
            for c in components:
                p = processes.get(c.name)
                info = state['components'].setdefault(c.name, {'state': 'WAITING'})
                if p is None:
                    dependencies = all(ready.get(n, False) for n in c.depends)
                    prerequisites = all(checks.get(k, False) for k in (*c.requires, *c.start_requires))
                    if dependencies and prerequisites and (preflight_passed or c.name == 'probe'):
                        argv = list(c.argv)
                        log = (run/(c.name+'.log')).open('xb'); logs.append(log)
                        env = dict(os.environ, ROS_LOG_DIR=str(run/'ros_logs'),
                                   RANGER_SESSION_HEALTH_FILE=str(run/'health.json'))
                        p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                             env=env, start_new_session=True)
                        processes[c.name] = p; records[c.name] = identity(p.pid)
                        info.update(state='STARTING', process=records[c.name], argv=argv)
                        event('started', component=c.name, pid=p.pid)
                if p is not None:
                    if p.poll() is not None:
                        info.update(state='EXITED', exit_code=p.returncode)
                        ready[c.name] = False
                        if c.critical:
                            raise RuntimeError(f'{c.name} exited {p.returncode}; no automatic restart')
                    else:
                        ready[c.name] = (c.ready_key == 'process' or bool(checks.get(c.ready_key))) and all(
                            checks.get(key, False) for key in c.requires)
                        info['state'] = 'READY' if ready[c.name] else 'STARTING'
            critical_ready = all(ready.get(c.name, False) for c in components if c.critical)
            if critical_ready:
                running_once = True; lost_since = None
                state['state'] = 'RUNNING' if all(ready.get(c.name, False) for c in components) else 'DEGRADED'
            elif running_once:
                state['state'] = 'DEGRADED'
                lost_since = lost_since or time.monotonic()
                if fail_on_loss and time.monotonic()-lost_since >= loss_grace:
                    raise RuntimeError('critical health lost; stopping session, fresh start required')
            elif time.monotonic()-started > timeout:
                raise RuntimeError('startup readiness timeout')
            save()
            time.sleep(.1)
    except Exception as exc:
        code = 2
        state['reason'] = f'{type(exc).__name__}: {exc}'
        event('failure', reason=state['reason'])
    finally:
        state['state'] = 'STOPPING'; save()
        leftovers = cleanup_tree(os.getpid(), records.values(), grace)
        for p in processes.values():
            try:
                p.wait(timeout=.1)
            except (subprocess.TimeoutExpired, ChildProcessError):
                pass
        for log in logs:
            log.close()
        state['state'] = 'STOPPED' if code == 0 and not leftovers else 'FAILED'
        if leftovers:
            code = 2; state['reason'] += '; live descendants remain'
        save()
        event('stopped', exit_code=code, leftovers=state['owned'])
    return code
