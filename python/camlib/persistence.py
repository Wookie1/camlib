"""Completed, owned run snapshots without an application or CAD dependency.

Hashes detect damaged or mixed files; they do not authenticate imported proofs.
The snapshot format retains the existing motion/checkpoint/post journal formats.
"""

from copy import deepcopy
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import shutil
import struct
import threading
import os

from .playback import RecordedRun
from .posted import PostedRun, _CHUNK, _MOVE
from .simulation import Frame, FRAME_LIMIT, MANIFEST_LIMIT, ProtocolError, SimulationError, VerificationStream, _Decoder

_META_LIMIT = 4 * 1024**2
_FILE_LIMIT = 16384
_CORE = {
    'manifest.bin',
    'end.bin',
    'motions.bin',
    'motions.idx',
    'checkpoints.idx',
    'controls.bin',
    'verification.bin',
    'verification.idx',
}
_POST = {'post-manifest.json', 'post-chunks.bin', 'post-chunks.idx', 'post-motions.idx'}
_CHECKPOINT = re.compile(r'[0-9a-f]{16}\.checkpoint\.gz\Z')


def _cancelled(cancel):
    if cancel is not None and cancel():
        raise SimulationError('saved run operation cancelled')


def _sync_directory(path):
    if os.name != 'nt':
        descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def _digest(path, cancel, destination=None):
    digest = hashlib.sha256()
    size = 0
    with path.open('rb') as source:
        while block := source.read(1024**2):
            _cancelled(cancel)
            size += len(block)
            digest.update(block)
            if destination is not None:
                destination.write(block)
    return size, digest.hexdigest()


def _metadata_bytes(saved):
    return json.dumps(saved, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def _bounded_integer(value, name):
    if type(value) is not int or not 0 <= value < 2**63:
        raise ProtocolError('invalid saved ' + name)
    return value


class SavedRecordedRun(RecordedRun):
    """Read view of a completed journal. Opening never starts a producer."""

    def __init__(self, directory, saved, *, executable=None):
        self.directory = Path(directory)
        self._condition = threading.Condition()
        self._cancel = threading.Event()
        self._closed = False
        self._preview_session = None
        self._config = dict(executable=executable)
        self._saved_status = deepcopy(saved['status'])
        self._available = _bounded_integer(self._saved_status['available'], 'motion count')
        self._finished, self._error = True, None
        if (
            self._saved_status['finished'] is not True
            or self._saved_status.get('cancelled')
            or self._saved_status.get('error')
            or self._saved_status.get('producing')
        ):
            raise ProtocolError('saved journal has no complete successful finish')
        path = self.directory / 'manifest.bin'
        if path.stat().st_size > MANIFEST_LIMIT + 20:
            raise ProtocolError('oversized saved manifest')
        self._manifest_frame = Frame.parse(path.read_bytes(), MANIFEST_LIMIT + 20)
        if self._manifest_frame.kind != 3:
            raise ProtocolError('saved journal needs a canonical manifest')
        manifest = json.loads(self._manifest_frame.payload.tobytes())
        self._control_bytes = (self.directory / 'controls.bin').stat().st_size
        self._verification_count = _bounded_integer(
            saved['verification'].get('available_records', 0), 'verification count'
        )
        self._verification_coverage = deepcopy(saved['verification'].get('coverage'))
        self._verification_reader = (
            VerificationStream(self._manifest_frame) if self._verification_coverage is not None else None
        )
        self._stock_identity = (
            manifest['run']['id'],
            manifest['run']['revision'],
            hashlib.sha256(self._manifest_frame.payload).digest(),
        )
        self._verification_identity = self._stock_identity

    def status(self):
        return dict(self._saved_status, producing=False, cancelled=self._cancel.is_set(), replayed_saved_output=True)

    def wait(self, timeout=30):
        self.wait_ready()
        return self.status()

    def wait_stopped(self, timeout=5):
        return True

    def cancel(self):
        self._cancel.set()

    def close(self):
        self._cancel.set()
        with self._condition:
            query, self._preview_session = self._preview_session, None
            self._closed = True
        if query is not None:
            query.close()

    def wait_ready(self, timeout=5):
        if self._closed:
            raise SimulationError('saved replay view closed')
        self._raise_if_stopped()
        return self._manifest_frame


class SavedPostedRun(SavedRecordedRun, PostedRun):
    """Read view retaining checked output, source attribution and stock binding."""

    def __init__(self, directory, saved, *, executable=None):
        super().__init__(directory, saved, executable=executable)
        path = self.directory / 'post-manifest.json'
        if path.stat().st_size > 2 * 1024**2:
            raise ProtocolError('oversized saved post manifest')
        raw = path.read_bytes()
        post = json.loads(raw)
        self._post_binding = hashlib.sha256(raw).digest()
        self._post_run_id, self._post_identity = post['run_id'], post['identity']
        self._output_tolerance_mm = post['definition']['machine']['tolerance_mm']
        status = saved['posting']
        self._post_count = _bounded_integer(status['checked_motions'], 'posted count')
        self._post_chunks = _bounded_integer(status['chunks'], 'post chunk count')
        self._post_lines = _bounded_integer(status['lines'], 'NC line count')
        self._post_largest, self._post_finished = status['max_chunk_bytes'], True
        self._post_scheduling = deepcopy(status.get('motion_scheduling'))
        self._post_arc_output = deepcopy(status.get('arc_output'))
        if status['finished'] is not True or self._post_count != self._available:
            raise ProtocolError('saved journal has no complete checked finish')
        if status['run_id'] != self._post_run_id or status['identity'] != self._post_identity:
            raise ProtocolError('saved post identity mismatch')
        self.posting_manifest()

    def status(self):
        return dict(super().status(), posting=self.posting_status())


def _layout(run, names):
    """Check the completion/index envelope without rerunning machining."""
    root = run.directory
    status = run.status()
    checkpoints = _bounded_integer(status['checkpoints'], 'checkpoint count')
    if (root / 'motions.idx').stat().st_size != run._available * 32:
        raise ProtocolError('saved motion count/index mismatch')
    if (root / 'verification.idx').stat().st_size != run._verification_count * 48:
        raise ProtocolError('saved verification count/index mismatch')
    if not checkpoints or (root / 'checkpoints.idx').stat().st_size != checkpoints * 8:
        raise ProtocolError('saved checkpoint count/index mismatch')
    expected = set(_CORE)
    previous = -1
    with (root / 'checkpoints.idx').open('rb') as stream:
        for i in range(checkpoints):
            (value,) = struct.unpack('<Q', stream.read(8))
            if (i == 0 and value != 0) or not previous < value <= run._available:
                raise ProtocolError('invalid saved checkpoint ordering')
            if value:
                expected.add(f'{value:016x}.checkpoint.gz')
            previous = value
    if previous != run._available:
        raise ProtocolError('saved final checkpoint is missing')
    path = root / 'end.bin'
    if path.stat().st_size > FRAME_LIMIT:
        raise ProtocolError('oversized saved finish')
    end = Frame.parse(path.read_bytes())
    manifest = run.wait_ready()
    identity = json.loads(manifest.payload.tobytes())['run']
    identity = (identity['id'], identity['revision'], hashlib.sha256(manifest.payload).digest())
    decoder = _Decoder(end.payload)
    if (
        end.kind != 4
        or (decoder.string(128), decoder.string(512), decoder.take(32).tobytes()) != identity
        or decoder.number('<Q') != run._available
    ):
        raise ProtocolError('saved finish identity/count mismatch')
    seconds = status['seconds']
    if not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or seconds < 0:
        raise ProtocolError('invalid saved duration')
    if run._available and run.motion_time(run._available - 1)[1] != seconds:
        raise ProtocolError('saved duration/index mismatch')
    if isinstance(run, PostedRun):
        expected |= _POST
        if (root / 'post-chunks.idx').stat().st_size != run._post_chunks * _CHUNK.size or (
            root / 'post-motions.idx'
        ).stat().st_size != run._available * _MOVE.size:
            raise ProtocolError('saved post count/index mismatch')
        if not run._post_chunks:
            raise ProtocolError('missing saved post finish')
        final, _, _ = run._read_post(run._post_chunks - 1)
        if final['lines'][-1]['number'] != run._post_lines:
            raise ProtocolError('saved NC line count mismatch')
    if names != expected:
        raise ProtocolError('saved journal file inventory mismatch')


def save_run(run, destination, *, cancel=None, max_bytes=8 * 1024**3):
    """Copy a finished run; publish run.json last. Existing paths are never replaced.

    A failed copy removes only the newly created destination. Keep the completed
    snapshot immutable while readers use it. Cancellation is a callable predicate.
    """
    _cancelled(cancel)
    if not isinstance(run, RecordedRun):
        raise TypeError('save requires a retained run')
    status = run.status()
    if not status['finished'] or status['producing'] or status['cancelled'] or status['error']:
        raise SimulationError('only a finished, stopped run can be saved; call wait first')
    if type(max_bytes) is not int or max_bytes < 1024:
        raise ValueError('saved run byte budget must be at least 1024')
    destination = Path(destination)
    if destination.resolve().is_relative_to(run.directory.resolve()):
        raise ValueError('saved destination must be outside the active journal')
    posted = isinstance(run, PostedRun)
    saved = dict(
        version=1, kind='posted' if posted else 'native', status=status, verification=run.verification_status()
    )
    if posted:
        saved['posting'] = run.posting_status()
    source = sorted(itertools.islice((p for p in run.directory.iterdir() if p.name != 'run.json'), _FILE_LIMIT + 1))
    if len(source) > _FILE_LIMIT:
        raise SimulationError('saved file inventory exceeds 16384 entries')
    names = {p.name for p in source}
    if any(
        p.is_symlink() or not p.is_file() or not (p.name in _CORE | _POST or _CHECKPOINT.fullmatch(p.name))
        for p in source
    ):
        raise ProtocolError('unexpected saved journal file')
    _layout(run, names)
    if sum(p.stat().st_size for p in source) > max_bytes:
        raise SimulationError('saved run exceeds byte budget')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.mkdir()  # Exclusive reservation, including existing empty directories.
    try:
        inventory = []
        total = 0
        for path in source:
            _cancelled(cancel)
            with (destination / path.name).open('xb') as output:
                size, digest = _digest(path, cancel, output)
                total += size
                if total > max_bytes:
                    raise SimulationError('saved run exceeds byte budget')
                output.flush()
                os.fsync(output.fileno())
            inventory.append(dict(path=path.name, bytes=size, sha256=digest))
        saved['integrity'] = inventory
        saved['metadata_sha256'] = hashlib.sha256(_metadata_bytes(saved)).hexdigest()
        raw = _metadata_bytes(saved)
        if len(raw) > _META_LIMIT or total + len(raw) > max_bytes:
            raise SimulationError('saved run metadata/byte budget exceeded')
        with (destination / 'run.writing').open('xb') as output:
            output.write(raw)
            output.flush()
            os.fsync(output.fileno())
        _cancelled(cancel)
        (destination / 'run.writing').replace(destination / 'run.json')
        _sync_directory(destination)
        _sync_directory(destination.parent)
        return destination
    except BaseException:
        shutil.rmtree(destination)
        raise


def open_run(directory, *, executable=None, cancel=None, max_bytes=8 * 1024**3):
    """Validate a completed snapshot and open a read-only replay view.

    This checks owned-file integrity, not provenance/authenticity of arbitrary
    imported output. Whole-file hashing is bounded in memory; no planner runs.
    """
    _cancelled(cancel)
    directory = Path(directory)
    if type(max_bytes) is not int or max_bytes < 1024:
        raise ValueError('saved run byte budget must be at least 1024')
    meta = directory / 'run.json'
    if directory.is_symlink() or meta.is_symlink() or not meta.is_file() or meta.stat().st_size > _META_LIMIT:
        raise ProtocolError('missing/oversized saved run metadata')
    view = None
    try:
        saved = json.loads(meta.read_bytes())
        digest = saved.pop('metadata_sha256')
        if digest != hashlib.sha256(_metadata_bytes(saved)).hexdigest():
            raise ProtocolError('saved metadata checksum mismatch')
        if type(saved['version']) is not int or saved['version'] != 1 or saved['kind'] not in ('native', 'posted'):
            raise ProtocolError('unsupported saved run format')
        inventory = saved['integrity']
        if not isinstance(inventory, list) or not len(_CORE) <= len(inventory) <= _FILE_LIMIT:
            raise ProtocolError('invalid saved file inventory')
        names = set()
        total = meta.stat().st_size
        for item in inventory:
            _cancelled(cancel)
            name = item['path']
            if (
                not isinstance(name, str)
                or name in names
                or not (name in _CORE | _POST or _CHECKPOINT.fullmatch(name))
                or type(item['bytes']) is not int
                or item['bytes'] < 0
            ):
                raise ProtocolError('invalid saved file entry')
            names.add(name)
            total += item['bytes']
            if total > max_bytes:
                raise SimulationError('saved run exceeds byte budget')
            path = directory / name
            if path.is_symlink() or not path.is_file() or path.stat().st_size != item['bytes']:
                raise ProtocolError('saved journal size/type mismatch')
            if _digest(path, cancel) != (item['bytes'], item['sha256']):
                raise ProtocolError('saved journal checksum mismatch')
        if {p.name for p in itertools.islice(directory.iterdir(), _FILE_LIMIT + 2)} != names | {'run.json'}:
            raise ProtocolError('saved journal file inventory mismatch')
        cls = SavedPostedRun if saved['kind'] == 'posted' else SavedRecordedRun
        view = cls(directory, saved, executable=executable)
        _layout(view, names)
        _cancelled(cancel)
        return view
    except (KeyError, TypeError, ValueError, struct.error) as error:
        if view is not None:
            view.close()
        raise ProtocolError('invalid saved run metadata: ' + str(error)) from error
    except BaseException:
        if view is not None:
            view.close()
        raise
