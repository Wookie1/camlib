# Saving and reopening completed runs

The core library can save native and checked posted histories without an
application, CAD package or server. Saving requires a completed run: call `wait()`
first. The destination must not already exist.

```python
from camlib.playback import RecordedRun, Player, Cursor

with job.record("top", rapid_mm_min=10000) as run:  # your machine's rapid rate
    run.wait()
    run.save("saved-pocket")

# A later process needs only camlib and the saved directory.
with RecordedRun.open("saved-pocket") as saved:
    player = Player(saved, compact=True)
    player.seek(Cursor(saved.status()["available"]))
    print(player.state.volume())
```

`job.post(...)` returns a run with the same `save` method. `RecordedRun.open`
automatically restores a native or posted view; `PostedRun.open` requires a posted
snapshot. Posted views expose the original `nc_lines`, `posted_motion`, controller
configuration, source mappings and verification reports. Reopening does not plan,
post, or start a producer. Fractional seeking uses the normal native read-only
stock query worker. Closing a saved view leaves its snapshot intact.

The copied snapshot is independent of the producer's temporary directory. Files
are flushed before the `run.json` completion marker is committed. A failed save
removes its new destination; existing directories are never overwritten. A crash
before completion can leave an incomplete directory, which `open` rejects. The
caller can remove that incomplete directory and retry.

Both methods accept `cancel`, a callable returning true to cancel, and
`max_bytes`, defaulting to 8 GiB including metadata. Copying and checksum checks
use bounded buffers. The file inventory is limited to 16,384 entries and metadata
to 4 MiB. A cancelled or failed live run is not a completed snapshot.

Snapshot format version 1 wraps the existing canonical manifest, motion, control,
verification, checkpoint and optional checked-post journals. Reopening validates
file sizes, hashes, metadata checksum, inventory, completion identity, counts and
post/stock binding.
Existing journal readers continue to validate frames and attribution during
playback. The format records no source checkout or native executable path; an
optional `executable=` override applies only to the current reader.

Hashes protect the integrity of owned saved output; they do not authenticate
arbitrary imported proofs or establish controller execution. Keep snapshots
immutable while open. Saving and reopening preserve uncertainty and failures in
machining reports; completion does not turn those reports into a clearance claim.
