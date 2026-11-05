"""Export acceptance: a machine-ready program is released only when every
non-clear verification result has been accepted on purpose.

Posting already refuses output that does not match the planned motion. The
stream's geometric checks (scene bodies such as fixtures, stock contact,
material) report per move; this gate turns their violations and uncertain
results into a refusal unless the caller accepts that kind explicitly.
"""

from __future__ import annotations

ACCEPTABLE = frozenset(("uncertain", "violation"))


class ExportRefused(ValueError):
    """The program has unaccepted verification findings; nothing was written.

    ``findings``: the non-clear results (each with its move, status and
    detail). ``statuses``: the result kinds that were not accepted."""

    def __init__(self, findings, statuses):
        self.findings = tuple(findings)
        self.statuses = tuple(sorted(statuses))
        first = self.findings[0] if self.findings else {}
        detail = next(iter(first.get("findings") or ()), {})
        detail = detail.get("finding", detail)
        super().__init__(
            f"export refused: {len(self.findings)} move(s) with unaccepted "
            f"{'/'.join(self.statuses)} verification results (first at move {first.get('completed')}: "
            f"{detail.get('kind')} {detail.get('object_id')}); fix the job, or pass "
            f"accept={{{', '.join(repr(s) for s in self.statuses)}}} to release it anyway"
        )


def accepted(accept):
    """Validate an ``accept`` argument: a set drawn from uncertain/violation."""
    kinds = frozenset(accept or ())
    if not kinds <= ACCEPTABLE:
        raise ValueError(f"accept takes {sorted(ACCEPTABLE)}, not {sorted(kinds - ACCEPTABLE)}")
    return kinds


def gate(findings, accept):
    """Raise ExportRefused when any finding's status is not accepted."""
    kinds = accepted(accept)
    refused = [f for f in findings if f["status"] not in kinds]
    if refused:
        raise ExportRefused(refused, {f["status"] for f in refused})


def material_status(report):
    """A material report's export status: a definite gouge is a violation; a
    possible gouge or an incomplete analysis is uncertain. Leftover material
    is reported, not refused (a roughing setup leaves its allowance)."""
    gouge = report["gouge"]
    if gouge["definite_cells"]:
        return "violation"
    if gouge["possible_only_cells"] or not report["complete"]:
        return "uncertain"
    return "clear"


def unchecked(verification, *, turning=False):
    """The geometric checks this setup did not configure (so not run).

    A lathe setup (``turning``) runs all of its checks always: every move is
    checked as it is planned (and a posted program as it is replayed), the
    insert against the part, bore and chuck, the holder and rapids against the
    stock, and the stock is compared with the part (material)."""
    if turning:
        return ()
    config = verification or {}
    missing = []
    if not config.get("scene"):
        missing.append("scene")
    if not config.get("stock_contact"):
        missing.append("stock_contact")
    if not config.get("material"):
        missing.append("material")
    return tuple(missing)
