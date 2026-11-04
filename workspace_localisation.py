"""Workspace localisation adapter: Codes 04, 05 and 06 on one independent scan.

No mathematics lives here. Each job calls the SAME public function, with the
SAME prepared inputs, as the corresponding desktop page (desktop_app.py):

  grid    Code 04  LocalisationPage.align + AutoAlignViewer ("new" mode)
                   points = filter_ranges(points_local); config = GridSearchConfig();
                   global: localise_global(grid, points, progress, should_stop)
                   local:  localise_scan(grid, points, initial, config), where the
                           guess goes through the viewer's sliders (clipped to
                           the grid extent, heading via degrees). NOT cancellable.
  vector  Code 05  VectorLocalisationPage.compute: localise_vector(vmap, raw
                   points, initial | None, progress, should_stop)
  scans   Code 06  ScanLocalisationPage.compute: localise_on_scans(raw points,
                   reference_scans(...), initial | None, progress, should_stop, mode)

References are supplied by the caller from the session's shared caches (the
"grid" build's GridMap, the page-05 "vector_reference" VectorMap, the session's
visible scans) and are never modified. No tkinter, serial or file writes.
"""
from dataclasses import dataclass
import math

import numpy as np

import localisation_common as common
from app_session import grid_localisation as loc, vector_localisation as vloc, \
    scan_localisation as sloc

METHODS = {"grid": "04 Grid", "vector": "05 Vector", "scans": "06 Scan"}
SEARCHES = {"global": "Global: whole map, 360°", "local": "Local: ±1 m / ±30° around a guess"}
SCAN_REFERENCES = {"accumulated": "All positioned scans (accumulated)",
                   "single": "One recorded scan (strict)"}
VECTOR_REFERENCE_KEY = "vector_reference"  # SharedSession.cache key shared with page 05
# Each method's score is defined by that method and keeps its own thresholds:
# never compare scores across methods.
SCORE_LABELS = {"grid": "Code 04 grid score", "vector": "Code 05 vector score",
                "scans": "Code 06 scan score"}
# desktop_app.verdict(): only "ok" is accepted, "weak" is plausible but not
# confirmed, every other status (low_score, ambiguous, out_of_map,
# inconsistent, at_search_limit, failed, cancelled) is not localised.
ACCEPTED, PLAUSIBLE = "ok", "weak"


@dataclass(frozen=True)
class Request:
    """Everything that determines one localisation. Equal requests on the same
    reference revision give the same result; any change makes a result stale."""
    method: str                      # key of METHODS
    search: str                      # key of SEARCHES
    points_local: np.ndarray         # N x 2 sensor-frame metres, read-only copy
    target_label: str
    guess_day1: tuple = None         # (x_m, y_m, heading_deg); local search only
    scan_reference: str = "accumulated"
    reference_id: int = None         # Code 06 "single" reference scan id
    reference_revision: int = 0      # SharedSession.revision of the reference

    def __post_init__(self):
        if self.method not in METHODS:
            raise ValueError(f"Unknown method {self.method!r}.")
        if self.search not in SEARCHES:
            raise ValueError(f"Unknown search {self.search!r}.")
        if self.search == "local" and self.guess_day1 is None:
            raise ValueError("A local search needs an initial pose.")
        if self.method == "scans" and self.scan_reference not in SCAN_REFERENCES:
            raise ValueError(f"Unknown scan reference {self.scan_reference!r}.")
        if self.method == "scans" and self.scan_reference == "single" and self.reference_id is None:
            raise ValueError("A single-scan reference needs a reference scan id.")
        points = np.array(self.points_local, dtype=float, copy=True)
        points.setflags(write=False)
        object.__setattr__(self, "points_local", points)
        if self.guess_day1 is not None:
            object.__setattr__(self, "guess_day1", tuple(float(v) for v in self.guess_day1))

    @property
    def signature(self):
        """Identity of the inputs: a finished result is shown only if it still
        matches the current request's signature."""
        return (self.method, self.search, self.guess_day1 if self.search == "local" else None,
                self.scan_reference if self.method == "scans" else None,
                self.reference_id if self.method == "scans" and self.scan_reference == "single"
                else None, loc.measurement_signature(self.points_local), self.reference_revision)


@dataclass(frozen=True)
class Outcome:
    """A finished localisation, with the method's own result kept unchanged."""
    request: Request
    result: common.LocalisationResult

    @property
    def status(self):
        return self.result.status

    @property
    def accepted(self):
        return self.result.status == ACCEPTED and self.result.pose is not None

    @property
    def plausible(self):
        return self.result.status == PLAUSIBLE and self.result.pose is not None

    @property
    def verdict_level(self):
        """'ok', 'weak' or 'bad', exactly as desktop_app.verdict()."""
        return {ACCEPTED: "ok", PLAUSIBLE: "weak"}.get(self.result.status, "bad")

    @property
    def pose(self):
        """(x, y, heading_rad) in the global frame; None when not localised."""
        return self.result.pose if self.accepted or self.plausible else None

    @property
    def pose_day1(self):
        """(x_m, y_m, heading_deg), or None when not localised."""
        return common.pose_to_day1(self.pose) if self.pose is not None else None

    @property
    def candidate_pose(self):
        """The method's best pose even when rejected (for a clearly marked,
        optional display), or None."""
        return self.result.pose

    @property
    def runner_up_pose(self):
        """Equally good alternative (radians), reported only for 'ambiguous'."""
        runner = self.result.details.get("runner_up_pose")
        if self.result.status == "ambiguous" and runner is not None:
            return common.pose_from_day1(runner)
        return None

    @property
    def score_label(self):
        return SCORE_LABELS[self.request.method]


def check_target(points, reference_scans):
    """None if `points` may be localised on a reference built from
    `reference_scans`, else the reason (the pages' own checks and wording)."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) \
            or not np.all(np.isfinite(points)):
        return "The new scan must contain finite local [x, y] points in metres."
    signature = loc.measurement_signature(points)
    duplicates = [scan["id"] for scan in reference_scans
                  if loc.measurement_signature(scan["points_local"]) == signature]
    if duplicates:
        return (f"These measurements already belong to reference scan(s) {duplicates}; "
                "localising them here would be self-matching. Use the leave-one-out "
                "evaluation on pages 04-06 instead.")
    return None


def load_targets(path):
    """Scans of a separate recording, local points only (source poses dropped).
    Raises ValueError with a readable message for a missing or invalid file."""
    try:
        return loc.load_recorded_targets(path)
    except (ValueError, loc.part2.SessionError) as error:
        raise ValueError(str(error)) from error


def cancellable(method, search):
    """Code 04's local search has no stop hook; it can only be made stale."""
    return not (method == "grid" and search == "local")


def grid_initial_pose(guess_day1, grid):
    """The guess exactly as Code 04's viewer hands it to localise_scan:
    AutoAlignViewer.set_initial_pose clips X/Y to the grid extent and stores
    the heading in degrees; initial_pose() converts back to radians."""
    pose = common.pose_from_day1(guess_day1)
    left, right, bottom, top = grid.extent
    x = float(np.clip(pose[0], left, right))
    y = float(np.clip(pose[1], bottom, top))
    theta_slider = math.degrees(common.wrap_angle(pose[2]))
    return x, y, common.wrap_angle(math.radians(theta_slider))


def grid_points(points_local):
    """Target points as Code 04's viewer prepares them (AutoAlignViewer.load_scan).
    The grid search also range-filters internally, so this keeps the inputs,
    not only the results, identical to the page."""
    return common.filter_ranges(points_local)


def scan_references(reference_scans, request):
    """Code 06 reference for the request (copied by reference_scans)."""
    if request.scan_reference == "single":
        return sloc.reference_scans(reference_scans, "single", request.reference_id)
    return sloc.reference_scans(reference_scans, "accumulated")


def make_job(request, reference, progress=None, should_stop=None):
    """A no-argument callable for a worker thread, returning an Outcome.

    reference: the GridMap ("grid"), VectorMap ("vector") or the session's
    visible scan list ("scans"), as the pages obtain them.
    """
    points, method = request.points_local, request.method
    local = request.search == "local"
    if method == "grid":
        grid = reference
        prepared = grid_points(points)
        if local:
            initial = grid_initial_pose(request.guess_day1, grid)
            config = loc.GridSearchConfig()
            run = lambda: loc.localise_scan(grid, prepared, initial, config)  # noqa: E731
        else:
            run = lambda: loc.localise_global(grid, prepared, progress=progress,  # noqa: E731
                                              should_stop=should_stop)
    elif method == "vector":
        initial = vloc.pose_from_day1(request.guess_day1) if local else None
        run = lambda: vloc.localise_vector(reference, points, initial,  # noqa: E731
                                           progress=progress, should_stop=should_stop)
    else:
        initial = vloc.pose_from_day1(request.guess_day1) if local else None
        references = scan_references(reference, request)
        mode = "single" if request.scan_reference == "single" else "accumulated"
        run = lambda: sloc.localise_on_scans(points, references, initial,  # noqa: E731
                                             progress=progress, should_stop=should_stop,
                                             mode=mode)
    return lambda: Outcome(request, run())
