"""Validation of the published `/transport/track` + `/transport/route_segment` route.

The Ubuntu checker and the portable tests share this logic.  `EDGES` from
`four_room_layout` is the authority for which track a selected segment belongs
to, so a `rung_in_*` segment that legitimately publishes `TRACK_2` is not
mistaken for a service excursion, and a pair that arrives as two asynchronous
transient-local samples is never read as a segment/track mismatch.
"""
from .four_room_layout import EDGES, ROOMS, route_plan

# The two diagnostics are independent transient-local samples.  Only treat them
# as one observation when both halves arrived inside this simulation-time window.
PAIR_WINDOW = .5


class RouteWatch:
    """Record the published route pairs and validate the ordered mission route."""

    def __init__(self, plan=None):
        self.plan = tuple(route_plan() if plan is None else plan)
        self.pairs = []

    def record(self, track, segment, track_stamp, segment_stamp):
        """Record one observation of both diagnostics, pairing asynchronously."""
        if not track or not segment:
            return
        if track_stamp is None or segment_stamp is None:
            return
        if abs(track_stamp-segment_stamp) > PAIR_WINDOW:
            # Only one half of an asynchronous pair has been delivered so far.
            return
        # A new segment and the previous track may arrive within one tick.
        # Wait for a consistent pair; never latch this transient combination.
        if segment in EDGES and EDGES[segment].track != str(track):
            return
        pair = (segment, str(track))
        if not self.pairs or self.pairs[-1] != pair:
            self.pairs.append(pair)

    @property
    def segments(self):
        return [segment for segment, _ in self.pairs]

    @property
    def tracks(self):
        return {track for _, track in self.pairs}

    def errors(self):
        """Return the ordered-route problems found in the recorded pairs."""
        errors = []
        for segment, track in self.pairs:
            if segment not in EDGES:
                errors.append('Unknown route segment published: ' + segment)
            elif EDGES[segment].track != track:
                errors.append(f'{segment} was published on {track}, '
                              f'expected {EDGES[segment].track}')
        segments = self.segments
        if segments != list(self.plan):
            missing = [name for name in self.plan if name not in segments]
            if missing:
                errors.append('Route never selected: ' + ', '.join(missing))
            else:
                wrong = [name for name, expected in zip(segments, self.plan)
                         if name != expected]
                extra = segments[len(self.plan):]
                errors.append('Route segments were selected out of mission order: ' +
                              (', '.join(wrong+extra) or 'unknown'))
        if self.tracks != {'TRACK_1', 'TRACK_2'}:
            errors.append('The mission did not select both tape tracks: ' +
                          str(sorted(self.tracks)))
        service = [name for name in segments
                   if name in EDGES and EDGES[name].kind == 'service']
        expected_service = [f'service_{room.number:02d}' for room in ROOMS]
        if service != expected_service:
            errors.append('Track 2 service excursions were not one per glovebox in order: ' +
                          str(service))
        if not {'main_tail', 'main_home'} <= set(segments):
            errors.append('The mission never returned home along the Track 1 tail')
        else:
            tail = segments[segments.index('main_tail'):]
            if tail != ['main_tail', 'main_home']:
                errors.append('The return home did not run the Track 1 tail in order: ' +
                              str(tail))
            elif any(EDGES[name].track != 'TRACK_1' for name in tail):
                errors.append('The return home left Track 1: ' + str(tail))
        return errors
