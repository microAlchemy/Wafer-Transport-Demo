from wafer_transport_sim.route_watch import RouteWatch
from wafer_transport_sim.four_room_layout import EDGES, route_plan


def test_valid_route_accepts_async_track_updates():
    watch = RouteWatch()
    previous = 'TRACK_1'
    for i, name in enumerate(route_plan()):
        track = EDGES[name].track
        watch.record(previous, name, i-.05, i)
        watch.record(track, name, i, i)
        previous = track
    assert watch.errors() == []


def test_missing_or_reordered_service_fails():
    plan = list(route_plan())
    for changed in (plan[:2]+plan[3:], plan[:2]+[plan[6]]+plan[3:]):
        watch = RouteWatch()
        for i, name in enumerate(changed):
            watch.record(EDGES[name].track, name, i, i)
        assert watch.errors()


def test_missing_home_or_service_on_return_fails():
    plan = list(route_plan())
    for changed in (plan[:-2], plan+['service_01']):
        watch = RouteWatch()
        for i, name in enumerate(changed):
            watch.record(EDGES[name].track, name, i, i)
        assert watch.errors()
