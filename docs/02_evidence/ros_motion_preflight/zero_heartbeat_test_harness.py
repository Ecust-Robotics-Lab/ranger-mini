def exercise(phase, exited=False):
    heartbeat_next = 0.0
    events = []
    class Clock:
        now = 0.0
        def monotonic(self): return self.now
        def sleep(self, seconds): self.now += seconds
    time = Clock()
    class Child:
        def poll(self): return 1 if exited else None
    child = Child()
    publisher = object()
    node = object()
    class Ros:
        def spin_once(self, *args, **kwargs): pass
    rclpy = Ros()
    def drain_can(): pass
    def publish(v): events.append((time.monotonic(), v))
    def pump(duration):
        nonlocal heartbeat_next
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if phase == 'static' and child is not None and (child.poll() is None) and (publisher is not None) and (time.monotonic() >= heartbeat_next):
                publish(0.0)
                heartbeat_next = time.monotonic() + 0.02
            rclpy.spin_once(node, timeout_sec=0.001)
            drain_can()
            time.sleep(0.002)
    pump(0.2)
    return events

