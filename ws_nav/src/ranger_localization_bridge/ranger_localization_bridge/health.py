"""One readiness policy shared by odometry callbacks and the watchdog."""
import math


def fresh(now, stamp, max_age):
    return stamp is not None and math.isfinite(stamp) and 0 < stamp <= now and now - stamp <= max_age


def readiness(now, sample, received, odom_stamp, pose_max_age, odom_max_age,
              heartbeat_max_age, innovation_ok):
    checks = {
        'odom_present': fresh(now, odom_stamp, odom_max_age),
        'odom_age': fresh(now, odom_stamp, odom_max_age),
        'loc_tf': sample is not None,
        'loc_tf_age_ok': sample is not None and fresh(now, sample.header.stamp.sec +
                            sample.header.stamp.nanosec * 1e-9, pose_max_age),
        'backend_alive': fresh(now, received, heartbeat_max_age),
        'backend_valid': sample is not None and sample.state == sample.VALID,
        'innovation': innovation_ok,
    }
    return all(checks.values()), checks
