"""Контроллер склада: устойчивое следование пути после смены целевой точки."""


def configure_warehouse_controller(params):
    # DWB can choose a stationary local minimum at the warehouse aisle turn.
    # RPP rotates towards the path, then follows it; goal checkers still choose
    # position/orientation/proximity and retain their separate tolerances.
    params['controller_server']['ros__parameters']['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': 0.15,
        'lookahead_dist': 0.4,
        'min_lookahead_dist': 0.3,
        'max_lookahead_dist': 0.6,
        'lookahead_time': 1.5,
        'use_velocity_scaled_lookahead_dist': True,
        'use_rotate_to_heading': True,
        'rotate_to_heading_angular_vel': 0.5,
        'rotate_to_heading_min_angle': 0.5,
        'max_angular_accel': 3.2,
    }
