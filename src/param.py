params = {
  "in_bounds": {
    "x_bounds": [
      -1.0, 
      1.0
    ], 
    "y_bounds": [
      -1.2, 
      1.2
    ]
  }, 
  "noise_sigma": 0.03, 
  "min_dist": 0.02, 
  "_args": {
    "noise_sigma": 0.03, 
    "min_dist": 0.03, 
    "robot_max_speed": 0.03, 
    "t_step": 2, 
    "t_future": 1000, 
    "t_past": -100
  }, 
  "initial_robot_state": {
    "x": 0.0, 
    "y": -1.0, 
    "vx": 0, 
    "vy": 0.04, 
    "max_speed": 0.02
  }, 
  "goal_bounds": {
    "x_bounds": [
      -1.0, 
      1.0
    ], 
    "y_bounds": [
      1.0, 
      1.2
    ]
  },
  'static_obstacles': {
      'pos': [],
      'radius': []
  }
}
