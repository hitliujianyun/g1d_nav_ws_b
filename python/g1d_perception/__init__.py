"""Camera person segmentation fused with Mid-360S points (P3 of docs/DYNAMIC_OBJECTS_STUDY.md).

Head camera frame -> person masks (YOLO-seg on PC2) -> LiDAR points inside the masks ->
rt/nav/person_points (short-lived costmap layer) and rt/nav/map_delete (late removal from the
FAST-LIO map). Runs beside the safety loop: the costmap already reacts to raw returns and
M-detector labels; camera evidence only makes moving people disappear faster and cleaner.
"""

__version__ = "0.1.0"
