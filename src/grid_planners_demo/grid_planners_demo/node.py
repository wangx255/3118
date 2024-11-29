import math
from geometry_msgs.msg import Pose

class Node:
    def __init__(self, x=0.0, y=0.0, z=0.0, orientation=None):
        self.x = x
        self.y = y
        self.z = z
        self.orientation = orientation if orientation else [0.0, 0.0, 0.0, 1.0]
        self.parent = None
        self.g = math.inf
        self.h = 0.0
        self.f = math.inf

    @staticmethod
    def from_pose(pose: Pose) -> 'Node':
        if pose is None:
            raise ValueError("Pose is None")
        
        return Node(
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            orientation=[
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
        )

    @staticmethod
    def from_tf(position: list, orientation: list) -> 'Node':
        if not position or not orientation:
            raise ValueError("Position or Orientation is None")
        
        return Node(
            x=position[0],
            y=position[1],
            z=position[2],
            orientation=orientation
        )

    def to_pose_stamped(self) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose.position.x = self.x
        pose_stamped.pose.position.y = self.y
        pose_stamped.pose.position.z = self.z
        pose_stamped.pose.orientation.x = self.orientation[0]
        pose_stamped.pose.orientation.y = self.orientation[1]
        pose_stamped.pose.orientation.z = self.orientation[2]
        pose_stamped.pose.orientation.w = self.orientation[3]
        return pose_stamped

    def calculate_distance(self, other: 'Node') -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def generate_neighbors(self, map_resolution: float) -> list:
        """Generate 8-connected neighbors based on map resolution"""
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),          (0, 1),
            (1, -1),  (1, 0), (1, 1)
        ]
        neighbors = []
        for dx, dy in directions:
            neighbor = Node(
                x=self.x + dx * map_resolution,
                y=self.y + dy * map_resolution,
                z=self.z,
                orientation=self.orientation.copy()
            )
            neighbors.append(neighbor)
        return neighbors

    def backtrack_path(self) -> list:
        """Reconstruct path from goal to start"""
        path = []
        node = self
        while node is not None:
            path.append(node)
            node = node.parent
        path.reverse()
        return path

    def get_key(self) -> tuple:
        """Generate a key with rounded coordinates to avoid floating point precision issues"""
        return (round(self.x, 4), round(self.y, 4))

    def __hash__(self):
        return hash(self.get_key())

    def __eq__(self, other):
        return isinstance(other, Node) and self.get_key() == other.get_key()

