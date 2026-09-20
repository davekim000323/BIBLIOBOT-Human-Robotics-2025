import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import cv2
import matplotlib.pyplot as plt
from collections import deque
from heapq import heappush, heappop
from itertools import permutations
import datetime
import os

class BibliobotLibraryNavigation:
    def __init__(self):
        # PyBullet 연결
        self.physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 시뮬레이션 설정
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.setTimeStep(1/240)
        
        # 물리 파라미터
        p.setPhysicsEngineParameter(
            numSolverIterations=10,
            numSubSteps=2,
            fixedTimeStep=1/240,
            contactBreakingThreshold=0.001
        )
        
        # Navigation 파라미터
        self.approach_distance = 0.8  # 책장 접근 거리
        self.safety_distance = 0.7    # 안전 거리 증가 (0.5 → 0.7)
        self.robot_radius = 0.4       # 로봇 반경
        self.robot_mass = 37.0        # 로봇 질량 37kg
        
        # 로봇 속도 파라미터
        self.max_linear_speed = 1.0   # 최대 직진 속도
        self.max_angular_speed = 0.75  # 최대 회전 속도
        
        # 확장된 도서관 크기
        self.library_width = 28.0     # 30미터
        self.library_height = 38.0    # 40미터
        
        # 실제 스케일 책장 크기
        self.shelf_scale = 2.5  # 책장 크기 스케일
        
        # 경로 기록
        self.path_history = []
        self.current_path = None
        self.orientation_arrows = []
        
        # Manipulator 제어
        self.current_shelf_level = 2  # 기본 2단
        self.target_shelf_level = 2
        self.current_manipulator_height = 0.7  # 현재 높이 추적
        self.at_shelf = False  # Platform Extension 제어용 플래그
        
        # 동역학 데이터 기록
        self.dynamics_data = {
            'time': [],
            'positions': [],
            'velocities': [],
            'accelerations': [],
            'torques': [],
            'joint_positions': [],
            'joint_velocities': [],
            'joint_torques': []
        }
        
        # 녹화 설정
        self.recording = False
        self.recording_frames = []
        self.vision_frames = []
        self.recording_start_time = None
        
        # 도서관 환경 설정
        self.setup_library_environment()
        
        # 로봇 로드
        self.load_robot()
        
        # 책장 생성
        self.setup_library_shelves()
        
        # 컴퓨터 비전 설정
        self.setup_robot_vision()
        
        # 비주얼라이저 설정
        self.setup_visualizer()
        
    def setup_library_environment(self):
        """짙은 회색 도서관 환경 생성"""
        print("🏛️ 도서관 환경 생성 중...")
        
        # 짙은 회색 바닥
        floor_color = [0.3, 0.3, 0.3, 1.0]
        floor_size = [self.library_width/2, self.library_height/2, 0.01]
        floor_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=floor_size,
            rgbaColor=floor_color
        )
        floor_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=floor_size
        )
        
        self.floor_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=floor_collision,
            baseVisualShapeIndex=floor_visual,
            basePosition=[0, 0, -0.01]
        )
        
        # 벽 생성
        self.create_library_walls()
        
    def create_library_walls(self):
        """짙은 회색 도서관 벽 생성"""
        wall_color = [0.25, 0.25, 0.25, 1.0]
        wall_height = 5.0
        wall_thickness = 0.3
        
        # 도서관 경계
        min_x = -self.library_width/2
        max_x = self.library_width/2
        min_y = -self.library_height/2
        max_y = self.library_height/2
        
        # 벽 정의
        walls = [
            # 왼쪽 벽
            {'pos': [min_x - wall_thickness/2, 0, wall_height/2], 
             'size': [wall_thickness/2, self.library_height/2, wall_height/2]},
            # 오른쪽 벽
            {'pos': [max_x + wall_thickness/2*1.3, 0, wall_height/2], 
             'size': [wall_thickness/2, self.library_height/2, wall_height/2]},
            # 위쪽 벽
            {'pos': [0, max_y + wall_thickness/2*1.3, wall_height/2], 
             'size': [self.library_width/2, wall_thickness/2, wall_height/2]},
        ]
        
        for wall in walls:
            wall_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=wall['size'],
                rgbaColor=wall_color
            )
            wall_collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=wall['size']
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=wall_collision,
                baseVisualShapeIndex=wall_visual,
                basePosition=wall['pos']
            )
            
        # 벽 꼭지점 채우기
        corner_size = [wall_thickness/2, wall_thickness/2, wall_height/2]
        corner_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=corner_size,
            rgbaColor=wall_color
        )
        corner_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=corner_size
        )
        
        # 네 모서리
        corners = [
            [min_x - wall_thickness/2, min_y - wall_thickness/2, wall_height/2],
            [min_x - wall_thickness/2, max_y + wall_thickness/2, wall_height/2],
            [max_x + wall_thickness/2, min_y - wall_thickness/2, wall_height/2],
            [max_x + wall_thickness/2, max_y + wall_thickness/2, wall_height/2]
        ]
        
        for corner_pos in corners:
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=corner_collision,
                baseVisualShapeIndex=corner_visual,
                basePosition=corner_pos
            )
        
        # 입구 표시
        p.addUserDebugText(
            "ENTRANCE",
            [0, min_y - 1, 2],
            textColorRGB=[1, 1, 0],
            textSize=1.0
        )
            
    def load_robot(self):
        """Bibliobot 로드"""
        print("🤖 Bibliobot 로드 중...")
        
        self.robot_start_pos = [0, -18, 0.1]
        self.robot_start_orient = p.getQuaternionFromEuler([0, 0, math.pi/2])
        
        try:
            # URDF is resolved relative to this file (repo: sim/ -> ../urdf/Bibliobot.urdf)
            urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "..", "urdf", "Bibliobot.urdf")
            self.robotId = p.loadURDF(
                urdf_path,
                self.robot_start_pos, 
                self.robot_start_orient,
                flags=p.URDF_USE_INERTIA_FROM_FILE
            )
            print(f"✅ 로봇 로드 성공. ID: {self.robotId}")
            
            # 로봇 질량 설정
            p.changeDynamics(self.robotId, -1, mass=self.robot_mass)
            
            # 로봇 색상 커스터마이징
            self.customize_robot_appearance()
            
            # 관절 정보 수집
            self.collect_joint_info()
            
            # 로봇 동역학 설정
            self.setup_robot_dynamics()
            
        except Exception as e:
            print(f"❌ 로봇 로드 실패: {e}")
            print("📝 간단한 대체 로봇 생성...")
            self.create_simple_robot()
            
    def create_simple_robot(self):
        """URDF 로드 실패 시 간단한 로봇 생성"""
        # 베이스
        base_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=self.robot_radius,
            length=0.5,
            rgbaColor=[0.1, 0.1, 0.1, 1.0]
        )
        base_collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=self.robot_radius,
            height=0.5
        )
        
        self.robotId = p.createMultiBody(
            baseMass=self.robot_mass,
            baseCollisionShapeIndex=base_collision,
            baseVisualShapeIndex=base_visual,
            basePosition=self.robot_start_pos,
            baseOrientation=self.robot_start_orient
        )
        
        # Manipulator 대체 표시
        manip_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.1, 0.4, 0.05],
            rgbaColor=[0.5, 0.1, 0.1, 1.0]
        )
        self.manipulator_id = p.createMultiBody(
            baseMass=0.1,
            baseVisualShapeIndex=manip_visual,
            basePosition=[0, 0.4, 0.3],
            parentBodyUniqueId=self.robotId,
            parentLinkIndex=-1
        )
        
    def customize_robot_appearance(self):
        """로봇 외관 커스터마이징"""
        try:
            black = [0.1, 0.1, 0.1, 1.0]
            burgundy = [0.5, 0.1, 0.1, 1.0]
            
            # 베이스 검은색
            p.changeVisualShape(self.robotId, -1, rgbaColor=black)
            
            # 모든 링크 검은색으로, manipulator만 버건디로
            for i in range(p.getNumJoints(self.robotId)):
                joint_info = p.getJointInfo(self.robotId, i)
                link_name = joint_info[12].decode('utf-8')
                
                # Manipulator 관련 부품만 버건디
                if any(keyword in link_name.lower() for keyword in ['manipulator', 'platform', 'effector']):
                    p.changeVisualShape(self.robotId, i, rgbaColor=burgundy)
                else:
                    p.changeVisualShape(self.robotId, i, rgbaColor=black)
        except:
            pass
            
    def collect_joint_info(self):
        """관절 정보 수집"""
        self.joint_indices = {}
        self.joint_names = {}
        self.manipulator_joints = []
        self.wheel_joints = []
        self.wheel_connector_joints = []
        self.platform_joint = None  # Platform joint 추가
        
        try:
            num_joints = p.getNumJoints(self.robotId)
            for i in range(num_joints):
                joint_info = p.getJointInfo(self.robotId, i)
                joint_name = joint_info[1].decode('utf-8')
                link_name = joint_info[12].decode('utf-8')
                
                self.joint_indices[joint_name] = i
                self.joint_names[i] = joint_name
                
                # Manipulator 관절 찾기
                if any(key in joint_name for key in ["Slider-12", "ManipulatorStart", "ManipulatorEnd"]):
                    self.manipulator_joints.append((i, joint_name))
                    
                # Platform joint 찾기
                if "Slider-24" in joint_name or ("Platform" in joint_name and "Slider" in joint_name):
                    self.platform_joint = i
                    print(f"✅ Platform joint 발견: {joint_name} (index: {i})")
                    
                # WheelConnector 관절 찾기
                if "Connector" in joint_name and ("Revolute" in joint_name or "Continuous" in joint_name):
                    self.wheel_connector_joints.append((i, joint_name))
                    
                # 바퀴 관절 찾기
                elif "Wheel" in joint_name:
                    self.wheel_joints.append((i, joint_name))
                    
                # End effector 링크 찾기
                if "ManipulatorEnd" in link_name or "Platform" in link_name:
                    self.manipulator_link = i
                    
        except:
            self.manipulator_link = -1
            
    def setup_robot_dynamics(self):
        """로봇 동역학 설정"""
        p.changeDynamics(
            self.robotId, -1,
            linearDamping=0.05,
            angularDamping=0.05,
            lateralFriction=2.0
        )
        
    def setup_library_shelves(self):
        """실제 스케일 도서관 책장 생성"""
        print("📚 실제 스케일 도서관 책장 생성 중...")
        
        self.shelves = {}
        self.shelf_objects = {}
        self.shelf_labels = []
        
        # 실제 스케일 책장 크기
        shelf_length = 3      # 3미터 길이
        shelf_width = 0.6     # 60cm 깊이
        aisle_width = 6.0     # 통로 폭
        
        # 중앙 통로 책장 (A, B, C)
        aisles = ['A', 'B', 'C']
        x_spacing = 8
        
        for i, aisle in enumerate(aisles):
            x_base = -8 + i * x_spacing
            
            for j in range(6):  # 6개 책장
                y_pos = -12.0 + j * shelf_length
                
                # 왼쪽 책장
                self.create_realistic_shelf(
                    f"{aisle}{j+1}L",
                    [x_base - aisle_width/2 - shelf_width/2, y_pos, 0],
                    [shelf_width, shelf_length],
                    'east'
                )
                
                # 오른쪽 책장
                self.create_realistic_shelf(
                    f"{aisle}{j+1}R",
                    [x_base + aisle_width/2 + shelf_width/2, y_pos, 0],
                    [shelf_width, shelf_length],
                    'west'
                )
        
        # 상단 책장 (T1-T14)
        row_spacing = 3.0
        
        # 첫 번째 줄 (T1-T7)
        for i in range(7):
            x_pos = -8 + i * shelf_length
            y_pos = 15
            self.create_realistic_shelf(
                f"T{i+1}",
                [x_pos, y_pos, 0],
                [shelf_length, shelf_width],
                'south'
            )
        
        # 두 번째 줄 (T8-T14)
        for i in range(7):
            x_pos = -8 + i * shelf_length
            y_pos = 10 + shelf_width
            self.create_realistic_shelf(
                f"T{i+8}",
                [x_pos, y_pos, 0],
                [shelf_length, shelf_width],
                'north'
            )
            
        print(f"✅ {len(self.shelves)}개의 실제 스케일 책장 생성 완료")
        
    def create_realistic_shelf(self, code, position, size, book_side):
        """실제 스케일 책장 생성"""
        shelf_height = 2.5  # 2.5미터 높이
        self.shelves[code] = {
            'position': position,
            'size': [size[0], size[1], shelf_height],
            'book_side': book_side,
            'access_side': self.get_opposite_side(book_side),
            'shelf_levels': {
                1: position[2] + 0.9,   # 1단 높이
                2: position[2] + 1.5,   # 2단 높이
                3: position[2] + 2.1    # 3단 높이
            }
        }
        
        wood_thickness = 0.05
        wood_color = [0.45, 0.25, 0.1, 1.0]
        
        # 책장 프레임 생성
        # 바닥판
        bottom_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[size[0]/2, size[1]/2, wood_thickness/2],
            rgbaColor=wood_color
        )
        bottom_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[size[0]/2, size[1]/2, wood_thickness/2]
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=bottom_collision,
            baseVisualShapeIndex=bottom_visual,
            basePosition=[position[0], position[1], position[2] + 0.3]
        )
        
        # 상판
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=bottom_collision,
            baseVisualShapeIndex=bottom_visual,
            basePosition=[position[0], position[1], position[2] + shelf_height]
        )
        
        # 뒷판
        if book_side in ['north', 'south']:
            back_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[size[0]/2, wood_thickness/2, shelf_height/2],
                rgbaColor=wood_color
            )
            back_collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[size[0]/2, wood_thickness/2, shelf_height/2]
            )
        else:
            back_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[wood_thickness/2, size[1]/2, shelf_height/2],
                rgbaColor=wood_color
            )
            back_collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[wood_thickness/2, size[1]/2, shelf_height/2]
            )
        
        # 뒷판 위치
        back_offset = [0, 0]
        if book_side == 'north':
            back_offset[1] = -size[1]/2
        elif book_side == 'south':
            back_offset[1] = size[1]/2
        elif book_side == 'east':
            back_offset[0] = -size[0]/2
        elif book_side == 'west':
            back_offset[0] = size[0]/2
            
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=back_collision,
            baseVisualShapeIndex=back_visual,
            basePosition=[position[0] + back_offset[0], position[1] + back_offset[1], position[2] + shelf_height/2 + 0.15]
        )
        
        # 측면판
        if book_side in ['north', 'south']:
            side_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[wood_thickness/2, size[1]/2, shelf_height/2],
                rgbaColor=wood_color
            )
            side_collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[wood_thickness/2, size[1]/2, shelf_height/2]
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=side_collision,
                baseVisualShapeIndex=side_visual,
                basePosition=[position[0] - size[0]/2, position[1], position[2] + shelf_height/2 + 0.15]
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=side_collision,
                baseVisualShapeIndex=side_visual,
                basePosition=[position[0] + size[0]/2, position[1], position[2] + shelf_height/2 + 0.15]
            )
        else:
            side_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[size[0]/2, wood_thickness/2, shelf_height/2],
                rgbaColor=wood_color
            )
            side_collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[size[0]/2, wood_thickness/2, shelf_height/2]
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=side_collision,
                baseVisualShapeIndex=side_visual,
                basePosition=[position[0], position[1] - size[1]/2, position[2] + shelf_height/2 + 0.15]
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=side_collision,
                baseVisualShapeIndex=side_visual,
                basePosition=[position[0], position[1] + size[1]/2, position[2] + shelf_height/2 + 0.15]
            )
        
        # 중간 선반들
        shelf_heights = [0.9, 1.5, 2.1]
        for shelf_y in shelf_heights:
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=bottom_collision,
                baseVisualShapeIndex=bottom_visual,
                basePosition=[position[0], position[1], position[2] + shelf_y]
            )
            
        # 책이 있는 면 표시
        indicator_color = [0.9, 0.1, 0.1, 1.0]
        
        if book_side in ['east', 'west']:
            indicator_size = [0.02, size[1]/2 - 0.1, shelf_height/2 - 0.2]
        else:
            indicator_size = [size[0]/2 - 0.1, 0.02, shelf_height/2 - 0.2]
            
        indicator_offset = [0, 0]
        if book_side == 'east':
            indicator_offset[0] = size[0]/2
        elif book_side == 'west':
            indicator_offset[0] = -size[0]/2
        elif book_side == 'north':
            indicator_offset[1] = size[1]/2
        elif book_side == 'south':
            indicator_offset[1] = -size[1]/2
            
        indicator_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=indicator_size,
            rgbaColor=indicator_color
        )
        
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=indicator_visual,
            basePosition=[
                position[0] + indicator_offset[0],
                position[1] + indicator_offset[1],
                position[2] + shelf_height/2 + 0.15
            ]
        )
        
        # 책장 라벨
        label_id = p.addUserDebugText(
            code,
            [position[0], position[1], position[2] + shelf_height + 0.3],
            textColorRGB=[1, 1, 1],
            textSize=1.0
        )
        self.shelf_labels.append(label_id)
        
    def get_opposite_side(self, side):
        """반대 방향 반환"""
        opposites = {
            'north': 'south',
            'south': 'north',
            'east': 'west',
            'west': 'east'
        }
        return opposites.get(side, side)
        
    def setup_robot_vision(self):
        """개선된 컴퓨터 비전 설정"""
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fov = 90
        self.camera_near = 0.1
        self.camera_far = 10.0
        
        # 비전 윈도우
        cv2.namedWindow('Bibliobot Vision - Front View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Bibliobot Vision - Front View', 640, 480)
        
    def update_robot_vision(self):
        """로봇 시점 카메라 업데이트"""
        try:
            # 로봇 위치 및 방향
            robot_pos, robot_orient = p.getBasePositionAndOrientation(self.robotId)
            robot_matrix = p.getMatrixFromQuaternion(robot_orient)
            rotation_matrix = np.array(robot_matrix).reshape(3, 3)
            
            # 카메라 위치: 로봇 정면 약간 위
            camera_offset = rotation_matrix.dot([0.3, 0, 3])
            camera_pos = np.array(robot_pos) + camera_offset
            
            # 타겟: 로봇 전방 1.5m 지점의 중간 높이
            target_offset = rotation_matrix.dot([1.5, 0, 1.2])
            target_pos = np.array(robot_pos) + target_offset
            
            # View/Projection 행렬
            view_matrix = p.computeViewMatrix(
                cameraEyePosition=camera_pos,
                cameraTargetPosition=target_pos,
                cameraUpVector=[0, 0, 1]
            )
            
            projection_matrix = p.computeProjectionMatrixFOV(
                fov=self.camera_fov,
                aspect=self.camera_width/self.camera_height,
                nearVal=self.camera_near,
                farVal=self.camera_far
            )
            
            # 이미지 렌더링
            _, _, rgb_img, depth_img, seg_img = p.getCameraImage(
                width=self.camera_width,
                height=self.camera_height,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix,
                renderer=p.ER_BULLET_HARDWARE_OPENGL
            )
            
            # OpenCV 형식 변환
            rgb_array = np.array(rgb_img).reshape(self.camera_height, self.camera_width, 4)[:, :, :3]
            rgb_array = np.ascontiguousarray(rgb_array, dtype=np.uint8)
            
            # 정보 오버레이
            cv2.putText(rgb_array, "BIBLIOBOT VISION", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 현재 타겟 책장
            if hasattr(self, 'current_target'):
                cv2.putText(rgb_array, f"Target: {self.current_target}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                           
            # 현재 선반 레벨
            cv2.putText(rgb_array, f"Level: {self.current_shelf_level}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Manipulator 높이
            if hasattr(self, 'manipulator_joints') and self.manipulator_joints:
                for idx, (joint_idx, joint_name) in enumerate(self.manipulator_joints[:1]):
                    pos = p.getJointState(self.robotId, joint_idx)[0]
                    height_m = abs(pos)
                    # 높이에 따른 레벨 계산
                    if height_m < 0.5:
                        level = 1
                    elif height_m < 1.0:
                        level = 2
                    else:
                        level = 3
                    cv2.putText(rgb_array, f"Arm: {height_m:.2f}m (L{level})", (10, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 1)
            
            # 중앙 십자선
            center_x = self.camera_width // 2
            center_y = self.camera_height // 2
            cv2.line(rgb_array, (center_x - 20, center_y), (center_x + 20, center_y), (255, 0, 0), 2)
            cv2.line(rgb_array, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 2)
            
            # 선반 높이 가이드라인
            if hasattr(self, 'at_shelf') and self.at_shelf:
                level_lines = {
                    3: int(self.camera_height * 0.2),
                    2: int(self.camera_height * 0.4),
                    1: int(self.camera_height * 0.6)
                }
                for level, y_pos in level_lines.items():
                    color = (0, 255, 0) if level == self.current_shelf_level else (100, 100, 100)
                    cv2.line(rgb_array, (50, y_pos), (100, y_pos), color, 2)
                    cv2.putText(rgb_array, f"L{level}", (20, y_pos + 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            cv2.imshow('Bibliobot Vision - Front View', rgb_array)
            cv2.waitKey(1)
            
            # 녹화 중이면 프레임 저장
            if self.recording:
                self.vision_frames.append(rgb_array.copy())
            
        except Exception as e:
            pass
            
    def setup_visualizer(self):
        """카메라 뷰 설정"""
        p.resetDebugVisualizerCamera(
            cameraDistance=25.0,
            cameraYaw=45,
            cameraPitch=-45,
            cameraTargetPosition=[0, 0, 0]
        )
        
    def start_recording(self):
        """화면 녹화 시작"""
        self.recording = True
        self.recording_frames = []
        self.vision_frames = []
        self.recording_start_time = datetime.datetime.now()
        print("🎥 녹화 시작...")
        
    def stop_recording(self):
        """화면 녹화 중지 및 저장"""
        if not self.recording:
            return
            
        self.recording = False
        timestamp = self.recording_start_time.strftime("%Y%m%d_%H%M%S")
        
        # GUI 화면 녹화 저장
        if self.recording_frames:
            gui_filename = f"bibliobot_gui_{timestamp}.mp4"
            self.save_video(self.recording_frames, gui_filename, fps=30)
            print(f"📹 GUI 녹화 저장: {gui_filename}")
            
        # 비전 화면 녹화 저장
        if self.vision_frames:
            vision_filename = f"bibliobot_vision_{timestamp}.mp4"
            self.save_video(self.vision_frames, vision_filename, fps=30)
            print(f"📹 비전 녹화 저장: {vision_filename}")
            
    def save_video(self, frames, filename, fps=30):
        """프레임을 비디오로 저장"""
        if not frames:
            return
            
        height, width = frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(filename, fourcc, fps, (width, height))
        
        for frame in frames:
            out.write(frame)
            
        out.release()
        
    def capture_gui_frame(self):
        """GUI 화면 캡처"""
        if self.recording:
            # PyBullet GUI 화면 캡처
            width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                width=1920,
                height=1080,
                renderer=p.ER_BULLET_HARDWARE_OPENGL
            )
            
            rgb_array = np.array(rgbImg).reshape(height, width, 4)[:, :, :3]
            rgb_array = np.ascontiguousarray(rgb_array, dtype=np.uint8)
            self.recording_frames.append(rgb_array)
            
    def predict_joint_kinematics_multi(self, shelf_list):
        """다중 책장 경로에 대한 전체 운동학 예측 - 연속적인 Manipulator/Platform 움직임"""
        print("\n📊 다중 경로 예상 관절 운동학 분석...")
        
        # 현재 위치
        robot_pos, robot_orient = p.getBasePositionAndOrientation(self.robotId)
        current_pos = [robot_pos[0], robot_pos[1]]
        current_yaw = p.getEulerFromQuaternion(robot_orient)[2]
        
        # 최적 순서 계산
        optimized_sequence = self.optimize_shelf_sequence(shelf_list)
        if not optimized_sequence:
            return
            
        # 전체 경로 계산
        total_distance = 0
        total_rotation = 0
        segments = []
        
        prev_pos = current_pos
        prev_yaw = current_yaw
        
        for target in optimized_sequence:
            if isinstance(target, tuple):
                shelf_code, shelf_level = target
            else:
                shelf_code, shelf_level = target, 2
                
            approach_pos, approach_yaw = self.get_approach_position(shelf_code)
            if approach_pos is None:
                continue
                
            # 세그먼트 정보
            segment_distance = math.sqrt((approach_pos[0] - prev_pos[0])**2 + 
                                       (approach_pos[1] - prev_pos[1])**2)
            segment_rotation = abs(approach_yaw - prev_yaw)
            if segment_rotation > math.pi:
                segment_rotation = 2 * math.pi - segment_rotation
                
            segments.append({
                'shelf': shelf_code,
                'level': shelf_level,
                'distance': segment_distance,
                'rotation': segment_rotation,
                'target_pos': approach_pos,
                'target_yaw': approach_yaw
            })
            
            total_distance += segment_distance
            total_rotation += segment_rotation
            
            prev_pos = approach_pos
            prev_yaw = approach_yaw
            
        # 예상 시간 계산
        rotation_time = 1.5
        platform_time = 1.0
        work_time = 2.0
        retract_time = 1.0  # Platform 축소 시간
        
        total_time = 0
        for seg in segments:
            seg_rotation_time = seg['rotation'] / self.max_angular_speed + rotation_time
            seg_linear_time = seg['distance'] / self.max_linear_speed + 2.0
            total_time += seg_rotation_time + seg_linear_time + platform_time + work_time + retract_time
            
        # 시간 벡터
        t = np.linspace(0, total_time, 500)
        dt = t[1] - t[0]
        
        # 예상 관절 궤적
        wheel_positions = np.zeros((len(t), 2))
        wheel_velocities = np.zeros((len(t), 2))
        wheel_torques = np.zeros((len(t), 2))
        
        manipulator_position = np.zeros(len(t))
        manipulator_velocity = np.zeros(len(t))
        manipulator_force = np.zeros(len(t))
        
        platform_extension = np.zeros(len(t))
        platform_velocity = np.zeros(len(t))
        platform_force = np.zeros(len(t))
        
        # 세그먼트별 색상
        segment_colors = plt.cm.tab10(np.linspace(0, 1, len(segments)))
        
        # 운동학 계산
        wheel_radius = 0.1
        wheel_base = 0.4
        current_time = 0
        
        # 현재 Manipulator 높이 (초기값)
        current_manip_height = 0  # 기본 1단 높이
        
        for seg_idx, seg in enumerate(segments):
            # 목표 Manipulator 높이
            level_heights = {1: 0.3, 2: 0.7, 3: 1.2}
            target_height = level_heights.get(seg['level'], 0.7)
            
            # 1. 회전 단계 (제자리에서)
            rotation_duration = seg['rotation'] / self.max_angular_speed + rotation_time
            
            for i, time_point in enumerate(t):
                if current_time <= time_point <= current_time + rotation_duration:
                    local_time = time_point - current_time
                    progress = local_time / rotation_duration
                    
                    # 5차 다항식 속도 프로파일
                    s = 10*progress**3 - 15*progress**4 + 6*progress**5
                    s_dot = 30*progress**2 - 60*progress**3 + 30*progress**4
                    s_ddot = 60*progress - 180*progress**2 + 120*progress**3
                    
                    # 각속도
                    angular_vel = s_dot * seg['rotation'] / rotation_duration
                    angular_acc = s_ddot * seg['rotation'] / (rotation_duration**2)
                    
                    # 바퀴 속도 (차동 구동)
                    wheel_velocities[i, 0] = angular_vel * wheel_base / 2
                    wheel_velocities[i, 1] = -angular_vel * wheel_base / 2
                    
                    # Manipulator 움직임 시작 (목표 높이로 연속적으로 이동)
                    if progress > 0.2:  # 회전 20% 이후부터
                        manip_progress = (progress - 0.2) / 0.8
                        s_manip = 10*manip_progress**3 - 15*manip_progress**4 + 6*manip_progress**5
                        manipulator_position[i] = current_manip_height + (target_height - current_manip_height) * s_manip
                        
            current_time += rotation_duration
            
            # 2. 직진 단계
            linear_duration = seg['distance'] / self.max_linear_speed + 2.0
            
            for i, time_point in enumerate(t):
                if current_time <= time_point <= current_time + linear_duration:
                    local_time = time_point - current_time
                    progress = local_time / linear_duration
                    
                    # 5차 다항식 속도 프로파일
                    s = 10*progress**3 - 15*progress**4 + 6*progress**5
                    s_dot = 30*progress**2 - 60*progress**3 + 30*progress**4
                    
                    velocity = s_dot * seg['distance'] / linear_duration
                    
                    # 바퀴 속도 (동일)
                    wheel_velocities[i, 0] = velocity / wheel_radius
                    wheel_velocities[i, 1] = velocity / wheel_radius
                    
                    # Manipulator 계속 이동 (필요시)
                    current_height = manipulator_position[i-1] if i > 0 else current_manip_height
                    if abs(current_height - target_height) > 0.01:
                        height_progress = min(1.0, progress * 2)
                        s_height = 10*height_progress**3 - 15*height_progress**4 + 6*height_progress**5
                        manipulator_position[i] = current_height + (target_height - current_height) * s_height
                    else:
                        manipulator_position[i] = target_height
                        
            current_time += linear_duration
            
            # 3. Platform Extension (도착 후에만, 5차 다항식)
            for i, time_point in enumerate(t):
                if current_time <= time_point <= current_time + platform_time:
                    local_time = time_point - current_time
                    progress = local_time / platform_time
                    # 5차 다항식 프로파일
                    s_platform = 10*progress**3 - 15*progress**4 + 6*progress**5
                    s_platform_dot = (30*progress**2 - 60*progress**3 + 30*progress**4) / platform_time
                    s_platform_ddot = (60*progress - 180*progress**2 + 120*progress**3) / (platform_time**2)
                    
                    platform_extension[i] = 0.115 * s_platform
                    platform_velocity[i] = 0.115 * s_platform_dot
                    platform_acceleration = 0.115 * s_platform_ddot
                    platform_force[i] = 2.0 * 9.81 + 2.0 * platform_acceleration  # 2kg 부하
                    
                    # Manipulator는 목표 높이 유지
                    manipulator_position[i] = target_height
                    
            current_time += platform_time
            
            # 4. 작업 시간
            for i, time_point in enumerate(t):
                if current_time <= time_point <= current_time + work_time:
                    # Platform 유지
                    platform_extension[i] = 0.115
                    platform_velocity[i] = 0
                    platform_force[i] = 2.0 * 9.81  # 정적 힘만
                    # Manipulator 유지
                    manipulator_position[i] = target_height
                    
            current_time += work_time
            
            # Platform 축소 (5차 다항식)
            retract_time = 1.0  # 축소 시간 증가
            for i, time_point in enumerate(t):
                if current_time <= time_point <= current_time + retract_time:
                    local_time = time_point - current_time
                    progress = local_time / retract_time
                    # 5차 다항식 프로파일
                    s_retract = 10*progress**3 - 15*progress**4 + 6*progress**5
                    s_retract_dot = (30*progress**2 - 60*progress**3 + 30*progress**4) / retract_time
                    s_retract_ddot = (60*progress - 180*progress**2 + 120*progress**3) / (retract_time**2)
                    
                    platform_extension[i] = 0.115 * (1 - s_retract)
                    platform_velocity[i] = -0.115 * s_retract_dot
                    platform_acceleration = -0.115 * s_retract_ddot
                    platform_force[i] = 2.0 * 9.81 + 2.0 * platform_acceleration  # 2kg 부하
                    
                    # Manipulator는 계속 유지
                    manipulator_position[i] = target_height
            
            # 현재 Manipulator 높이 업데이트
            current_manip_height = target_height
            
        for i in range(len(manipulator_position)):
            if manipulator_position[i] ==0 :
                manipulator_position[i] = manipulator_position[i-1]
        
        
        # 위치 적분
        for i in range(1, len(t)):
            wheel_positions[i, 0] = wheel_positions[i-1, 0] + wheel_velocities[i, 0] * dt
            wheel_positions[i, 1] = wheel_positions[i-1, 1] + wheel_velocities[i, 1] * dt
            
        # Manipulator 속도/가속도/힘 계산 (5차 다항식 기반)
        manipulator_velocity = np.gradient(manipulator_position, dt)
        manipulator_acceleration = np.gradient(manipulator_velocity, dt)
        manipulator_force = 5.0 * 9.81 + 5.0 * manipulator_acceleration  # 5kg 부하
        
        # Platform 속도/힘은 위에서 이미 계산됨 (5차 다항식 기반)
        
        # 토크 계산 (부드러운 프로파일)
        wheel_inertia = 0.5
        wheel_mass = self.robot_mass / 3
        
        for i in range(len(t)):
            for j in range(2):
                if i > 0:
                    accel = (wheel_velocities[i, j] - wheel_velocities[i-1, j]) / dt
                else:
                    accel = 0
                wheel_torques[i, j] = wheel_inertia * accel + 0.02 * wheel_mass * 9.81 * wheel_radius * np.sign(wheel_velocities[i, j])
                
        # 그래프 생성
        fig, axes = plt.subplots(4, 3, figsize=(18, 16))
        fig.suptitle(f'Multi-Path Kinematics Prediction - {len(segments)} Shelves, Total Time: {total_time:.1f}s', 
                    fontsize=16, fontweight='bold')
        
        # 바퀴 위치
        axes[0, 0].plot(t, wheel_positions[:, 0], 'b-', linewidth=2, label='Left Wheel')
        axes[0, 0].plot(t, wheel_positions[:, 1], 'r-', linewidth=2, label='Right Wheel')
        axes[0, 0].set_title('Wheel Positions', fontsize=12, fontweight='bold')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position (rad)')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        
        # 바퀴 속도
        axes[0, 1].plot(t, wheel_velocities[:, 0], 'b-', linewidth=2, label='Left Wheel')
        axes[0, 1].plot(t, wheel_velocities[:, 1], 'r-', linewidth=2, label='Right Wheel')
        axes[0, 1].set_title('Wheel Velocities', fontsize=12, fontweight='bold')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Velocity (rad/s)')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()
        
        # 바퀴 토크
        axes[0, 2].plot(t, wheel_torques[:, 0], 'b-', linewidth=2, label='Left Wheel')
        axes[0, 2].plot(t, wheel_torques[:, 1], 'r-', linewidth=2, label='Right Wheel')
        axes[0, 2].set_title('Wheel Torques (Smooth Profile)', fontsize=12, fontweight='bold')
        axes[0, 2].set_xlabel('Time (s)')
        axes[0, 2].set_ylabel('Torque (Nm)')
        axes[0, 2].grid(True, alpha=0.3)
        axes[0, 2].legend()
        
        # Manipulator 높이 (연속적인 움직임 강조)
        axes[1, 0].plot(t, manipulator_position, 'g-', linewidth=3)
        axes[1, 0].set_title('Manipulator Height (Continuous Motion)', fontsize=12, fontweight='bold')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Height (m)')
        axes[1, 0].grid(True, alpha=0.3)
        
        # 레벨 표시
        for level, height in level_heights.items():
            axes[1, 0].axhline(y=height, color='gray', linestyle='--', alpha=0.5, label=f'L{level}')
        axes[1, 0].legend()
        
        # Manipulator 속도/힘
        axes[1, 1].plot(t, manipulator_velocity, 'g-', linewidth=2, label='Velocity')
        ax2 = axes[1, 1].twinx()
        ax2.plot(t, manipulator_force, 'orange', linewidth=2, label='Force')
        axes[1, 1].set_title('Manipulator Velocity & Force (5th-order)', fontsize=12, fontweight='bold')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Velocity (m/s)', color='g')
        ax2.set_ylabel('Force (N)', color='orange')
        axes[1, 1].grid(True, alpha=0.3)
        
        # Platform Extension
        axes[1, 2].plot(t, platform_extension * 100, 'm-', linewidth=3)
        axes[1, 2].set_title('Platform Extension', fontsize=12, fontweight='bold')
        axes[1, 2].set_xlabel('Time (s)')
        axes[1, 2].set_ylabel('Extension (cm)')
        axes[1, 2].grid(True, alpha=0.3)
        axes[1, 2].axhline(y=11.5, color='r', linestyle='--', label='Max Extension')
        axes[1, 2].legend()
        
        # Platform 힘
        axes[2, 0].plot(t, platform_force, 'm-', linewidth=2)
        axes[2, 0].set_title('Platform Extension Force (5th-order)', fontsize=12, fontweight='bold')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_ylabel('Force (N)')
        axes[2, 0].grid(True, alpha=0.3)
        
        # 경로 시각화
        axes[2, 1].axis('equal')
        axes[2, 1].set_title('Planned Path', fontsize=12, fontweight='bold')
        axes[2, 1].set_xlabel('X (m)')
        axes[2, 1].set_ylabel('Y (m)')
        axes[2, 1].grid(True, alpha=0.3)
        
        # 경로 그리기
        path_x = [current_pos[0]]
        path_y = [current_pos[1]]
        for seg in segments:
            path_x.append(seg['target_pos'][0])
            path_y.append(seg['target_pos'][1])
            
        axes[2, 1].plot(path_x, path_y, 'b-', linewidth=2, marker='o', markersize=8)
        
        # 책장 위치 표시
        for seg_idx, seg in enumerate(segments):
            axes[2, 1].text(seg['target_pos'][0], seg['target_pos'][1], 
                          f"{seg['shelf']}\nL{seg['level']}", 
                          ha='center', va='bottom', fontsize=8,
                          bbox=dict(boxstyle="round,pad=0.3", 
                                  facecolor=segment_colors[seg_idx], alpha=0.5))
        
        # 작업 순서 타임라인
        axes[2, 2].set_title('Work Timeline', fontsize=12, fontweight='bold')
        axes[2, 2].set_xlabel('Time (s)')
        axes[2, 2].set_ylabel('Shelf')
        axes[2, 2].grid(True, alpha=0.3)
        
        current_time = 0
        for seg_idx, seg in enumerate(segments):
            seg_rotation_time = seg['rotation'] / self.max_angular_speed + rotation_time
            seg_linear_time = seg['distance'] / self.max_linear_speed + 2.0
            seg_total_time = seg_rotation_time + seg_linear_time + platform_time + work_time + retract_time
            
            axes[2, 2].barh(seg_idx, seg_total_time, left=current_time, height=0.8,
                          color=segment_colors[seg_idx], alpha=0.7,
                          label=f"{seg['shelf']} (L{seg['level']})")
            axes[2, 2].text(current_time + seg_total_time/2, seg_idx, 
                          f"{seg['shelf']}", ha='center', va='center', fontsize=9)
            current_time += seg_total_time
            
        axes[2, 2].set_yticks(range(len(segments)))
        axes[2, 2].set_yticklabels([seg['shelf'] for seg in segments])
        
        # 전체 통계 (아래쪽 그래프 공간 활용)
        axes[3, 0].axis('off')
        axes[3, 1].axis('off')
        axes[3, 2].axis('off')
        
        # 콘솔에 상세 정보 출력
        print("\n" + "="*60)
        print("📊 경로 요약:")
        print(f"  총 책장 수: {len(segments)}")
        print(f"  총 이동 거리: {total_distance:.2f}m")
        print(f"  총 회전량: {total_rotation*180/np.pi:.1f}°")
        print(f"  총 소요 시간: {total_time:.1f}s")
        print(f"  평균 속도: {total_distance/total_time:.2f}m/s")
        print("\n  방문 순서:")
        for idx, seg in enumerate(segments):
            print(f"    {idx+1}. {seg['shelf']} (Level {seg['level']})")
            
        print("\n💪 최대값:")
        print(f"  바퀴 토크: {np.max(np.abs(wheel_torques)):.2f}Nm")
        print(f"  Manipulator 힘: {np.max(np.abs(manipulator_force)):.2f}N")
        print(f"  Platform 힘: {np.max(np.abs(platform_force)):.2f}N")
        
        print("\n🎛️ PID 제어 권장값:")
        print("  바퀴 모터: Kp=15, Ki=1, Kd=3")
        print("  Manipulator: Kp=50, Ki=1, Kd=5")
        print("  Platform: Kp=30, Ki=0.5, Kd=2")
        
        # 에너지 소비 예측 (휴머노이드 로봇 배터리 기준)
        wheel_energy = np.trapz(np.abs(wheel_torques[:, 0] * wheel_velocities[:, 0]) + 
                              np.abs(wheel_torques[:, 1] * wheel_velocities[:, 1]), t)
        manip_energy = np.trapz(np.abs(manipulator_force * manipulator_velocity), t)
        platform_energy = np.trapz(np.abs(platform_force * platform_velocity), t)
        
        total_energy = wheel_energy + manip_energy + platform_energy
        
        print("\n🔋 에너지 소비 예측:")
        print(f"  바퀴 모터: {wheel_energy:.1f}J")
        print(f"  Manipulator: {manip_energy:.1f}J")
        print(f"  Platform: {platform_energy:.1f}J")
        print(f"  총 에너지: {total_energy:.1f}J ({total_energy/3600:.2f}Wh)")
        
        # 일반적인 휴머노이드 로봇 배터리 (24V 20Ah = 480Wh)
        typical_battery = 480  # Wh
        consumption_percent = (total_energy/3600/typical_battery)*100
        print(f"\n🔋 배터리 소비 (휴머노이드 로봇 기준):")
        print(f"  일반적인 배터리: 24V 20Ah = 480Wh")
        print(f"  예상 소비량: {consumption_percent:.2f}%")
        print(f"  남은 용량: {100-consumption_percent:.2f}%")
        print("="*60)
        
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(5)
        plt.close()
        
        print("✅ 다중 경로 운동학 분석 완료. 이동을 시작합니다.")
        
    def control_manipulator_height(self, target_level):
        """Manipulator 높이 제어 + Platform Extension (5차 다항식 적용)"""
        if not hasattr(self, 'manipulator_joints') or not self.manipulator_joints:
            return
            
        # 레벨별 높이 설정 (실제 스케일)
        level_heights = {
            1: 0.3,   # 1단 (낮음)
            2: 0.7,   # 2단 (중간)
            3: 1.2    # 3단 (높음)
        }
        
        target_height = level_heights.get(target_level, 0.7)
        
        # 책장에 도착한 경우에만 Platform Extension
        if hasattr(self, 'at_shelf') and self.at_shelf and not hasattr(self, 'platform_extended'):
            print(f"📤 Platform Extension 시작 (Level {target_level})...")
            
            # Platform joint 찾기
            if self.platform_joint is not None:
                # Platform 전진 (5차 다항식 프로파일)
                extend_steps = 30
                max_extension = -0.195  # 최대 확장 위치
                base_position = -0.08   # 기본 위치
                
                for step in range(extend_steps):
                    t = step / float(extend_steps - 1)
                    # 5차 다항식
                    s = 10*t**3 - 15*t**4 + 6*t**5
                    extension = base_position + (max_extension - base_position) * s
                    
                    p.setJointMotorControl2(
                        self.robotId,
                        self.platform_joint,
                        p.POSITION_CONTROL,
                        targetPosition=extension,
                        force=1000,
                        maxVelocity=0.2
                    )
                    
                    for _ in range(3):
                        p.stepSimulation()
                    
                    if step % 5 == 0:
                        self.update_robot_vision()
                        self.capture_gui_frame()
                    
                print("✅ Platform Extension 완료")
                self.platform_extended = True
                self.platform_extended_value = max_extension  # 현재 확장 값 저장
            
            # 작업 시뮬레이션
            for _ in range(10):
                p.stepSimulation()
                self.update_robot_vision()
                self.capture_gui_frame()
                time.sleep(0.02)
                
    def get_approach_position(self, shelf_code):
        """책장 접근 위치 계산 - 정확한 방향 계산"""
        if shelf_code not in self.shelves:
            return None, None
            
        shelf = self.shelves[shelf_code]
        pos = shelf['position']
        size = shelf['size']
        book_side = shelf['book_side']
        
        # 책이 있는 면을 향하도록 접근 (올바른 방향 설정)
        if book_side == 'east':
            approach_pos = [pos[0] + size[0]/2 + self.approach_distance, pos[1]]
            approach_yaw = math.pi  # 180도 (서쪽을 향함 - 책을 보기 위해)
        elif book_side == 'west':
            approach_pos = [pos[0] - size[0]/2 - self.approach_distance, pos[1]]
            approach_yaw = 0  # 0도 (동쪽을 향함 - 책을 보기 위해)
        elif book_side == 'north':
            approach_pos = [pos[0], pos[1] + size[1]/2 + self.approach_distance]
            approach_yaw = -math.pi/2  # -90도 (남쪽을 향함 - 책을 보기 위해)
        elif book_side == 'south':
            approach_pos = [pos[0], pos[1] - size[1]/2 - self.approach_distance]
            approach_yaw = math.pi/2  # 90도 (북쪽을 향함 - 책을 보기 위해)
            
        print(f"   책장 {shelf_code}: 책이 {book_side}쪽에 있음")
        print(f"   접근 위치: ({approach_pos[0]:.2f}, {approach_pos[1]:.2f})")
        print(f"   목표 방향: {approach_yaw*180/math.pi:.0f}°")
            
        return approach_pos, approach_yaw
        
    def plan_manhattan_path(self, start_pos, target_pos):
        """Manhattan 스타일 경로 계획 - 더 세밀한 안전거리"""
        print(f"\n🗺️ Manhattan 경로 계획")
        print(f"시작: ({start_pos[0]:.2f}, {start_pos[1]:.2f})")
        print(f"목표: ({target_pos[0]:.2f}, {target_pos[1]:.2f})")
        
        # 그리드 설정
        grid_resolution = 0.05  # 더 세밀한 그리드
        min_x, max_x = -self.library_width/2 - 0.5, self.library_width/2 + 0.5
        min_y, max_y = -self.library_height/2 - 0.5, self.library_height/2 + 0.5
        
        # 그리드 좌표 변환
        def to_grid(pos):
            return (
                int((pos[0] - min_x) / grid_resolution),
                int((pos[1] - min_y) / grid_resolution)
            )
            
        def to_world(grid_pos):
            return (
                min_x + grid_pos[0] * grid_resolution,
                min_y + grid_pos[1] * grid_resolution
            )
            
        start_grid = to_grid(start_pos)
        goal_grid = to_grid(target_pos)
        
        # A* 알고리즘
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
            
        def is_valid(grid_pos):
            x, y = to_world(grid_pos)
            # 경계 확인
            if x < -self.library_width/2 or x > self.library_width/2 or y < -self.library_height/2 or y > self.library_height/2:
                return False
            # 책장과의 충돌 확인
            return self.is_position_safe([x, y])
            
        # A* 탐색
        open_set = []
        heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: heuristic(start_grid, goal_grid)}
        
        # Manhattan 이동 (4방향)
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        while open_set:
            current = heappop(open_set)[1]
            
            if current == goal_grid:
                # 경로 재구성
                path = []
                while current in came_from:
                    path.append(to_world(current))
                    current = came_from[current]
                path.append(start_pos)
                path.reverse()
                
                # 경로 단순화
                simplified = self.simplify_path(path)
                print(f"✅ 경로 발견: {len(simplified)}개 웨이포인트")
                return simplified
                
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not is_valid(neighbor):
                    continue
                    
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal_grid)
                    heappush(open_set, (f_score[neighbor], neighbor))
                    
        print("❌ 경로를 찾을 수 없습니다")
        return None
        
    def simplify_path(self, path):
        """경로 단순화"""
        if len(path) <= 2:
            return path
            
        simplified = [path[0]]
        
        for i in range(1, len(path)-1):
            prev_dx = path[i][0] - path[i-1][0]
            prev_dy = path[i][1] - path[i-1][1]
            next_dx = path[i+1][0] - path[i][0]
            next_dy = path[i+1][1] - path[i][1]
            
            # 방향이 바뀌는 코너 포인트만 추가
            if (abs(prev_dx) > 0 and abs(next_dy) > 0) or (abs(prev_dy) > 0 and abs(next_dx) > 0):
                simplified.append(path[i])
                
        simplified.append(path[-1])
        return simplified
        
    def is_position_safe(self, pos):
        """위치가 안전한지 확인 - 증가된 안전거리"""
        for shelf_code, shelf in self.shelves.items():
            shelf_pos = shelf['position']
            shelf_size = shelf['size']
            
            # 증가된 안전거리
            safety_x = self.safety_distance
            safety_y = self.safety_distance
            
            # 통로에서는 안전거리를 약간 줄임
            if shelf_code.endswith('L') or shelf_code.endswith('R'):
                safety_x = self.safety_distance * 0.9
                
            min_x = shelf_pos[0] - shelf_size[0]/2 - safety_x
            max_x = shelf_pos[0] + shelf_size[0]/2 + safety_x
            min_y = shelf_pos[1] - shelf_size[1]/2 - safety_y
            max_y = shelf_pos[1] + shelf_size[1]/2 + safety_y
            
            if min_x <= pos[0] <= max_x and min_y <= pos[1] <= max_y:
                return False
                
        # 벽과의 충돌
        wall_margin = 0.3
        if pos[0] < -self.library_width/2 + wall_margin or pos[0] > self.library_width/2 - wall_margin:
            return False
        if pos[1] < -self.library_height/2 + wall_margin or pos[1] > self.library_height/2 - wall_margin:
            return False
            
        return True
        
    def retract_platform(self):
        """Platform을 천천히 원위치로 복귀 (5차 다항식)"""
        if hasattr(self, 'platform_extended_value') and self.platform_joint is not None:
            print("📥 Platform Retraction 시작...")
            
            retract_steps = 30
            current_extension = self.platform_extended_value
            base_position = -0.08
            
            for step in range(retract_steps):
                t = step / float(retract_steps - 1)
                # 5차 다항식
                s = 10*t**3 - 15*t**4 + 6*t**5
                extension = current_extension + (base_position - current_extension) * s
                
                p.setJointMotorControl2(
                    self.robotId,
                    self.platform_joint,
                    p.POSITION_CONTROL,
                    targetPosition=extension,
                    force=1000,
                    maxVelocity=0.2
                )
                
                for _ in range(3):
                    p.stepSimulation()
                
                if step % 5 == 0:
                    self.update_robot_vision()
                    self.capture_gui_frame()
                    
            print("✅ Platform Retraction 완료")
            self.platform_extended_value = base_position
            
    def navigate_to_shelf(self, target_shelf_code, shelf_level=None):
        """책장으로 네비게이션 - 개선된 버전"""
        print(f"\n{'='*60}")
        print(f"🎯 목표 책장: {target_shelf_code}")
        if shelf_level:
            print(f"📚 선반 레벨: {shelf_level}단")
            self.target_shelf_level = shelf_level
        print(f"{'='*60}")
        
        # 이전 작업의 Platform이 확장되어 있다면 먼저 축소
        if hasattr(self, 'platform_extended') and self.platform_extended:
            self.retract_platform()
        
        # 상태 초기화
        self.current_target = target_shelf_code
        self.at_shelf = False
        self.platform_extended = False  # Platform Extension 플래그 리셋
        
        # 접근 위치 계산
        approach_pos, approach_yaw = self.get_approach_position(target_shelf_code)
        if approach_pos is None:
            print(f"❌ 책장 '{target_shelf_code}'를 찾을 수 없습니다")
            return False
            
        # 현재 위치
        robot_pos, robot_orient = p.getBasePositionAndOrientation(self.robotId)
        current_pos = [robot_pos[0], robot_pos[1]]
        current_yaw = p.getEulerFromQuaternion(robot_orient)[2]
        
        # 경로 계획
        path = self.plan_manhattan_path(current_pos, approach_pos)
        if path is None:
            return False
            
        # 경로 시각화
        self.visualize_path(path)
        
        # 경로 따라 이동 (omnidrive)
        success = self.follow_path_omnidrive(path, approach_yaw, shelf_level)
        
        if success:
            print(f"✅ {target_shelf_code} 도착 완료!")
            self.at_shelf = True
            if shelf_level:
                self.current_shelf_level = shelf_level
                # 현재 Manipulator 높이 업데이트
                level_heights = {1: 0.3, 2: 0.7, 3: 1.2}
                self.current_manipulator_height = level_heights.get(shelf_level, 0.7)
                
            self.path_history.append({
                'shelf': target_shelf_code,
                'level': self.current_shelf_level,
                'path': path,
                'timestamp': time.time()
            })
            
            # 도착 후 한 번만 Platform Extension 실행
            self.control_manipulator_height(self.current_shelf_level)
            
        return success
        
    def visualize_path(self, path):
        """경로 시각화"""
        # 이전 경로 삭제
        if hasattr(self, 'path_lines'):
            for line_id in self.path_lines:
                p.removeUserDebugItem(line_id)
                
        self.path_lines = []
        
        # 새 경로 그리기 (빨간색)
        for i in range(len(path)-1):
            line_id = p.addUserDebugLine(
                [path[i][0], path[i][1], 0.05],
                [path[i+1][0], path[i+1][1], 0.05],
                lineColorRGB=[1, 0, 0],
                lineWidth=4
            )
            self.path_lines.append(line_id)
            
        # 웨이포인트 표시
        for i, waypoint in enumerate(path):
            p.addUserDebugText(
                f"W{i}",
                [waypoint[0], waypoint[1], 0.3],
                textColorRGB=[1, 1, 0],
                textSize=1.5,
                lifeTime=10.0
            )
            
    def follow_path_omnidrive(self, path, final_yaw, shelf_level=None):
        """Omnidrive 경로 추종 - 최종 방향으로 먼저 회전 후 직선 이동"""
        print("\n🚗 Omnidrive 경로 추종 시작")
        
        # 목표 레벨 설정
        if shelf_level:
            self.target_shelf_level = shelf_level
            print(f"📐 Manipulator 높이 조절 목표: {self.target_shelf_level}단")
            
        # Orientation 화살표 초기화
        self.clear_orientation_arrows()
        
        # 현재 위치와 방향
        robot_pos, robot_orient = p.getBasePositionAndOrientation(self.robotId)
        current_yaw = p.getEulerFromQuaternion(robot_orient)[2]
        
        # 최종 목표 방향으로 먼저 회전
        angle_diff = final_yaw - current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        if abs(angle_diff) > 0.05:  # 5도 이상일 때만 회전
            print(f"🔄 목표 방향으로 회전: {angle_diff*180/math.pi:.1f}°")
            print(f"   현재 방향: {current_yaw*180/math.pi:.1f}°")
            print(f"   목표 방향: {final_yaw*180/math.pi:.1f}°")
            self.rotate_in_place_fast(final_yaw)
            
        # Manipulator 높이 조절 시작 (비동기로 연속적으로)
        if shelf_level:
            self.start_manipulator_continuous_motion(shelf_level)
        
        # 경로의 각 세그먼트를 직진으로 이동 (방향 유지)
        for i in range(len(path)-1):
            start_point = path[i]
            end_point = path[i+1]
            
            # 거리 계산
            dx = end_point[0] - start_point[0]
            dy = end_point[1] - start_point[1]
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance > 0.01:
                print(f"📍 세그먼트 {i+1}/{len(path)-1}: {distance:.2f}m")
                self.move_omnidrive_with_orientation(end_point, final_yaw)
                
        # 최종 위치 미세 조정
        final_pos = path[-1]
        current_pos = p.getBasePositionAndOrientation(self.robotId)[0]
        
        p.resetBasePositionAndOrientation(
            self.robotId,
            [final_pos[0], final_pos[1], current_pos[2]],
            p.getQuaternionFromEuler([0, 0, final_yaw])
        )
        
        # 최종 화살표
        self.add_orientation_arrow(final_pos[0], final_pos[1], final_yaw)
        
        print(f"\n✅ 경로 추종 완료!")
        print(f"📍 최종 위치: ({final_pos[0]:.2f}, {final_pos[1]:.2f})")
        print(f"🧭 최종 방향: {final_yaw*180/math.pi:.0f}°")
        if shelf_level:
            print(f"📚 현재 선반 레벨: {self.current_shelf_level}단")
        
        return True
        
    def move_omnidrive_with_orientation(self, target_pos, target_yaw):
        """Omnidrive 이동 - 특정 방향을 유지하며 이동"""
        start_pos, _ = p.getBasePositionAndOrientation(self.robotId)
        target_orient = p.getQuaternionFromEuler([0, 0, target_yaw])
        
        # 거리 계산
        dx = target_pos[0] - start_pos[0]
        dy = target_pos[1] - start_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.01:
            return
            
        # 이동 시간 계산
        move_time = distance / self.max_linear_speed
        steps = int(move_time * 30)  # 30Hz로 증가 (더 부드러운 비전)
        
        if steps == 0:
            steps = 1
            
        for step in range(steps + 1):
            t = step / float(steps)
            
            # 5차 다항식 프로파일
            s = 10*t**3 - 15*t**4 + 6*t**5
            
            # 현재 위치
            current_x = start_pos[0] + s * dx
            current_y = start_pos[1] + s * dy
            
            # 로봇 위치 업데이트 (목표 방향 유지)
            p.resetBasePositionAndOrientation(
                self.robotId,
                [current_x, current_y, start_pos[2]],
                target_orient
            )
            
            # Orientation 화살표 추가
            if step % 15 == 0:
                self.add_orientation_arrow(current_x, current_y, target_yaw)
            
            # 시뮬레이션 스텝
            for _ in range(8):  # 물리 스텝 줄임
                p.stepSimulation()
                
            # 비전/GUI 업데이트 (더 자주)
            if step % 3 == 0:  # 매 3스텝마다 업데이트
                self.update_robot_vision()
                self.capture_gui_frame()
                
    def move_omnidrive(self, target_pos):
        """Omnidrive 이동 - 현재 방향 유지하며 이동"""
        start_pos, orient = p.getBasePositionAndOrientation(self.robotId)
        
        # 거리 계산
        dx = target_pos[0] - start_pos[0]
        dy = target_pos[1] - start_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.01:
            return
            
        # 이동 시간 계산
        move_time = distance / self.max_linear_speed
        steps = int(move_time * 30)  # 30Hz로 증가 (더 부드러운 비전)
        
        if steps == 0:
            steps = 1
            
        for step in range(steps + 1):
            t = step / float(steps)
            
            # 5차 다항식 프로파일
            s = 10*t**3 - 15*t**4 + 6*t**5
            
            # 현재 위치
            current_x = start_pos[0] + s * dx
            current_y = start_pos[1] + s * dy
            
            # 로봇 위치 업데이트 (방향은 유지)
            p.resetBasePositionAndOrientation(
                self.robotId,
                [current_x, current_y, start_pos[2]],
                orient
            )
            
            # Orientation 화살표 추가
            if step % 15 == 0:
                current_yaw = p.getEulerFromQuaternion(orient)[2]
                self.add_orientation_arrow(current_x, current_y, current_yaw)
            
            # 시뮬레이션 스텝
            for _ in range(8):  # 물리 스텝 줄임
                p.stepSimulation()
                
            # 비전/GUI 업데이트 (더 자주)
            if step % 3 == 0:  # 매 3스텝마다 업데이트
                self.update_robot_vision()
                self.capture_gui_frame()
                
    def start_manipulator_continuous_motion(self, target_level):
        """Manipulator 연속 움직임 시작 (현재 높이에서 목표 높이로)"""
        if not hasattr(self, 'manipulator_joints') or not self.manipulator_joints:
            return
            
        # 레벨별 높이 설정
        level_heights = {
            1: 0.3,   # 1단 (낮음)
            2: 0.7,   # 2단 (중간)
            3: 1.2    # 3단 (높음)
        }
        
        target_height = level_heights.get(target_level, 0.7)
        
        # Slider-12 찾기 및 제어
        for joint_idx, joint_name in self.manipulator_joints:
            if "Slider-12" in joint_name or "ManipulatorStart" in joint_name:
                # 현재 높이에서 목표 높이로 부드럽게 이동
                p.setJointMotorControl2(
                    self.robotId,
                    joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=-target_height,  # 음수로 변환 (URDF 좌표계)
                    force=5000,
                    maxVelocity=1.0  # 적절한 속도
                )
            
    def add_orientation_arrow(self, x, y, yaw):
        """바닥에 orientation 화살표 추가"""
        arrow_length = 0.5
        arrow_head = 0.15
        
        # 화살표 끝점
        end_x = x + arrow_length * math.cos(yaw)
        end_y = y + arrow_length * math.sin(yaw)
        
        # 화살표 몸통
        arrow_id = p.addUserDebugLine(
            [x, y, 0.01],
            [end_x, end_y, 0.01],
            lineColorRGB=[1, 1, 0],
            lineWidth=5
        )
        self.orientation_arrows.append(arrow_id)
        
        # 화살표 머리
        head_angle1 = yaw + 2.8
        head_angle2 = yaw - 2.8
        
        head1_x = end_x + arrow_head * math.cos(head_angle1)
        head1_y = end_y + arrow_head * math.sin(head_angle1)
        
        head2_x = end_x + arrow_head * math.cos(head_angle2)
        head2_y = end_y + arrow_head * math.sin(head_angle2)
        
        head_id1 = p.addUserDebugLine(
            [end_x, end_y, 0.01],
            [head1_x, head1_y, 0.01],
            lineColorRGB=[1, 1, 0],
            lineWidth=5
        )
        head_id2 = p.addUserDebugLine(
            [end_x, end_y, 0.01],
            [head2_x, head2_y, 0.01],
            lineColorRGB=[1, 1, 0],
            lineWidth=5
        )
        
        self.orientation_arrows.extend([head_id1, head_id2])
        
    def clear_orientation_arrows(self):
        """orientation 화살표 제거"""
        for arrow_id in self.orientation_arrows:
            p.removeUserDebugItem(arrow_id)
        self.orientation_arrows = []
        
    def rotate_in_place_fast(self, target_yaw):
        """제자리 회전 - 정확한 목표 각도로 회전"""
        _, orient = p.getBasePositionAndOrientation(self.robotId)
        current_yaw = p.getEulerFromQuaternion(orient)[2]
        
        # 회전각 계산 (최단 경로)
        angle_diff = target_yaw - current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        print(f"   회전각: {angle_diff*180/math.pi:.1f}°")
            
        # 회전 시간 계산
        rotation_time = abs(angle_diff) / (self.max_angular_speed * 2)
        steps = int(rotation_time * 30)  # 30Hz로 증가
        
        if steps == 0:
            steps = 1
            
        for step in range(steps + 1):
            t = step / float(steps)
            
            # 5차 다항식 프로파일
            s = 10*t**3 - 15*t**4 + 6*t**5
            
            # 현재 각도
            interpolated_yaw = current_yaw + s * angle_diff
            
            # 로봇 위치 업데이트
            pos, _ = p.getBasePositionAndOrientation(self.robotId)
            new_orient = p.getQuaternionFromEuler([0, 0, interpolated_yaw])
            
            p.resetBasePositionAndOrientation(
                self.robotId,
                pos,
                new_orient
            )
            
            # Orientation 화살표 추가
            if step % 15 == 0:
                self.add_orientation_arrow(pos[0], pos[1], interpolated_yaw)
            
            # 시뮬레이션 스텝
            for _ in range(8):
                p.stepSimulation()
                
            # 비전/GUI 업데이트 (더 자주)
            if step % 3 == 0:  # 매 3스텝마다
                self.update_robot_vision()
                self.capture_gui_frame()
                
        # 최종 각도 확인 및 보정
        pos, _ = p.getBasePositionAndOrientation(self.robotId)
        final_orient = p.getQuaternionFromEuler([0, 0, target_yaw])
        p.resetBasePositionAndOrientation(self.robotId, pos, final_orient)
        current_yaw = p.getEulerFromQuaternion(orient)[2]
        
        # 회전각 계산 (최단 경로)
        angle_diff = target_yaw - current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        print(f"   회전각: {angle_diff*180/math.pi:.1f}°")
            
        # 회전 시간 계산
        rotation_time = abs(angle_diff) / (self.max_angular_speed * 2)
        steps = int(rotation_time * 20)  # 20Hz
        
        if steps == 0:
            steps = 1
            
        for step in range(steps + 1):
            t = step / float(steps)
            
            # 5차 다항식 프로파일
            s = 10*t**3 - 15*t**4 + 6*t**5
            
            # 현재 각도
            interpolated_yaw = current_yaw + s * angle_diff
            
            # 로봇 위치 업데이트
            pos, _ = p.getBasePositionAndOrientation(self.robotId)
            new_orient = p.getQuaternionFromEuler([0, 0, interpolated_yaw])
            
            p.resetBasePositionAndOrientation(
                self.robotId,
                pos,
                new_orient
            )
            
            # Orientation 화살표 추가
            if step % 10 == 0:
                self.add_orientation_arrow(pos[0], pos[1], interpolated_yaw)
            
            # 시뮬레이션 스텝
            for _ in range(10):
                p.stepSimulation()
                
            # 비전/GUI 업데이트
            if step % 10 == 0:
                self.update_robot_vision()
                self.capture_gui_frame()
                
        # 최종 각도 확인 및 보정
        pos, _ = p.getBasePositionAndOrientation(self.robotId)
        final_orient = p.getQuaternionFromEuler([0, 0, target_yaw])
        p.resetBasePositionAndOrientation(self.robotId, pos, final_orient)
                
    def optimize_shelf_sequence(self, shelf_list):
        """다중 책장 순회를 위한 최적 경로 계산"""
        print(f"\n📊 {len(shelf_list)}개 책장의 최적 순서 계산 중...")
        
        # 현재 로봇 위치
        robot_pos, _ = p.getBasePositionAndOrientation(self.robotId)
        current_pos = [robot_pos[0], robot_pos[1]]
        
        # 모든 책장의 접근 위치 계산
        shelf_positions = {}
        for shelf_code in shelf_list:
            if isinstance(shelf_code, tuple):
                code, level = shelf_code
            else:
                code, level = shelf_code, None
                
            approach_pos, _ = self.get_approach_position(code)
            if approach_pos:
                shelf_positions[shelf_code] = approach_pos
                
        if not shelf_positions:
            print("❌ 유효한 책장이 없습니다")
            return []
            
        # 거리 행렬 계산
        def manhattan_distance(pos1, pos2):
            return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
            
        # Greedy 알고리즘으로 최적 순서 찾기
        unvisited = list(shelf_positions.keys())
        path = []
        current = current_pos
        
        while unvisited:
            # 가장 가까운 책장 찾기
            min_dist = float('inf')
            nearest = None
            
            for shelf in unvisited:
                dist = manhattan_distance(current, shelf_positions[shelf])
                if dist < min_dist:
                    min_dist = dist
                    nearest = shelf
                    
            if nearest:
                path.append(nearest)
                current = shelf_positions[nearest]
                unvisited.remove(nearest)
                
        # 총 거리 계산
        total_distance = 0
        prev_pos = [robot_pos[0], robot_pos[1]]
        for shelf in path:
            next_pos = shelf_positions[shelf]
            total_distance += manhattan_distance(prev_pos, next_pos)
            prev_pos = next_pos
            
        print(f"✅ 최적 경로 발견!")
        print(f"📏 총 이동 거리: {total_distance:.2f}m")
        print(f"📋 방문 순서: {[s[0] if isinstance(s, tuple) else s for s in path]}")
        
        return path
        
    def navigate_multiple_shelves(self, shelf_list):
        """다중 책장 연속 방문"""
        print(f"\n{'='*70}")
        print(f" 📚 다중 책장 순회 시작 - {len(shelf_list)}개 목표 ".center(70))
        print(f"{'='*70}")
        
        # 녹화 시작
        self.start_recording()
        
        # 전체 경로에 대한 운동학 예측
        self.predict_joint_kinematics_multi(shelf_list)
        
        # 최적 순서 계산
        optimized_sequence = self.optimize_shelf_sequence(shelf_list)
        
        if not optimized_sequence:
            print("❌ 순회할 수 없습니다")
            return False
            
        # 각 책장 순차 방문
        success_count = 0
        for i, target in enumerate(optimized_sequence):
            print(f"\n[{i+1}/{len(optimized_sequence)}] ", end='')
            
            if isinstance(target, tuple):
                shelf_code, level = target
                success = self.navigate_to_shelf(shelf_code, level)
            else:
                success = self.navigate_to_shelf(target)
                
            if success:
                success_count += 1
                # 각 책장에서 작업 시뮬레이션
                print("📖 책 정리 작업 중...")
                
                # 작업 후 상태 리셋
                self.at_shelf = False
                self.platform_extended = False
                
                for _ in range(10):
                    p.stepSimulation()
                    self.update_robot_vision()
                    self.capture_gui_frame()
                    time.sleep(0.02)
            else:
                print(f"❌ {target} 방문 실패")
                
        # 녹화 종료
        self.stop_recording()
        
        print(f"\n{'='*70}")
        print(f"✅ 순회 완료! 성공률: {success_count}/{len(optimized_sequence)}")
        print(f"{'='*70}")
        
        return success_count == len(optimized_sequence)
        
    def interactive_navigation(self):
        """대화형 네비게이션"""
        print("\n" + "="*70)
        print(" 📚 BIBLIOBOT LIBRARY NAVIGATION SYSTEM ".center(70))
        print(" High-Speed Navigation with Kinematics Analysis ".center(70))
        print("="*70)
        
        # 사용 가능한 책장 표시
        shelves_by_area = {
            'Aisle A': [],
            'Aisle B': [],
            'Aisle C': [],
            'Top Row 1 (T1-T7)': [],
            'Top Row 2 (T8-T14)': []
        }
        
        for code in sorted(self.shelves.keys()):
            if code.startswith('A'):
                shelves_by_area['Aisle A'].append(code)
            elif code.startswith('B'):
                shelves_by_area['Aisle B'].append(code)
            elif code.startswith('C'):
                shelves_by_area['Aisle C'].append(code)
            elif code.startswith('T'):
                num = int(code[1:])
                if num <= 7:
                    shelves_by_area['Top Row 1 (T1-T7)'].append(code)
                else:
                    shelves_by_area['Top Row 2 (T8-T14)'].append(code)
                    
        print("\n📚 사용 가능한 책장:")
        for area, codes in shelves_by_area.items():
            if codes:
                print(f"  {area}: {', '.join(codes)}")
                
        print("\n💡 명령:")
        print("  • 책장 코드 입력 (예: A1L)")
        print("  • 책장 코드와 레벨 입력 (예: A1L 3)")
        print("  • 다중 책장 입력 (예: A1L,B2R,C3L 또는 A1L:2,B2R:3,C3L:1)")
        print("  • 'demo' - 자동 시연")
        print("  • 'record' - 녹화 시작/중지")
        print("  • 'q' - 종료")
        
        while True:
            try:
                user_input = input("\n🎯 목표 입력: ").strip().upper()
                
                if user_input == 'Q':
                    break
                elif user_input == 'DEMO':
                    self.run_demo_sequence()
                elif user_input == 'RECORD':
                    if self.recording:
                        self.stop_recording()
                    else:
                        self.start_recording()
                elif ',' in user_input:
                    # 다중 책장 입력 처리
                    shelf_list = []
                    for item in user_input.split(','):
                        item = item.strip()
                        if ':' in item:
                            code, level = item.split(':')
                            shelf_list.append((code, int(level)))
                        else:
                            shelf_list.append(item)
                    self.navigate_multiple_shelves(shelf_list)
                else:
                    # 단일 책장 입력
                    parts = user_input.split()
                    if len(parts) == 1:
                        # 책장만 입력
                        if parts[0] in self.shelves:
                            self.navigate_to_shelf(parts[0])
                        else:
                            print(f"❌ 잘못된 책장 코드: {parts[0]}")
                    elif len(parts) == 2:
                        # 책장과 레벨 입력
                        shelf_code = parts[0]
                        try:
                            level = int(parts[1])
                            if shelf_code in self.shelves and 1 <= level <= 3:
                                self.navigate_to_shelf(shelf_code, level)
                            else:
                                print(f"❌ 잘못된 입력: 책장 코드 또는 레벨(1-3) 확인")
                        except ValueError:
                            print(f"❌ 레벨은 숫자(1-3)여야 합니다")
                    else:
                        print("❌ 형식: '책장코드' 또는 '책장코드 레벨'")
                    
            except KeyboardInterrupt:
                break
                
    def run_demo_sequence(self):
        """데모 시퀀스 실행"""
        print("\n🎬 자동 시연 시작...")
        
        # 다중 책장 순회 데모
        demo_shelves = [
            ('A1L', 1),    # 1단
            ('B4R', 3),    # 3단 (위로)
            ('C4L', 2),    # 2단 (아래로)
            'T5',          # 2단 유지
            'T10'          # 2단 유지
        ]
        
        self.navigate_multiple_shelves(demo_shelves)
            
        print("\n✅ 시연 완료!")
        
    def run(self):
        """메인 실행 함수"""
        try:
            # 초기 안정화
            print("\n⏳ 시뮬레이션 초기화 중...")
            for _ in range(50):
                p.stepSimulation()
                
            print("✅ 준비 완료!")
            
            # 대화형 네비게이션 시작
            self.interactive_navigation()
            
        except Exception as e:
            print(f"❌ 오류 발생: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 녹화 중이면 저장
            if self.recording:
                self.stop_recording()
                
            cv2.destroyAllWindows()
            plt.close('all')
            p.disconnect()
            print("\n👋 시뮬레이션 종료")
            
if __name__ == "__main__":
    nav_system = BibliobotLibraryNavigation()
    nav_system.run()