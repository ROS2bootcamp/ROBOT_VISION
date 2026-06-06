import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from ultralytics import YOLO

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        rgb_topic_name = '/camera/image' 
        depth_topic_name = '/camera/depth_image'
        self.bridge = CvBridge()

        # RGB 이미지 구독
        self.image_sub = self.create_subscription(Image, rgb_topic_name, self.image_callback, 10)
        # Depth 이미지 구독
        self.depth_sub = self.cr (실행 전 매번 source install/setup.bash 적용 필수)eate_subscription(Image, depth_topic_name, self.depth_callback, 10)
        
        # 로봇암 제어 노드가 구독할 토픽
        self.result_pub = self.create_publisher(String, '/vision/detection_results', 10)
        
        # Depth 이미지 저장 변수
        self.latest_depth_image = None

        # tf2 버퍼 및 리스너 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Loading YOLOv8 Model')
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info('Loading Complete!')

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth 변환 에러: {e}")

    def image_callback(self, msg):
        try:
            # ROS 2 이미지를 OpenCV 형식(BGR)으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # YOLO 추론 진행 (CUDA가 세팅되어 있다면 device='cuda' 파라미터 추가)
            results = self.model(cv_image, verbose=False)
            # 인식된 결과를 이미지 위에 그리기 (바운딩 박스)
            annotated_frame = results[0].plot()

            # 프레임에서 발견된 모든 객체 정보를 담을 리스트
            frame_detections = []

            # 바운딩 박스 중심점 계산 및 출력
            for box in results[0].boxes:
                # 바운딩 박스 좌상단(x1, y1), 우하단(x2, y2) 추출
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 중심점 계산
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                # 초록색 조준점 그리기
                cv2.circle(annotated_frame, (u, v), 5, (0, 255, 0), -1)

                # 클래스 id 확인
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                confidence = float(box.conf[0])

                # 거리 측정 로직
                distance = 0.0
                X_cam, Y_cam, Z_cam = 0.0, 0.0, 0.0
                X_base, Y_base, Z_base = None, None, None

                if self.latest_depth_image is not None:
                    # 이미지 배열은 y(행), x(열) 순서로 접근
                    # IndexError 처리
                    if 0 <= v < self.latest_depth_image.shape[0] and 0 <= u < self.latest_depth_image.shape[1]:
                        distance = self.latest_depth_image[v, u]

                        # 거리가 무한대이거나 측정 불가인 경우 처리
                        if np.isnan(distance) or np.isinf(distance):
                            distance = -1.0
                
                if distance > 0:
                    f_x = 554.25
                    f_y = 554.25
                    c_x = 320.0
                    c_y = 240.0

                    X_cam = float((u - c_x) * distance / f_x)
                    Y_cam = float((v - c_y) * distance / f_y)
                    Z_cam = float(distance)

                    # tf2 변환 로직
                    try:
                        # 현재 시간을 기준으로 camera_link 에서 base_link로의 변환 행렬을 가져옴
                        t = self.tf_buffer.lookup_transform(
                            'base_link', # 타겟 프레임
                            'camera_link', # 소스 프레임
                            rclpy.time.Time()
                        )

                        # 카메라 기준 좌표를 PointStamped 메시지로 만듦
                        p_cam = PointStamped()
                        p_cam.point.x = X_cam
                        p_cam.point.y = Y_cam
                        p_cam.point.z = Z_cam

                        p_base = tf2_geometry_msgs.do_transform_point(p_cam, t)

                        X_base = float(round(p_base.point.x, 3))
                        Y_base = float(round(p_base.point.y, 3))
                        Z_base = float(round(p_base.point.z, 3))
                    except Exception as e:
                        pass

                    # 거리를 텍스트로 출력
                    text = f"{distance:.2f}m"
                    cv2.putText(annotated_frame, text, (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
 
                obj_data = {
                    "class_name": str(class_name),
                    "confidence": float(round(confidence, 3)),
                    "center_2d": {"u": int(u), "v": int(v)},
                    "distance_m": float(round(distance, 3)),
                    "position_3d_camera_frame": {
                        "X": float(round(X_cam, 3)),
                        "Y": float(round(Y_cam, 3)),
                        "Z": float(round(Z_cam, 3))
                    },
                    "position_3d_base_frame": {
                        "X": X_base,
                        "Y": Y_base,
                        "Z": Z_base
                    }
                }
                frame_detections.append(obj_data)

            # 프레임 내 모든 객체 정보가 수집되면 하나의 JSON으로 패키징하여 퍼블리시    
            if frame_detections:
                # 타임스탬프와 함께 최종 딕셔너리 생성
                result_dict = {
                    "timestamp_ns": self.get_clock().now().nanoseconds,
                    "num_detections": len(frame_detections),
                    "objects": frame_detections
                }

                # 파이썬 딕셔너리를 JSON 포맷 문자열로 변환
                json_string = json.dumps(result_dict, ensure_ascii=False)
                
                # ros2 string 메시지에 담아서 발행
                msg_out = String()
                msg_out.data = json_string
                self.result_pub.publish(msg_out)
                
                # json 출력
                self.get_logger().info(f"Published JSON:\n{json.dumps(result_dict, indent=2)}")

            # 화면에 띄우기
            cv2.imshow("Gazebo YOLO Detection", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"이미지 변환 및 처리 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()