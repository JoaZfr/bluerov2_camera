#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import os
import shutil
from ultralytics import YOLO
from bluerov2_interfaces.msg import TargetInfo

frames_dir = "/home/joachim/Documents/br2_ws/src/bluerov2_comms/bluerov2_comms/logs/log1/camera_images"
if os.path.exists(frames_dir):
    shutil.rmtree(frames_dir)
os.makedirs(frames_dir)


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        ## ================== SUBSCRIBERS ================== ##
        self.subscription_f = self.create_subscription( Image, '/frontal', self.frontal_listener_callback, 10)

        ## ================== PUBLISHERS =================== ##
        self.target_info_publisher = self.create_publisher(TargetInfo, '/target_info', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Camera Viewer Node Started")
        self.load_model()

    def load_model(self):
        package_dir = get_package_share_directory('bluerov2_camera')
        self.model_filename = os.path.join(package_dir, 'model', 'best.pt')

        self.model = YOLO(self.model_filename)

        self.model.to("cuda")
        self.get_logger().info("model loaded")

    def process_image(self, cv_image, img_width, img_height):
        """
        Prend une image OpenCV, prédit avec le modèle, dessine la box avec la plus haute confiance,
        affiche un point rouge sur le centre, et log la position relative au centre de l'image.
        Retourne l'image annotée.
        """
        msg = TargetInfo()


        results = self.model.predict(cv_image, device="cuda")[0]  # Batch size = 1

        if len(results.boxes) == 0:
            msg.conf = 0.0
            self.target_info_publisher.publish(msg)    
            return cv_image  

        # Trier les boxes selon la confiance (décroissante)
        boxes = sorted(results.boxes, key=lambda b: float(b.conf[0]), reverse=True)

        # On garde uniquement la box la plus fiable
        box = boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        label = f"{self.model.model.names[cls_id]} {conf:.2f}"

        # Dessin de la box et du label
        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(cv_image, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        # Calcul et dessin du point rouge
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

        # Calcul de la position relative au centre de l'image
        rel_x = int(center_x - img_width / 2)
        rel_y = int(center_y - img_height / 2)

        # Log
        self.get_logger().info(
            f"Target : x: {rel_x:.1f}, y: {rel_y:.1f} | Confidence : {conf:.2f}"
        )

        msg.x, msg.y, msg.conf= rel_x, rel_y, conf
        self.target_info_publisher.publish(msg)

        return cv_image

    def frontal_listener_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Traitement et affichage
        processed_image = self.process_image(cv_image, msg.width, msg.height)

        # Affichage de l'image annotée
        cv2.namedWindow("Front Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Front Image", processed_image)
        cv2.resizeWindow("Front Image", 800, 600)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    viewer = CameraViewer()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
