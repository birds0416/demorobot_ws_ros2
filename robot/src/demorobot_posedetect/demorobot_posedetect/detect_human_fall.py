from typing import List
from ultralytics import YOLO
import cv2
import configparser
import time
import logging
from cv_bridge import CvBridge
import numpy as np
import torch

#ros message setting
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray
from demorobot_msg.msg import Detect

bridge = CvBridge()

class EventPublisher(Node):
    def __init__(self):
        super().__init__('event_pub')
        self.NAMESPACE = self.get_namespace()
        self.event_pub = self.create_publisher(String, self.NAMESPACE + '/pose_detect/event', 10)

class DetectPublisher(Node):
    def __init__(self):
        super().__init__('detect_pub')
        self.NAMESPACE = self.get_namespace()
        self.detect_pub = self.create_publisher(Detect, self.NAMESPACE + '/pose_detect/detect_points', 10)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_sub')
        self.NAMESPACE = self.get_namespace()
        qos = QoSProfile(depth=10)
        self.image_sub = self.create_subscription(
            Image,
            self.NAMESPACE + '/camera/color/image_raw',
            self.image_callback,
            qos
        )
        self.image = np.empty(shape=[1])
        self.img_w = 0
        self.img_h = 0
        

    def image_callback(self, data):
        self.image = bridge.imgmsg_to_cv2(data, 'bgr8')
        self.img_w = data.width
        self.img_h = data.height
        # cv2.imshow('img', self.image)
        # cv2.waitKey(5)
        
class EmptyNode(Node):
    def __init__(self):
        super().__init__('empty_node')


def main(args=None):
    rclpy.init(args=args)
    event_node = EventPublisher()
    evt_msg = String()  

    detect_node = DetectPublisher()
    detect_msg = Detect()

    img_node = ImageSubscriber()
    
    empty_node = EmptyNode()

    # Image saving publisher
    img_saver_node = rclpy.create_node('img_saver_node')
    pub_img_saver = img_saver_node.create_publisher(Image, 'save_img', 10)
    pub_img_empty_saver = img_saver_node.create_publisher(Image, 'save_img_empty', 10)

    #log file 설정
    logging.basicConfig(filename="./log_file_n.txt", level=logging.DEBUG, 
                        format="[ %(asctime)s | %(levelname)s ] %(message)s", 
                        datefmt="%Y-%m-%d %H:%M:%S")    
    logger = logging.getLogger()    

    # config 읽어오기
    properties = configparser.ConfigParser()
    properties.read('./src/demorobot_posedetect/demorobot_posedetect/config.ini')

    default = properties["DEFAULT"] #기본 세팅 목록
    timeset = properties["TIMESET"] #시간 관련 세팅 목록    

    event_count = timeset.getint("event_count") # 이벤트 반복 횟수
    fall_time = timeset.getfloat("fall_time") # 넘어짐 이벤트 판단 기준 시간 
    delay = timeset.getfloat("delay") # 이벤트 반복 재생 중간의 시간 
    event_time = timeset.getfloat("event_time") # 이벤트 반복 재생 후 다음 이벤트 발생 주기까지의 시간  
    fall_queue_num = timeset.getint("fall_queue") #넘어짐 감지 프레임 큐 크기
    fall_halfqueue = fall_queue_num / 2
    fall_queue = [] 

    #실행 모드 설정
    #0:화면 표시 없음, 1:감지 결과 표시
    mode = int(default["execution_mode"])

    #camera mode
    #0:use video, 1:use camera
    cam_mode = int(default["cam_mode"])

    #사용할 모델 설정
    device: str = "cuda" if torch.cuda.is_available() else "cpu"
    model = YOLO(default["model"])
    model.to(device)

    #입력 영상 설정
    video_path = default["video_path"]
    if cam_mode == 0:
        cap = cv2.VideoCapture(video_path)
        # out = cv2.VideoWriter("video/Drunk People Falling Compilation_falldetection.mp4",cv2.VideoWriter_fourcc(*'DIVX'),cap.get(cv2.CAP_PROP_FPS),(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))  
    
    # 시간 설정
    fall_timer = 0 #넘어짐 감지 타이머
    delay_timer = 0 #이벤트 반복 재생 타이머
    event_timer = 0 #이벤트 사이클 타이머

    #color
    blue_color = (255, 0, 0)
    green_color = (0, 255, 0)
    red_color = (0, 0, 255)
    orange_color = (60, 153, 255)
    
    # FPS
    prev_time = 0
    total_frames = 0
    start_time = time.time()

    #키포인트 높이 비교
    def keypoint_comp(pred_kpts_xy):
        keypoints = pred_kpts_xy[0]
        index = 0
        temp_keypoint = []
        for keypoint in keypoints:
            temp_keypoint.append(keypoint[0])
            temp_keypoint.append(keypoint[1])
            index += 1
            if keypoint[0] == 0 and keypoint[1] == 0:
                continue
            else:
                if index <= 5:
                    cv2.circle(annotated_frame, (int(keypoint[0]),int(keypoint[1])), 5, green_color, -1)
                elif index <= 11:
                    cv2.circle(annotated_frame, (int(keypoint[0]),int(keypoint[1])), 5, blue_color, -1)
                elif index <= 17:
                    cv2.circle(annotated_frame, (int(keypoint[0]),int(keypoint[1])), 5, orange_color, -1)
        # detect_msg.data = temp_keypoint

        #감지된 keypoint X,Y 값 저장
        upper_body = [] # 상체(코,눈,귀,어깨) y좌표 리스트
        lower_body = [] # 하체(발목,무릎,골반) y좌표 리스트
        gap = 200 # 하체 - gap 만큼의 높이 보다 상체의 높이가 낮아졌을 경우 넘어졌다고 판단

        #상체 윗 부분 부터 y좌표 저장
        for kp in keypoints[0:7]:
            upper_body.append(kp[1])    

        #하체 아랫 부분 부터 y좌표 저장
        for kp in keypoints[16:10:-1]:
            lower_body.append(kp[1])    

        for upb in upper_body:
            if upb: #상체 윗 부분 부터 0(미감지)이 아닌 경우
                for lwb in lower_body:
                    if lwb: #하체 아랫 부분 부터 0(미감지)이 아닌 경우 
                        if upb > (lwb - gap): #상체의 높이가 하체 - gqp 만큼의 높이보다 낮을 경우
                            return True
                        else:
                            return False
                    else:
                        continue
            else:
                continue    

        return False  

    	  
    
    while True: # 프레임 단위로 반복.
        if cam_mode == 0:
            ret, frame = cap.read()
            tf = ret
        elif cam_mode == 1:
            rclpy.spin_once(img_node)
            frame = img_node.image
            tf = True

        w = img_node.img_w
        h = img_node.img_h
        empty_node.get_logger().info("cap dimension - width: {}, height: {}".format(w, h))

        # for publish image as jpg
        frame_copy = frame.copy()
        
        # FPS
        curr_time = time.time()
        total_frames = total_frames + 1
        term = curr_time - prev_time
        fps = 1 / term
        prev_time = curr_time
        fps_string = f'term = {term:.3f}, FPS = {fps:.2f}'
        empty_node.get_logger().info(fps_string)

        if tf:
            #입력 프레임 감지
            results = model.predict(frame)[0].cpu()
            #감지결과 프레임 저장
            annotated_frame = frame
            try:
                people_fall_count = 0 # 해당 프레임에 넘어져 있는 사람이 있는지 카운트
                #감지 된 각 객체들을 분석
                for result in results:
                    #감지 정확도 낮을 시 건너뛰는 로직 필요
                    # score = float(result.boxes.conf[0])
                    # if score < 0.5:
                    #     continue  

                    #감지된 box 값 저장
                    pred_box_xywh = result.boxes.xywh.numpy()
                    box_w = pred_box_xywh[0][2]
                    box_h = pred_box_xywh[0][3]
                    box_rate = box_w / box_h #box의 세로길이대비 가로길이의 비  

                    pred_box_xyxy = result.boxes.xyxy.numpy()
                    box_x = int(pred_box_xyxy[0][0])
                    box_y = int(pred_box_xyxy[0][1])
                    box_xx = int(pred_box_xyxy[0][2])
                    box_yy = int(pred_box_xyxy[0][3])   

                    # 디텍션 박스의 가로세로 비율과 머리, 발 위치를 같이 사용해 넘어짐 감지
                    # (머리나 발이 감지 안될 경우 다른 상체, 하체 부위 사용할 수 있게, 앉아있을 경우에도 가능하게 고민)
                    keypoint_comp_result = keypoint_comp(result.keypoints.xy.numpy())
                    # detect_msg.box = pred_box_xyxy[0]
                    if box_rate > 0.5 and keypoint_comp_result:
                        people_fall_count += 1
                        detect_msg.fall = True
                        cv2.putText(annotated_frame, "FALL", (box_x-5,box_y-5),0,1,red_color,2)
                        cv2.rectangle(annotated_frame,(box_x,box_y),(box_xx,box_yy),red_color,2)
                    else:
                        detect_msg.fall = False
                        cv2.putText(annotated_frame, "Person", (box_x-5,box_y-5),0,1,green_color,2)
                        cv2.rectangle(annotated_frame,(box_x,box_y),(box_xx,box_yy),green_color,2) 
                    
                    detect_node.detect_pub.publish(detect_msg)

                if len(fall_queue) >= fall_queue_num:
                    fall_queue.pop(0)   
                fall_queue.append(people_fall_count)    

                if len(fall_queue) >= fall_queue_num:
                    trueCnt = 0

                    for fall in fall_queue:

                        if fall:
                            trueCnt += 1    

                    if trueCnt > fall_halfqueue:

                        if fall_timer == 0 and event_timer == 0: #넘어짐 타이머가 0일 경우 현재 시간 입력
                            fall_timer = time.time()

                        elif (time.time() - fall_timer) >= fall_time and delay_timer == 0 and (event_timer == 0 or (time.time() - event_timer) >= event_time): #첫 이벤트 발생 조건
                            print("--------------------넘어짐 감지 이벤트 발생--------------------")
                            logger.info("넘어짐 감지 이벤트 발생")
                            evt_msg.data = "EVT-001" #event on
                            event_node.event_pub.publish(evt_msg)

                            # img saving code
                            save_img = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                            save_img_empty = bridge.cv2_to_imgmsg(frame_copy, encoding='bgr8')
                            pub_img_saver.publish(save_img)
                            pub_img_empty_saver.publish(save_img_empty)

                            # fall_timer = 0
                            delay_timer = time.time()
                            event_count -= 1

                            if event_count == 0: # event_count 만큼 이벤트 발생 시 한 번의 이벤트 발생 주기 종료 후 다음 주기 까지 이벤트 발생 X
                                logger.info("이벤트 사이클 종료, 다음 사이클 대기 시작")
                                # msg.data = "event cycle finish"
                                # minimal_publisher.event_pub.publish(msg)
                                event_count = timeset.getint("event_count")
                                delay_timer = 0
                                event_timer = time.time()

                        elif not delay_timer == 0 and (time.time() - delay_timer) >= delay: # 첫 이벤트 발생 이후 재발생 조건
                            print("--------------------넘어짐 감지 이벤트 재발생--------------------")
                            logger.info("넘어짐 감지 이벤트 재발생")
                            evt_msg.data = "EVT-002" #event re on
                            event_node.event_pub.publish(evt_msg)

                            # img saving code
                            save_img = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                            save_img_empty = bridge.cv2_to_imgmsg(frame_copy, encoding='bgr8')
                            pub_img_saver.publish(save_img)
                            pub_img_empty_saver.publish(save_img_empty)

                            delay_timer = time.time()
                            event_count -= 1

                            if event_count == 0: # event_count 만큼 이벤트 발생 시 한 번의 이벤트 발생 주기 종료 후 다음 주기 까지 이벤트 발생 X
                                logger.info("이벤트 사이클 종료, 다음 사이클 대기 시작")
                                # msg.data = "event cycle finish"
                                # minimal_publisher.event_pub.publish(msg)
                                event_count = timeset.getint("event_count")
                                delay_timer = 0
                                event_timer = time.time()
            
                    else:
                        if event_timer == 0:
                            fall_timer = 0

                        if delay_timer and (time.time() - delay_timer) >= delay: # delay_timer 가 delay 만큼 시간이 되었는데 프레임 내에 넘어진 사람이 한명도 없는 경우 delay_timer 초기화
                            delay_timer = 0 

                if not event_timer == 0 and (time.time() - event_timer) >= event_time: #event_timer 가 event_time 만큼 되었을 경우 fall_timer 초기화
                    fall_timer = 0
                    event_timer = 0 
                
                if mode == 0:
                    continue
                elif mode == 1:
                    cv2.imshow("YOLOv8 Inference", annotated_frame)
                    # out.write(annotated_frame)
                

            except IndexError:
                print('array error')

                if mode == 0:
                    continue
                elif mode == 1:
                    cv2.imshow("YOLOv8 Inference", annotated_frame)
                    # out.write(annotated_frame)   

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break   

        else:
            break

    end_time = time.time()
    fps = total_frames / (start_time - end_time)
    PRINT_STRING = f'total_frames = {total_frames}, avg FPS = {fps:.2f}'
    empty_node.get_logger().info(PRINT_STRING)
    
    if cam_mode == 0:
        cap.release()
        # out.release()
    cv2.destroyAllWindows()
    if cam_mode == 1:
        img_node.destroy_node()
    detect_node.destroy_node()
    event_node.destroy_node()
    rclpy.shutdown()    

if  __name__ == '__main__':
    main()
