from typing import List
from ultralytics import YOLO
import cv2
import configparser
import time
import logging
from cv_bridge import CvBridge
import numpy as np
import torch
import datetime
import math

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
        if self.NAMESPACE == "/":
            self.NAMESPACE = ""
            
        self.event_pub = self.create_publisher(String, self.NAMESPACE + '/pose_detect/event', 10)

class DetectPublisher(Node):
    def __init__(self):
        super().__init__('detect_pub')
        self.NAMESPACE = self.get_namespace()
        if self.NAMESPACE == "/":
            self.NAMESPACE = ""
            
        self.detect_pub = self.create_publisher(Detect, self.NAMESPACE + '/pose_detect/detect_points', 10)
        self._infer_pub = self.create_publisher(Image, self.NAMESPACE + '/pose_detect/detect_img', 10)
        self._depth_pub = self.create_publisher(Image, self.NAMESPACE + '/pose_detect/depth_frame', 10)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_sub')
        self.NAMESPACE = self.get_namespace()
        if self.NAMESPACE == "/":
            self.NAMESPACE = ""
            
        qos = QoSProfile(depth=10)
        self.image_sub = self.create_subscription(
            Image,
            self.NAMESPACE + '/camera/color/image_raw',
            self.image_callback,
            qos
        )
        self.image = np.empty(shape=[1])
        self.depth_img = np.empty(shape=[1])
        self.img_w = 0
        self.img_h = 0
        
        self.img_values = []
        self.depth_image_sub = self.create_subscription(
            Image,
            self.NAMESPACE + '/camera/depth/image_raw', 
            self.depth_image_callback, 
            qos
        )

        self.img_encoding = None
        self.dep_encoding = None

    def image_callback(self, data):
        self.image = bridge.imgmsg_to_cv2(data, 'bgr8')
        self.img_encoding = data.encoding
        self.img_w = data.width
        self.img_h = data.height
    
    def depth_image_callback(self, msg):
        ''' Version 3'''
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = cv_img.shape
        '''
        applyColorMap list
        cv2.COLORMAP_AUTUMN
        cv2.COLORMAP_BONE
        cv2.COLORMAP_JET
        cv2.COLORMAP_WINTER
        cv2.COLORMAP_RAINBOW
        cv2.COLORMAP_OCEAN
        cv2.COLORMAP_SUMMER
        cv2.COLORMAP_SPRING
        cv2.COLORMAP_COOL
        cv2.COLORMAP_HSV
        cv2.COLORMAP_PINK
        cv2.COLORMAP_HOT
        cv2.COLORMAP_PARULA
        cv2.COLORMAP_MAGMA
        cv2.COLORMAP_INFERNO
        cv2.COLORMAP_PLASMA
        cv2.COLORMAP_VIRIDIS
        cv2.COLORMAP_CIVIDIS
        cv2.COLORMAP_TWILIGHT
        cv2.COLORMAP_TWILIGHT_SHIFTED
        cv2.COLORMAP_TURBO
        cv2.COLORMAP_DEEPGREEN
        '''
        normalized_depth_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        self.depth_img = cv2.applyColorMap(normalized_depth_img, cv2.COLORMAP_BONE)
        self.dep_encoding = "16UC1"
        # msg.encoding: 16UC1
        self.img_values = np.zeros((h, w), dtype=np.uint16)
        self.img_values = cv_img.astype(np.uint16)

class EmptyNode(Node):
    def __init__(self):
        super().__init__('empty_node')

def non_zero_cnt(nums_x, nums_y):
    res = sum(1 for num in nums_x if num != 0)
    for i in range(len(nums_x)):
        if nums_x[i] == 0 and nums_y[i] == 0:
            res += 1
    return res

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
    properties.read('./cornersdev/demorobot_ws/src/demorobot_posedetect/demorobot_posedetect/config.ini')

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

    now = datetime.datetime.now().strftime("%Y-%m-%d %H-%M-%S")
    # distance_f = open("./src/demorobot_posedetect/logs/{}_keypoint_distance.txt".format(now), 'w+')
    # distance_f = open("./src/demorobot_posedetect/distances/{}_distance.txt".format(now), 'w+')

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

        empty_node.get_logger().info("camera img width: {}".format(w))
        empty_node.get_logger().info("camera img height: {}".format(h))

        # Depth 이미지 처리
        depth_pix = img_node.img_values
        depth_frame = img_node.depth_img

        # for publish image as jpg
        frame_copy = frame.copy()

        # FPS
        curr_time = time.time()
        total_frames = total_frames + 1
        term = curr_time - prev_time
        fps = 1 / term
        prev_time = curr_time
        fps_string = f'term = {term:.3f},  FPS = {fps:.2f}'
        empty_node.get_logger().info(fps_string)

        if tf:
            #입력 프레임 감지
            results = model.predict(frame)[0].cpu()
            #감지결과 프레임 저장
            annotated_frame = frame
            # empty_node.get_logger().info("annotated frame: {}".format(annotated_frame.shape))

            try:
                people_fall_count = 0 # 해당 프레임에 넘어져 있는 사람이 있는지 카운트
                #감지 된 각 객체들을 분석

                if results != None:
                    for result in results:
                        #감지 정확도 낮을 시 건너뛰는 로직 필요
                        score = float(result.boxes.conf[0])
                        if score < 0.6:
                            continue  

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

                        detect_msg.box = [box_x, box_y, box_xx, box_yy]

                        keypoint_datas = result.keypoints.xy.numpy()[0]
                        detect_msg.data = []
                        for data in keypoint_datas:
                            detect_msg.data.append(data[0])
                            detect_msg.data.append(data[1]) 

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
                        
                        #region 계산 1
                        # 좌표 비율 계산 (yolo: 640x384)
                        # box_x_mod = int(box_x * (480 / 384))
                        # box_xx_mod = int(box_xx * (480 / 384))
                        # box_y_mod = int(box_y * (480 / 384))
                        # box_yy_mod = int(box_yy * (480 / 384))
                        
                        # box_mid_x = int((box_x_mod + box_xx_mod) / 2)
                        # box_mid_y = int((box_y_mod + box_yy_mod) / 2)
                        #endregion 계산 1

                        #region 계산 2
                        box_mid_x = int((box_x + box_xx) / 2)
                        box_mid_y = int((box_y + box_yy) / 2)

                        # distance = float(depth_pix[240][box_mid_x]) / 1000
                        # empty_node.get_logger().info("This is Distance: {}m".format(distance))
                        
                        # box_dist = round(distance, 2)

                        # dist_text = str(box_dist) + 'm'

                        # cv2.putText(annotated_frame, dist_text, (box_mid_x, box_mid_y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
                        # cv2.circle(annotated_frame, (box_mid_x, box_mid_y), radius=0, color=(0, 0, 255), thickness=1)
                        #endregion 계산 2
                        
                        #region 계산 3
                        # # 어깨 2 pnts
                        # # idx 5, 6 average
                        # idx_5_x = keypoint_datas[5][0]
                        # idx_5_y = keypoint_datas[5][1]
                        # idx_6_x = keypoint_datas[6][0]
                        # idx_6_y = keypoint_datas[6][1]
                        # # x_avg_sho = int((idx_5_x + idx_6_x) / 2)

                        # # 골반 2 pnts
                        # # idx 11, 12 average
                        # idx_11_x = keypoint_datas[11][0]
                        # idx_11_y = keypoint_datas[11][1]
                        # idx_12_x = keypoint_datas[12][0]
                        # idx_12_y = keypoint_datas[12][1]
                        # # x_avg_hip = int((idx_11_x + idx_12_x) / 2)

                        # # 무릎 2 pnts
                        # # idx 13, 14 average
                        # idx_13_x = keypoint_datas[13][0]
                        # idx_13_y = keypoint_datas[13][1]
                        # idx_14_x = keypoint_datas[14][0]
                        # idx_14_y = keypoint_datas[14][1]
                        # # x_avg_kne = int((idx_13_x + idx_14_x) / 2)

                        # # 발 2 pnts
                        # # idx 13, 14 average
                        # idx_15_x = keypoint_datas[15][0]
                        # idx_15_y = keypoint_datas[15][1]
                        # idx_16_x = keypoint_datas[16][0]
                        # idx_16_y = keypoint_datas[16][1]
                        # # x_avg_ank = int((idx_15_x + idx_16_x) / 2)

                        # x_list = [idx_5_x, idx_6_x, idx_11_x, idx_12_x, idx_13_x, idx_14_x, idx_15_x, idx_16_x]
                        # y_list = [idx_5_y, idx_6_y, idx_11_y, idx_12_y, idx_13_y, idx_14_y, idx_15_y, idx_16_y]

                        # non_zero = non_zero_cnt(x_list, y_list)

                        # x_avg = 0
                        # if non_zero > 0:
                        #     x_avg = int(sum(x_list) / non_zero)
                        # else:
                        #     x_avg = 0
                        #endregion 계산 3

                        #region 계산 4
                        # x_sum = 0
                        # x_avg = 0
                        # cnt = 0
                        # distance_f.write(now + " - idx " + str(box_idx) + ":\n")
                        # distance_f.write("[\n")
                        # for data in keypoint_datas:
                        #     if data[0] != 0 or data[1] != 0:
                        #         x_sum += data[0]
                        #         distance_f.write("\t"+ str(data[0]) + ", ")
                        #         # x_sum += data[0] * (480 / 384)
                        #         cnt += 1
                        # distance_f.write("\n]\n")
                        # if cnt != 0:
                        #     x_avg = int(x_sum / cnt)
                        #     distance_f.write("cnt: " + str(cnt) + " / x_avg = " + str(x_avg) + "\n")

                        #     distance = float(depth_pix[240][x_avg]) / 1000
                        #     if distance != 0.0:
                        #         distance_f.write("distance " + str(distance) + "m\n\n")
                        #         distance_f.flush()

                        #         box_dist = round(distance, 2)
                        #         dist_text = str(box_dist) + 'm'
                        #         cv2.putText(annotated_frame, "idx: " + str(box_idx), (box_mid_x + 10, box_mid_y - 30), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 1)
                        #         cv2.putText(annotated_frame, dist_text, (box_mid_x, box_mid_y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
                        #endregion 계산 4

                        #region 계산 5
                        temp_depths = []
                        now2 = datetime.datetime.now().strftime("%H-%M-%S")
                        # distance_f.write(now2 + ":\n")
                        empty_node.get_logger().info("Time: {}".format(now2))
                        for idx, data in enumerate(keypoint_datas):
                            depth_val = depth_pix[int(data[1])][int(data[0])]
                            # distance_f.write(str(idx) + "\t- Keypoint: [" + str(data[0]) + ", " + str(data[1]) + "]")
                            # distance_f.write("\t" + str(depth_val / 1000) + "m\n")
                            if depth_val != 0:
                                dist = depth_val / 1000
                                temp_depths.append(dist)
                                
                        temp_depths.sort()
                        # empty_node.get_logger().info("Sorted depths: {}".format(temp_depths))
                        dist_data_size = len(temp_depths)
                        # for d in temp_depths:
                        #     distance_f.write(str(d) + " ")
                        # distance_f.write("\n")

                        #region depth avg
                        # sum = 0
                        # if dist_data_size < 4:
                        #     distance_f.write("Number of Depth points: " + str(dist_data_size) + " - Not enough keypoints!\n")
                        #endregion depth avg

                        #region gemini created code
                        # Calculate the median and interquartile range (IQR)
                        median = temp_depths[dist_data_size // 2]
                        q1 = temp_depths[dist_data_size // 4]
                        q3 = temp_depths[3 * dist_data_size // 4]
                        iqr = q3 - q1

                        # Define upper and lower bounds for outliers
                        upper_bound = q3 + 1.5 * iqr
                        lower_bound = q1 - 1.5 * iqr
                        filtered_data = [x for x in temp_depths if lower_bound <= x <= upper_bound]
                        dist_avg = round(sum(filtered_data) / len(filtered_data), 3)
                        # distance_f.write(": " + str(dist_avg) + "m\n")
                        empty_node.get_logger().info("dist_avg: {}m\n".format(dist_avg))
                        cv2.putText(annotated_frame, str(dist_avg)+'m', (box_mid_x, box_mid_y - 15), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
                        #endregion gemini created code

                    
                        # distance_f.write("\n")
                        #endregion 계산 5
                    
                        detect_node.detect_pub.publish(detect_msg)
                
                cvtColor_img_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
                detect_node._infer_pub.publish(bridge.cv2_to_imgmsg(cvtColor_img_frame, encoding=img_node.img_encoding))
                detect_node._depth_pub.publish(bridge.cv2_to_imgmsg(depth_frame, encoding=img_node.img_encoding))

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
                
                #region test
                # now2 = datetime.datetime.now().strftime("%H-%M-%S")
                # idx1 = [200, 120]
                # idx2 = [320, 120]
                # idx3 = [440, 120]
                # idx4 = [200, 240]
                # idx5 = [320, 240]
                # idx6 = [440, 240]
                # idx7 = [200, 360]
                # idx8 = [320, 360]
                # idx9 = [440, 360]
                # idx_list = [idx1, idx2, idx3, idx4, idx5, idx6, idx7, idx8, idx9]

                # for i in range(9):
                #     dist = str(depth_pix[idx_list[i][1]][idx_list[i][0]] / 1000)
                #     distance_f.write(now2 + " - idx " + str(i) + ": " + dist + "m\n")
                #     cv2.putText(frame, str(i), (idx_list[i][0], idx_list[i][1]), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
                # distance_f.write("\n")
                # distance_f.flush()
                #endregion test

                if mode == 0:
                    continue
                elif mode == 1:
                    # cv2.imshow("YOLOv8 Inference", annotated_frame)
                    # cv2.imshow("Depth Frame", depth_frame)
                    pass
                    # out.write(annotated_frame)

            except IndexError:
                print('array error')

                if mode == 0:
                    continue
                elif mode == 1:
                    # cv2.imshow("YOLOv8 Inference", annotated_frame)
                    pass
                    # out.write(annotated_frame)   

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break   

        else:
            break

    end_time = time.time()
    fps = total_frames / (start_time - end_time)
    PRINT_STRING = f'total_frames = {total_frames},  avg FPS = {fps:.2f}'
    empty_node.get_logger().info(PRINT_STRING)

    if cam_mode == 0:
        cap.release()
        # out.release()
    cv2.destroyAllWindows()
    if cam_mode == 1:
        img_node.destroy_node()
    detect_node.destroy_node()
    event_node.destroy_node()
    empty_node.destroy_node()
    img_node.destroy_node()
    rclpy.shutdown()    

if  __name__ == '__main__':
    main()