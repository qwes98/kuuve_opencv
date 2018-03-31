# 2018 crosswalk stop using opencv only

## Dependency
이 패키지는 lane_detector패키지에 있는 코드를 사용하므로, lane_detector패키지도 ros workspace안에 있어야함

## About main codes(classes)
* CrosswalkStop: LaneDetector 클래스를 확장한 클래스, 횡단보도 인식을 위한 코드 추가
* CrosswalkStopNode: CrosswalkStop를 이용한 ROS 노드 클래스
* main: CrossStopNode 클래스 객체를 생성하여 노드를 생성하는 메인 함수
* *Dependency: etc/Class Diagram.png 참고*

## About launch files
* opencv_team_gtcam: prosilica gt 드라이버 노드 + crosswalk_stop 노드 + serial_example 노드(rc car) or kuuve_control 노드(scale platform)
* opencv_team_webcam: usb_cam 드라이버 노드 + crosswalk_stop 노드 + serial_example 노드(rc car) or kuuve_control 노드(scale platform)
* test_with_gtcam: prosilica gt 드라이버 노드 + crosswalk_stop 노드
* test_with_webcam: usb_cam 드라이버 노드 + crosswalk_stop 노드
