# 2018 vision narrow drive

## Dependency
이 패키지는 lane_detector패키지에 있는 코드와 vision_narrow_drive에 있는 코드를 사용하므로, 두 패키지 모두 ros workspace안에 있어야함

## About main codes(classes)

## About launch files
* opencv_team_gtcam: prosilica gt 드라이버 노드 + crosswalk_stop 노드 + serial_example 노드(rc car) or kuuve_control 노드(scale platform)
* opencv_team_webcam: usb_cam 드라이버 노드 + crosswalk_stop 노드 + serial_example 노드(rc car) or kuuve_control 노드(scale platform)
* test_with_gtcam: prosilica gt 드라이버 노드 + crosswalk_stop 노드
* test_with_webcam: usb_cam 드라이버 노드 + crosswalk_stop 노드

## Debug mode printlog parsing
* debug mode로 실행시 print log들이 publish된다. 이때 `rostopic echo /lane_detector/printlog | sed 's/\\n/\n/g'`을 사용하면 로그를 실시간으로 볼수있다
