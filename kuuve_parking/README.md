# 2018 parking

## About main codes(classes)

## About launch files
* opencv_team_gtcam: prosilica gt 드라이버 노드 + lane_detector 노드 + serial_example 노드(rc car) or kuuve_control 노드(scale platform)
* opencv_team_webcam: usb_cam 드라이버 노드 + lane_detector 노드 + serial_example 노드(rc car) or kuuve_control 노드(scale platform)
* test_with_gtcam: prosilica gt 드라이버 노드 + lane_detector 노드
* test_with_webcam: usb_cam 드라이버 노드 + lane_detector 노드

## Debug mode printlog parsing
* debug mode로 실행시 print log들이 publish된다. 이때 `rostopic echo /lane_detector/printlog | sed 's/\\n/\n/g'`을 사용하면 로그를 실시간으로 볼수있다
