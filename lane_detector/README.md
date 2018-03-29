# 2018 lane detector using opencv only

## About main codes(classes)
* LinePointDetector: 차선 검출을 위해 차선 위의 점을 찾는 클래스
* LaneDetector: LinePointDetector를 이용하여 차선을 검출하는 클래스
* LaneDetectorNode: LaneDetector를 이용한 ROS 노드 클래스
* main: LaneDetectorNode 클래스 객체를 생성하는 메인 함수 
Dependency: LinePointDetector <- LaneDetector <- LaneDetectorNode <- main
