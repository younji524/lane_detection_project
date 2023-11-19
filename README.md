lane_detection_project
====

차선인식 프로젝트 (programmers devcourse)
===
> **프로그래머스 K-Digital Training 자율주행 데브코스**  
> Xycar를 활용한 차선인식 프로젝트 (Lane keeping project using Xycar)  
> 개발기간 (Development period): 2023.10.28 ~ 2023.11.17

## 팀 소개
| 김나혜 | 허동욱 |
|------|------|
|[@nahye03](https://github.com/nahye03)|[@dongwookheo](https://github.com/dongwookheo)|

## 프로젝트 소개
![track](https://github.com/dongwookheo/lane_detection_project/assets/124948998/5bf6f9fd-c2fb-48ec-b703-914d3b91bf98)
- 카메라 센서로 차선을 인식하여, Xycar가 주어진 코스를 완주할 수 있도록 한다  
- 블록을 넘어뜨릴 경우, 차선을 이탈할 경우 감점

## 주요 기능


## 환경 설정
### Requirements
- OpenCV 4.5.5
### Installation
```
git clone https://github.com/dongwookheo/lane_detection_project.git
```
```
cd lane_detection_project/
```
```
mkdir thirdparty && cd thirdparty
```
```
mkdir OpenCV && cd OpenCV
```
```
mkdir build install
```
```
git clone https://github.com/opencv/opencv.git
```
```
cd opencv
```
```
git checkout 4.5.5
```
```
cd ../build
```
```
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install ../opencv
```
- You can check the number of cores through the `nproc` command.
```
make -j<core_num>
```
```
make install
```

## Stacks
### Environment
<img src="https://img.shields.io/badge/visualstudiocode-007ACC?style=for-the-badge&logo=visualstudiocode&logoColor=white"> <img src="https://img.shields.io/badge/clion-000000?style=for-the-badge&logo=clion&logoColor=white">
<img src="https://img.shields.io/badge/cmake-064F8C?style=for-the-badge&logo=cmake&logoColor=white"> <img src="https://img.shields.io/badge/git-F04032?style=for-the-badge&logo=git&logoColor=white"> <img src="https://img.shields.io/badge/github-181717?style=for-the-badge&logo=github&logoColor=white">

### Config
<img src="https://img.shields.io/badge/yaml-CB171E?style=for-the-badge&logo=yaml&logoColor=white">

### Development
<img src="https://img.shields.io/badge/cplusplus-00599C?style=for-the-badge&logo=cplusplus&logoColor=white"> <img src="https://img.shields.io/badge/ros-22314E?style=for-the-badge&logo=ros&logoColor=white"> 
<img src="https://img.shields.io/badge/opencv-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white">

### Communication
<img src="https://img.shields.io/badge/slack-4A154B?style=for-the-badge&logo=slack&logoColor=white"> <img src="https://img.shields.io/badge/notion-000000?style=for-the-badge&logo=notion&logoColor=white">
<img src="https://img.shields.io/badge/jira-0052CC?style=for-the-badge&logo=jira&logoColor=white">

