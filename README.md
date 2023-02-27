# Rubber_Cone
라바콘 주행할 때 필요한 패키지들이다.
rplidar_ros : RPLiDAR A2를 사용할 때 필요
lidar_race : 객체를 인식하여 조향각을 계산한 후 topic을 publish
cmd_erp : noise로 인해 조향각이 틔는 것을 방지해주며 초반에 출발할 때 많은 구동력이 필요하기 때문에 약 1초 동안 속도를 높인다.
erp_race : Custom Msg에 담은 조향각 및 속도를 통해 ERP-42와 연결하여 주행.
**주의 : 처음 이 패키지들을 git clone하여 catkin_make를 하면 header파일(.h)에 의해 오류 발생할 가능성이 크다.
        이건 알아서 고쳐서 할 것.
