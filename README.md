### safety cost を含めたfootstep planning を試す用の`.rosinstall`

```
- git:
    local-name: hrpsys
    uri: https://github.com/fkanehiro/hrpsys-base.git
- git:
    local-name: openhrp3
    uri: https://github.com/fkanehiro/openhrp3.git
- git:
    local-name: rtm-ros-robotics/rtmros_choreonoid
    uri: https://github.com/start-jsk/rtmros_choreonoid.git
- git:
    local-name: rtm-ros-robotics/rtmros_common
    uri: https://github.com/yuki-shark/rtmros_common.git
    version: fix_comp
- git:
    local-name: rtm-ros-robotics/rtmros_hrp2
    uri: https://github.com/start-jsk/rtmros_hrp2.git
- git:
    local-name: rtm-ros-robotics/rtmros_tutorials
    uri: https://github.com/start-jsk/rtmros_tutorials.git
- git:
    local-name: trans_system
    uri: https://github.com/jsk-ros-pkg/trans_system.git
- git:
    local-name: jsk_recognition
    uri: https://github.com/yuki-shark/jsk_recognition.git
    version: pointcloud_xyzi_to_xyz
- git:
    local-name: safe_footstep_planner
    uri: https://github.com/yuki-shark/safe_footstep_planner.git
- git:
    local-name: viso2
    uri: https://github.com/srv/viso2.git
- git:
    local-name: jsk_control
    uri: https://github.com/yuki-shark/jsk_control.git
    version: chidori_footstep_planning
- hg:
    local-name: multisense
    uri: https://bitbucket.org/crl/multisense_ros
    version: 3.4.9
```
