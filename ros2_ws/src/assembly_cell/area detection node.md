5.4 area_detection_node.py

AMR의 /amr/odom을 구독하여
Assembly Cell A/B/C 영역 진입을 판단한다.

/amr/odom
 ↓
Area Detection Node
 ↓
/assembly/part_arrived
 ↓
Assembly Node

동일 Area 안에서 이벤트가 반복 발행되지 않도록
현재 Area 상태를 유지한다.
