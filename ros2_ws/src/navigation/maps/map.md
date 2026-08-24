5.2 factory_map.yaml

Map Server가 factory_map.pgm을 읽기 위한 메타데이터 파일이다.

현재 프로젝트의 공장 World 크기와 Nav2 좌표계를 맞추기 위해 사용한다.

--------------------------------------------------------------------------------------

5.3 factory_map.pgm

Nav2 Global Costmap과 AMCL이 사용하는 실제 Occupancy Grid 이미지다.

기존에는 Isaac 내부 좌표만 사용해 직접 이동했기 때문에
별도의 2D Map이 필요하지 않았지만,
Nav2를 적용하면서 Map Server가 읽을 정적 지도가 필요해져 새로 생성했다.
