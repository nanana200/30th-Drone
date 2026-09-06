# UAV GPS + LoRa firmware

STM32H753 기반 UAV용 GPS + SX127x LoRa 통신 실험 펌웨어입니다.

이 저장소는 [`songjulim/30th-Drone`의 GPS 브랜치](https://github.com/songjulim/30th-Drone/tree/GPS)를
기반으로 GPS/LoRa 패킷 통신을 개별적으로 수정한 작업본입니다. 원본 저장소를 직접 변경하지
않고 구현 내용과 테스트 결과를 별도로 관리하기 위해 독립 저장소로 공개합니다.

주요 변경 내용:

- GCS/UAV/UGV 주소를 구분하는 LoRa 패킷 구조
- GCS의 UAV POLL 요청에 대한 GPS 좌표 응답
- UAV waypoint 수신 및 ACK 전송
- 수신 CRC, 패킷 길이, 목적지 및 SPI/FIFO 진단 로그
- LoRa SPI4 속도 7.5 MHz 및 단일 CS 트랜잭션 기반 레지스터/FIFO 접근
- 송신 완료 후 RX Continuous 자동 복귀

하드웨어 연결, 패킷 형식 및 벤치 테스트 방법은
[README_GPS_LORA.md](README_GPS_LORA.md)를 참고하세요.

> 이 코드는 실험 및 통신 검증용입니다. 비행 제어와 waypoint 자율비행 실행은 포함하지 않습니다.
