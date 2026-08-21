# Multi-AMR Digital Twin

혼류 생산 환경에서 Multi-AMR 기반 부품 공급을 구현하는
ROS 2 / Isaac Sim / NVIDIA cuOpt Digital Twin 프로젝트입니다.

## Project Goal

Assembly Cell에서 발생하는 부품 공급 작업을 FMS가 관리하고,
NVIDIA cuOpt를 이용해 작업을 최적화한 뒤 Nav2와 AMR을 통해
부품을 각 Assembly Cell까지 배송하는 Digital Twin 시스템을 구현합니다.

## System

- NVIDIA Isaac Sim
- ROS 2 Jazzy
- Nav2
- NVIDIA cuOpt
- FMS
- Assembly Cell
- AMR

## Scenario 0 - Normal Parts Supply

Assembly Cell에서 부품 Kit 요청  
→ FMS Task 생성  
→ cuOpt 최적화  
→ AMR Task 할당  
→ Nav2 경로 계획  
→ AMR Supermarket 이동  
→ Kit 적재  
→ Assembly Cell 이동  
→ Kit 배송  
→ Assembly Cell 작업 수행  

## PC Architecture

### PC_1 - Simulation

- Isaac Sim
- Factory World
- AMR Asset
- AMR Physics
- Sensors
- ROS 2 Bridge

### PC_2 - Control / Optimization

- FMS
- Assembly Cell Logic
- NVIDIA cuOpt
- Nav2
- AMR Control Node

## Development Environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Isaac Sim 5.1
- NVIDIA GPU
