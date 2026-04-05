/*
 * main.cpp
 * Author: Karthik Govindan V
 * Date: 2026-04-05
 *
 * Entry point for the MediMark AR trolley tracking system. Supports
 * two run modes: 'demo' for live webcam detection with AR overlay,
 * and 'evaluate' for processing sample frames and outputting metrics.
 */

#include <iostream>
#include "aruco_detector.hpp"
#include "kalman_predictor.hpp"
#include "collision_checker.hpp"
#include "visualizer.hpp"

int main(int argc, char** argv) {
    // Full implementation will be added in Phase 2 and Phase 4
    std::cout << "MediMark AR v1.0.0" << std::endl;
    std::cout << "Usage: medimark_ar --mode [demo|evaluate]" << std::endl;
    return 0;
}
