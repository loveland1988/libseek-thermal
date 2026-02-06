/*
 * Persistent snapshot loop (headless-friendly)
 * Writes output.png repeatedly without reopening the camera.
 */
#include <opencv2/imgproc/imgproc.hpp>
#include "seek.h"
#include <iostream>
#include <cstdio>
#include <thread>
#include <chrono>
#include "args.h"

using namespace cv;
using namespace LibSeek;

static void rotate_mat(Mat& m, int rotate)
{
    if (rotate == 90) {
        transpose(m, m);
        flip(m, m, 1);
    } else if (rotate == 180) {
        flip(m, m, -1);
    } else if (rotate == 270) {
        transpose(m, m);
        flip(m, m, 0);
    }
}

int main(int argc, char** argv)
{
    cv::Mat frame_u16, frame_f32, avg_frame;

    LibSeek::SeekThermalPro seekpro;
    LibSeek::SeekThermal seek;
    LibSeek::SeekCam* cam;

    args::ArgumentParser parser("Capture frames continuously and overwrite output file");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag<std::string> _output(parser, "outfile",
        "Name of the file to write to - default /dev/shm/bravo_thermal/output.png",
        {'o', "outfile"});
    args::ValueFlag<int> _smoothing(parser, "smoothing",
        "Smoothing factor, number of frames to collect and average - default 1",
        {'s', "smoothing"});
    args::ValueFlag<std::string> _camtype(parser, "camtype",
        "Seek Thermal Camera Model - seek or seekpro",
        {'t', "camtype"});
    args::ValueFlag<int> _warmup(parser, "warmup",
        "Warmup frames to discard - default 10",
        {'w', "warmup"});
    args::ValueFlag<int> _rotate(parser, "rotate",
        "Rotation - 0, 90, 180 or 270 (default) degrees",
        {'r', "rotate"});
    args::ValueFlag<int> _interval_ms(parser, "interval-ms",
        "Delay between writes in milliseconds - default 150 (~6-7 FPS)",
        {"interval-ms"});

    try {
        parser.ParseCLI(argc, argv);
    }
    catch (args::Help) {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }
    catch (args::ValidationError e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    int smoothing = _smoothing ? args::get(_smoothing) : 1;
    if (smoothing < 1) smoothing = 1;

    int warmup = _warmup ? args::get(_warmup) : 10;
    if (warmup < 0) warmup = 0;

    std::string outfile = _output ? args::get(_output) : "/dev/shm/bravo_thermal/output.png";
    std::string tmpfile = outfile + ".tmp";

    std::string camtype = _camtype ? args::get(_camtype) : "seek";

    int rotate = _rotate ? args::get(_rotate) : 270;
    int interval_ms = _interval_ms ? args::get(_interval_ms) : 150;
    if (interval_ms < 0) interval_ms = 0;

    if (camtype == "seekpro") cam = &seekpro;
    else cam = &seek;

    if (!cam->open()) {
        std::cout << "failed to open " << camtype << " cam" << std::endl;
        return 2;
    }

    for (int i = 0; i < warmup; i++) {
        if (!cam->grab()) {
            std::cout << "no more LWIR img during warmup" << std::endl;
            return 3;
        }
        cam->retrieve(frame_u16);
    }

    std::cout << "warmup complete, streaming..." << std::endl;

    while (true) {
        // Keep the camera in a steady streaming cadence to avoid "idle->FFC->frame" behavior.
        // Grab frequently, but only write output.png at a lower rate.
        const int grab_ms = 66;   // ~15 Hz internal grab cadence
        const int write_ms = interval_ms > 0 ? interval_ms : 250; // reuse interval_ms as write cadence
        static auto last_write = std::chrono::steady_clock::now();

        if (!cam->grab()) {
            std::cout << "no more LWIR img (grab failed)" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        auto since = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_write).count();
        if (since >= write_ms) {
            last_write = now;

            // Retrieve one frame and write it
            cam->retrieve(frame_u16);

            Mat frame_g8;
            frame_u16.convertTo(frame_g8, CV_8UC1, 1.0 / 256.0);

            rotate_mat(frame_g8, rotate);

            (void)cv::imwrite(outfile, frame_g8);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(grab_ms));
    }

    return 0;
}
