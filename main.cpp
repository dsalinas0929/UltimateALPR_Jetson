#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <climits>
#include <curl/curl.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <thread>
#include <regex>
#include <cctype>
#include <exception>

#include "ultimateALPR-SDK-API-PUBLIC.h"
#include "base64.h"

using namespace ultimateAlprSdk;
using json = nlohmann::json;

struct PlateDetection
{
    bool isValid;
    json plateJson;
    cv::Mat carImg;
    cv::Mat plateImg;
    float confidence;
    std::chrono::steady_clock::time_point timestamp; // analysis period start, it is the time of the first detection
    std::chrono::steady_clock::time_point sendTime;  // when to send webhook
    std::chrono::steady_clock::time_point lastSent;  // track when webhook was last sent
};

// Convert cv::Mat to Base64 string
std::string matToBase64(const cv::Mat &img)
{
    std::vector<uchar> buf;
    cv::imencode(".jpg", img, buf);
    auto *enc_msg = reinterpret_cast<const unsigned char *>(buf.data());
    std::string encoded = base64_encode(enc_msg, buf.size());
    return encoded;
}

// Create webhook payload
json createWebhookPayload(const json &plateJson,
                          const cv::Mat &fullCarImg,
                          const cv::Mat &plateImg,
                          const std::string &static1,
                          const std::string &static2)
{
    json payload;

    // Plate info
    payload["plate_number"] = plateJson.value("text", "N/A");

    // // Full confidences array
    // if (plateJson.contains("confidences")) {
    //     payload["confidences"] = plateJson["confidences"];
    // }

    // Get total confidence value
    float sum = 0.0;
    int count = plateJson["confidences"].size();
    for (auto &val : plateJson["confidences"])
    {
        sum += val.get<float>();
    }
    float average = sum / count;
    payload["confidence"] = average;

    // Vehicle info
    if (plateJson.contains("car"))
    {
        payload["car"] = plateJson["car"];
    }

    // Country info
    if (plateJson.contains("country"))
    {
        payload["country"] = plateJson["country"];
    }

    // Images as Base64
    payload["car_image_base64"] = matToBase64(fullCarImg);
    payload["plate_image_base64"] = matToBase64(plateImg);

    // Timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%SZ");
    payload["timestamp"] = ss.str();

    // Static details from config
    payload["static_detail_1"] = static1;
    payload["static_detail_2"] = static2;

    // std::cout << "===== Payload JSON: " << average << std::endl;

    return payload;
}

// send webhook
bool sendWebhook(const std::string &url, const json &payload)
{
    CURL *curl = curl_easy_init();
    if (!curl)
    {
        std::cerr << "Failed to initialize curl" << std::endl;
        return false;
    }

    std::string jsonStr = payload.dump(); // Convert JSON to string

    // Set curl options
    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    // Set URL and payload
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonStr.c_str());

    CURLcode res = curl_easy_perform(curl);

    // Cleanup
    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);

    if (res != CURLE_OK)
    {
        std::cerr << "Webhook POST failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    return true;
}

// Validate plate text with regex and length
bool isValidPlate(const std::string &text)
{
    // Basic regex: only uppercase letters and digits
    if (text.empty())
        return false;
    std::regex pattern("^[A-Z0-9]+$"); // only letters and digits
    if (!std::regex_match(text, pattern))
        return false;
    if (text.length() < 6 || text.length() > 8)
        return false; // filter short or too long misreads
    return true;
};

// process a frame and send webhook for each detected plate
void sendWebhookWithFrame(const json plate, cv::Mat &carImg, cv::Mat &plateImg, const std::string &webhookUrl, const std::string &static1, const std::string &static2)
{
    json payload = createWebhookPayload(plate, carImg, plateImg, static1, static2);
    float conf = payload["confidence"];
    std::string timestamp = payload["timestamp"];

    if (!sendWebhook(webhookUrl, payload))
    {
        // std::cerr << "Failed to send webhook for plate: " << plate.value("text", "N/A") << std::endl;
    }
    else
    {
        std::cout << std::endl
                  << std::endl
                  << "   ***** Webhook sent for plate: " << plate.value("text", "N/A") << " conf: " << conf << " from: " << static2 << " timestamp: " << timestamp << std::endl
                  << std::endl
                  << std::endl;
    }

    // ===== Save payload to file =====
    try
    {
        std::string plateText = plate.value("text", "N/A");
        if (plateText.empty())
            plateText = "unknown";

        // sanitize filename (remove invalid characters)
        for (auto &ch : plateText)
        {
            if (!std::isalnum(ch) && ch != '-' && ch != '_')
                ch = '_';
        }

        std::string filename = plateText + "_" + static2 + ".txt";
        std::ofstream outFile(filename);
        if (outFile.is_open())
        {
            outFile << payload.dump(4); // pretty-print JSON with 4-space indentation
            outFile.close();
            std::cout << "Saved payload to file: " << filename << std::endl;
        }
        else
        {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error saving payload to file: " << e.what() << std::endl;
    }
}

// --- Stream Processor Class ---
class StreamProcessor
{
public:
    StreamProcessor(const nlohmann::json &cfg, bool gui) : useGui(gui)
    {
        videoPath = cfg.value("video_path", "");
        webhookUrl = cfg.value("webhook_url", "");
        static1 = cfg.value("static_detail_1", "");
        static2 = cfg.value("static_detail_2", "");
        minConfidenceThreshold = cfg.value("min_confidence", 70.0f);
        repetitionCooldown = cfg.value("plate_repetition_cooldown", 20);
        analysisPeriod = cfg.value("plate_analysis_period", 5);

        if (cfg.contains("roi"))
        {
            roi.x = cfg["roi"].value("x", 0);
            roi.y = cfg["roi"].value("y", 0);
            roi.width = cfg["roi"].value("width", 400);
            roi.height = cfg["roi"].value("height", 400);
        }
        else
        {
            roi = cv::Rect(0, 0, 400, 400); // default
        }

        // log out config
        std::cout << "Stream Config:" << std::endl;
        std::cout << " Video Path: " << videoPath << std::endl;
        std::cout << " Webhook URL: " << webhookUrl << std::endl;
        std::cout << " Static Detail 1: " << static1 << std::endl;
        std::cout << " Static Detail 2: " << static2 << std::endl;
        std::cout << " Min Confidence Threshold: " << minConfidenceThreshold << std::endl;
        std::cout << " Plate Repetition Cooldown: " << repetitionCooldown << " seconds" << std::endl;
        std::cout << " Plate Analysis Period: " << analysisPeriod << " seconds" << std::endl;
        // std::cout << " ROI: x=" << roi.x << " y=" << roi.y << " width=" << roi.width << " height="  << roi.height << std::endl;
    }

    void run()
    {
        // Open video file
        cv::VideoCapture cap(videoPath, cv::CAP_FFMPEG);
        if (!cap.isOpened())
            return;

        cv::Mat frame;
        cv::Mat roiFrame; // frame cropped to ROI
        cv::Mat grayPrev; // store previous ROI gray frame for motion detection
        int frameCount = 0;

        while (true)
        {
            // Try to (re)connect
            if (!cap.isOpened())
            {
                std::cout << "Trying to connect to RTSP stream: " << videoPath << std::endl;
                cap.open(videoPath, cv::CAP_FFMPEG);

                if (!cap.isOpened())
                {
                    std::cerr << "Failed to connect. Retrying in 5 seconds... : " << videoPath << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    continue; // try again
                }
                std::cout << "Connected to RTSP stream!" << videoPath << std::endl;
            }
            if (!cap.read(frame) || frame.empty())
            {
                std::cerr << "Lost connection to RTSP stream. Reconnecting... : " << videoPath << std::endl;
                cap.release();
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue; // retry
            }

            // Validate ROI against frame size
            roi.x = std::max(0, roi.x);
            roi.y = std::max(0, roi.y);
            roi.width = std::min(roi.width, frame.cols - roi.x);
            roi.height = std::min(roi.height, frame.rows - roi.y);

            if (roi.width <= 0 || roi.height <= 0)
            {
                // std::cerr << "Invalid ROI after clamping!" << std::endl;
                continue; // skip this frame safely
            }
            roiFrame = frame(roi).clone();

            if (frame.empty())
            {
                // Restart video from beginning
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }
            auto now = std::chrono::steady_clock::now();

            frameCount++;
            // std::cout << "- Processing frame " << frameCount << std::endl;

            // Motion detection in ROI
            cv::Mat gray, diff;
            cv::cvtColor(frame(roi), gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

            bool motionDetected = true;
            if (!grayPrev.empty())
            {
                cv::absdiff(grayPrev, gray, diff);
                cv::threshold(diff, diff, 25, 255, cv::THRESH_BINARY);
                int motionPixels = cv::countNonZero(diff);
                if (motionPixels < 500)
                { // threshold for minimal motion
                    motionDetected = false;
                }
            }
            grayPrev = gray.clone();

            if (motionDetected)
            {
                // Run ALPR
                UltAlprSdkResult result = UltAlprSdkEngine::process(
                    ULTALPR_SDK_IMAGE_TYPE_BGR24,
                    roiFrame.ptr(),
                    roiFrame.cols,
                    roiFrame.rows);

                if (result.isOK())
                {
                    std::string resJson = result.json();
                    try
                    {
                        auto parsed = json::parse(resJson);

                        if (parsed.contains("plates") && parsed["plates"].is_array())
                        {
                            for (auto &plate : parsed["plates"])
                            {
                                if (!plate.contains("warpedBox") || !plate["warpedBox"].is_array())
                                    continue;

                                std::string plateText = plate.value("text", "N/A");

                                float sum = 0.0;
                                int count = plate["confidences"].size();
                                for (auto &val : plate["confidences"])
                                {
                                    sum += val.get<float>();
                                }
                                float totalConfidence = 0.0f;
                                if (count > 0)
                                    totalConfidence = sum / count;

                                if (totalConfidence < minConfidenceThreshold)
                                {
                                    // std::cerr << "*** confidence is too low: " << totalConfidence << " , so continue" <<  std::endl;
                                    continue; // skip low-confidence plates
                                }
                                // Dump full plate info for debugging
                                // std::cout << "Plate JSON: " << plate.dump(4) << std::endl;

                                // *** ADDED: Misrecognition filtering by regex + length ***

                                if (!isValidPlate(plateText))
                                {
                                    std::cout << "Ignored misrecognized plate: " << static2 << "  " << plateText << std::endl; // *** ADDED ***
                                    continue;                                                                                  // skip this detection
                                }

                                // --- Crop plate image ---
                                auto &plateBox = plate["warpedBox"];
                                float px_min = plateBox[0].get<float>();
                                float py_min = plateBox[1].get<float>();
                                float px_max = plateBox[0].get<float>();
                                float py_max = plateBox[1].get<float>();

                                for (int i = 0; i < 8; i += 2)
                                {
                                    float x = plateBox[i].get<float>();
                                    float y = plateBox[i + 1].get<float>();
                                    px_min = std::min(px_min, x);
                                    py_min = std::min(py_min, y);
                                    px_max = std::max(px_max, x);
                                    py_max = std::max(py_max, y);
                                }

                                cv::Rect plateRect(
                                    static_cast<int>(std::round(px_min)) + roi.x,
                                    static_cast<int>(std::round(py_min)) + roi.y,
                                    static_cast<int>(std::round(px_max - px_min)),
                                    static_cast<int>(std::round(py_max - py_min)));
                                cv::Mat plateCrop = frame(plateRect).clone();

                                // --- Crop car image ---
                                auto &carBox = plate["car"]["warpedBox"];
                                float cx_min = carBox[0].get<float>();
                                float cy_min = carBox[1].get<float>();
                                float cx_max = carBox[0].get<float>();
                                float cy_max = carBox[1].get<float>();

                                for (int i = 0; i < 8; i += 2)
                                {
                                    float x = carBox[i].get<float>();
                                    float y = carBox[i + 1].get<float>();
                                    cx_min = std::min(cx_min, x);
                                    cy_min = std::min(cy_min, y);
                                    cx_max = std::max(cx_max, x);
                                    cy_max = std::max(cy_max, y);
                                }

                                cv::Rect carRect(
                                    static_cast<int>(std::round(cx_min)) + roi.x,
                                    static_cast<int>(std::round(cy_min)) + roi.y,
                                    static_cast<int>(std::round(cx_max - cx_min)),
                                    static_cast<int>(std::round(cy_max - cy_min)));
                                cv::Mat carCrop = frame(carRect).clone();

                                // --- send webhook ---
                                if (!plateBuffer.isValid)
                                {
                                    plateBuffer = {
                                        true,
                                        plate,
                                        carCrop.clone(),
                                        plateCrop.clone(),
                                        totalConfidence,
                                        plateBuffer.lastSent, // keep lastSent unchanged
                                        now + std::chrono::seconds(analysisPeriod),
                                        std::chrono::steady_clock::time_point{} // lastSent = not set yet
                                    };
                                }
                                else
                                {
                                    // update only if this detection has higher confidence
                                    if (totalConfidence > plateBuffer.confidence)
                                    {
                                        plateBuffer.plateJson = plate;
                                        plateBuffer.carImg = carCrop.clone();
                                        plateBuffer.plateImg = plateCrop.clone();
                                        plateBuffer.confidence = totalConfidence;
                                        std::cout << "Updating plateBuffer with " << plateText << " " << static2 << " " << totalConfidence << std::endl;
                                    }
                                    else
                                    {
                                        std::cout << "Not updating " << plateText << " " << static2 << " conf " << totalConfidence << " is less than existing " << plateBuffer.confidence << std::endl;
                                    }
                                }

                                // draw rectangles
                                if (useGui)
                                {
                                    cv::rectangle(frame, plateRect, cv::Scalar(0, 255, 0), 2);
                                    cv::rectangle(frame, carRect, cv::Scalar(255, 0, 0), 2);
                                    int rectX = std::max(plateRect.x, 0);
                                    int rectY = std::max(plateRect.y, 0);

                                    // draw plate text
                                    cv::putText(frame, plateText, cv::Point(rectX, rectY - 5),
                                                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                                }
                            }
                        }
                    }
                    catch (std::exception &e)
                    {
                        std::cerr << "JSON parse error: " << e.what() << std::endl;
                    }
                }

                // Send webhook when timer expires

                if (now >= plateBuffer.sendTime && plateBuffer.isValid)
                {
                    // Check cooldown before sending
                    if (plateBuffer.lastSent.time_since_epoch().count() == 0 ||
                        std::chrono::duration_cast<std::chrono::seconds>(now - plateBuffer.lastSent).count() >= repetitionCooldown)
                    {
                        sendWebhookWithFrame(
                            plateBuffer.plateJson,
                            plateBuffer.carImg,
                            plateBuffer.plateImg,
                            webhookUrl,
                            static1,
                            static2);
                        // std::cout << "Webhook sent for plate: " << plateText << " " << static2 << "confidence: " <<  << std::endl;

                        // Update lastSent timestamp
                        plateBuffer.lastSent = now;
                    }
                    // After sending (or cooldown check), erase
                    plateBuffer.isValid = false;
                }
            }
            if (useGui)
            {
                // Draw ROI rectangle for debugging
                cv::rectangle(frame, roi, cv::Scalar(0, 0, 255), 1);

                // Show video with overlays
                cv::imshow("RTSP-ALPR", frame);
            }

            int key = cv::waitKey(1);
            if (key == 27)
                break; // ESC to exit
        }
        cap.release();
        if (useGui)
        {
            cv::destroyAllWindows();
        }
    }

private:
    cv::Rect roi;
    std::string videoPath;
    std::string webhookUrl;
    std::string static1;
    std::string static2;
    float minConfidenceThreshold;
    int repetitionCooldown;
    int analysisPeriod;
    bool useGui = false;

    PlateDetection plateBuffer;
};

int main(int argc, char *argv[])
{
    bool useGui = false;
    std::unordered_map<std::string, std::string> args;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg.rfind("--", 0) == 0)
        {
            std::string key = arg;
            std::string value = "true"; // default for flags
            if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0)
            {
                value = argv[i + 1];
                i++;
            }
            args[key] = value;
        }
    }
    if (args.find("--gui") != args.end())
    {
        useGui = true;
    }
    std::cout << "Use GUI: " << (useGui ? "true" : "false") << std::endl
              << std::endl;

    // Load app config
    json appConfig;
    std::ifstream configFile(std::string(PROJECT_DIR) + "/config.json");
    if (!configFile.is_open())
    {
        std::cerr << "Failed to open config.json" << std::endl;
        return -1;
    }
    configFile >> appConfig;
    configFile.close();

    std::string jsonConfig;
    auto licenseToken = appConfig.value("license_token_data", "");

    if (licenseToken.empty())
    {
        jsonConfig = R"({
            "assets_folder": ")" +
                     std::string(PROJECT_DIR) + R"(/ultimateALPR/assets",
            "charset": "LATIN",
            "recogn_rectify_enabled": true,
            "car_noplate_detect_enabled": false,
            "ienv_enabled": false,
            "gpgpu_enabled": true,
            "recogn_gpu_backend": "TensorRT",
            "openvino_enabled": false,
            "openvino_device": "GPU",
            "recogn_quantization_enabled": true,
            "recogn_score_type": "min",
            "recogn_minscore": 0.3,
            "detect_minscore": 0.1,
            "npu_enabled": false,
            "trt_enabled": true,
            "klass_lpci_enabled": true,
            "klass_vcr_enabled": true,
            "klass_vmmr_enabled": true,
            "klass_vbsr_enabled": true,
            "debug_level": "error"
        })";
    }
    else
    {
        jsonConfig = R"({
            "assets_folder": ")" +
                     std::string(PROJECT_DIR) + R"(/ultimateALPR/assets",
            "charset": "LATIN",
            "recogn_rectify_enabled": true,
            "car_noplate_detect_enabled": false,
            "ienv_enabled": false,
            "gpgpu_enabled": true,
            "recogn_gpu_backend": "TensorRT",
            "openvino_enabled": false,
            "openvino_device": "GPU",
            "recogn_quantization_enabled": true,
            "recogn_score_type": "min",
            "recogn_minscore": 0.3,
            "detect_minscore": 0.1,
            "npu_enabled": false,
            "trt_enabled": true,
            "klass_lpci_enabled": true,
            "klass_vcr_enabled": true,
            "klass_vmmr_enabled": true,
            "klass_vbsr_enabled": true,
            "debug_level": "error",
            "license_token_data": ")" +
                     appConfig.value("license_token_data", "") + R"("
        })";
    }

    UltAlprSdkResult result = UltAlprSdkEngine::init(jsonConfig.c_str());
    if (!result.isOK())
    {
        std::cerr << "Failed to initialize ALPR engine: " << result.phrase() << std::endl;
        return -1;
    }
    std::cout << "ALPR engine initialized successfully!" << std::endl;

    // Launch one worker per stream
    std::vector<std::thread> workers;
    try
    {
        for (auto &streamCfg : appConfig["streams"])
        {
            workers.emplace_back([streamCfg, useGui]()
                                 {
                                     StreamProcessor processor(streamCfg, useGui);
                                     processor.run(); // inside run(): capture frames + call UltAlprSdkEngine::process(...)
                                 });
        }
    }
    catch (std::exception &e)
    {
        std::cerr << "Error starting streams: " << e.what() << std::endl;
    }

    // Join all workers
    for (auto &t : workers)
    {
        if (t.joinable())
            t.join();
    }

    // Clean up

    UltAlprSdkEngine::deInit();
    std::cout << "ALPR engine deinitialized." << std::endl;

    return 0;
}
