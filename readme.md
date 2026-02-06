
# Jetson Nano ALPR Vehicle Detection

This project runs on **Jetson Nano** and monitors real-time RTSP streams (like IP cameras) to detect vehicles and their license plates using the **ultimateALPR SDK**. Detected data is analyzed and sent to a webhook server. The system supports region-of-interest (ROI) processing and motion detection for efficient monitoring.

---

## Features

- Real-time vehicle and license plate recognition via RTSP streams.
- Sends detailed vehicle and plate info to a webhook server.
- Analysis mechanism picks the most accurate detections within a time period.
- Only sends unique car detections to prevent duplicate webhook submissions.
- ROI-based detection with motion filtering: process only moving objects within a defined square.
- Base64 encoding for car and plate images in webhook payload.
- Configurable thresholds for confidence and detection timing.
- Multi-threaded support for multiple streams.

---

## Project Structure

```
project_root/
│
├─ CMakeLists.txt
├─ config.json            # Application configuration file
├─ ultimateALPR           # ultimateALPR SDK (Download from GitHub: https://github.com/DoubangoTelecom/ultimateALPR-SDK)
├─ main.cpp               # Main application code
├─ license_keygen.cpp
├─ build/                 # Build folder
├─ packages_required.txt  # Required dependencies
```

---

## Setup / Installation

### 1. Download ultimateALPR SDK
Clone or download the SDK from GitHub:

```bash
git clone https://github.com/DoubangoTelecom/ultimateALPR-SDK.git
```

### 2. Install Dependencies
Refer to `packages_required.txt` for all required packages and install them on Jetson Nano.

### 3. Build the Project
```bash
mkdir build
cd build
cmake ..
make
```

### 4. Configure the Application
Edit `config.json` to set:

- `video_path` – RTSP stream URL or video file.
- `webhook_url` – Endpoint to receive detection data.
- `roi` – Region of interest for detection (`x`, `y`, `width`, `height`).
- `min_confidence` – Minimum confidence threshold for plate detection.
- `plate_repetition_cooldown` – Seconds to wait before sending repeated plate detections.
- `plate_analysis_period` – Time window to analyze multiple detections for accuracy.
- `static_detail_1` and `static_detail_2` – Custom static info to include in webhook.

### 5. Run the Application
```bash
./ALPRApp --gui    # Optional flag to enable GUI visualization
```

- ESC to exit GUI.
- The app will reconnect automatically if RTSP stream is lost.

---

## How It Works

1. Captures frames from RTSP streams.
2. Crops the frame based on defined ROI.
3. Detects motion in the ROI; processes only if motion is detected.
4. Runs **ultimateALPR SDK** to detect vehicles and license plates.
5. Maintains an analysis buffer to select the most confident detection within a period.
6. Sends a **single webhook** with:

- Plate number
- Vehicle info
- Country
- Base64 car image
- Base64 plate image
- Timestamp
- Static details from config

7. Repeats for multiple streams using separate threads.

---

## Notes

- Ensure the license token for ultimateALPR is properly set in `config.json`.
- Adjust `min_confidence` and `roi` according to your camera setup for optimal performance.
- The project supports multiple RTSP streams; each stream runs in its own worker thread.

---

## Troubleshooting

- **Cannot connect to RTSP stream:** Ensure URL is correct and camera allows access. The app retries automatically every 5 seconds.
- **Low detection confidence:** Increase `min_confidence` or improve camera visibility/lighting.
- **Webhook not received:** Check network connectivity and validate `webhook_url`.

---

## License

This project uses **ultimateALPR SDK**, please follow the licensing terms from [Doubango Telecom GitHub](https://github.com/DoubangoTelecom/ultimateALPR-SDK).
