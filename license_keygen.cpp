#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <curl/curl.h>
#include <string>
#include <thread>

#include "ultimateALPR-SDK-API-PUBLIC.h"
#include "base64.h"

using namespace ultimateAlprSdk;
using json = nlohmann::json;

int main() {
    
    // Initialize ALPR engine
    std::string jsonConfig = R"({
        "assets_folder": ")" + std::string(PROJECT_DIR) + R"(/ultimateALPR/assets",
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
        "debug_level": "fatal"
    })";
    

    UltAlprSdkResult result = UltAlprSdkEngine::init(jsonConfig.c_str());
    if (!result.isOK()) {
        std::cerr << "Failed to initialize ALPR engine: " << result.phrase() << std::endl;
        return -1;
    }
    std::cout << "ALPR engine initialized successfully!" << std::endl;



    // Request runtime license key
	result = UltAlprSdkEngine::requestRuntimeLicenseKey(true);
	if (result.isOK()) {
		ULTALPR_SDK_PRINT_INFO("\n\n%s\n\n",
			result.json()
		);

        // Open the file for writing
        std::ofstream outFile("license_key.txt", std::ios::trunc);
        if (!outFile.is_open()) {
            std::cerr << "Error: Could not open file for writing." << std::endl;
            return 1;
        }

        // Write the string to the file
        outFile << result.json();

        // Close the file
        outFile.close();
	}
	else {
		ULTALPR_SDK_PRINT_ERROR("\n\n*** Failed: code -> %d, phrase -> %s ***\n\n",
			result.code(),
			result.phrase()
		);
	}

    UltAlprSdkEngine::deInit();
    std::cout << "ALPR engine deinitialized." << std::endl;

    return 0;
}
