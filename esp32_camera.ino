/* ESP32-CAM Electrical Device Detection with OLED Display and Web Interface
 * Based on Edge Impulse Arduino examples
 * Enhanced with OLED display and web streaming capabilities
 */

#include <ElecDevice_Detect_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// WiFi and Web Server
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// WiFi credentials - CHANGE THESE TO YOUR NETWORK
const char* ssid = "Your SSID";
const char* password = "Your Pass";

// Camera model selection
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#else
#error "Camera model not selected"
#endif

// Camera and inference constants
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

// OLED Display configuration
#define I2C_SDA 15
#define I2C_SCL 14
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Global objects
TwoWire I2Cbus = TwoWire(0);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);
WebServer server(80);

// State variables
static bool debug_nn = false;
static bool is_initialised = false;
static bool wifi_connected = false;
uint8_t *snapshot_buf;

// Latest detection results for web interface
struct DetectionResult {
  String device_name;
  float confidence;
  uint32_t x, y, width, height;
  unsigned long timestamp;
  bool valid;
};

DetectionResult latest_detection = {"", 0.0, 0, 0, 0, 0, 0, false};
String system_status = "Initializing...";

// Camera configuration
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1, // Use 1 frame buffer for inference, stream will use its own
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// Function declarations
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void setup_wifi();
void setup_web_server();
void handle_root();
void handle_api_status();
void handle_api_detection();
void handle_stream(); // <-- ADDED
void update_display(const DetectionResult& result);
void display_wifi_info();
void display_error(const char* error_msg);

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C for OLED
    I2Cbus.begin(I2C_SDA, I2C_SCL, 100000);
    
    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
        while (true) delay(1000);
    }
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ESP32-CAM Device");
    display.println("Detection System");
    display.println("Initializing...");
    display.display();
    
    // Wait for serial connection (optional)
    while (!Serial);
    Serial.println("ESP32-CAM Device Detection with Web Interface");
    
    // Initialize camera
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
        display_error("Camera Init Failed");
        while (true) delay(1000);
    } else {
        ei_printf("Camera initialized\r\n");
        system_status = "Camera Ready";
    }
    
    // Setup WiFi
    setup_wifi();
    
    // Setup web server
    setup_web_server();
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("System Ready");
    display.println("Starting detection...");
    display.display();
    
    system_status = "Running";
    ei_printf("\nStarting continuous inference...\n");
    delay(2000);
}

void loop() {
    // Handle web server requests
    server.handleClient();
    
    // Run inference
    if (ei_sleep(100) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        system_status = "Memory Error";
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        system_status = "Capture Error";
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        system_status = "Inference Error";
        free(snapshot_buf);
        return;
    }

    // Process results
    DetectionResult current_result = {"No device detected", 0.0, 0, 0, 0, 0, millis(), false};
    
    // Print timing info
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    bool device_found = false;
    
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        
        device_found = true;
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
        
        // Update current result with highest confidence detection
        if (bb.value > current_result.confidence) {
            current_result.device_name = String(bb.label);
            current_result.confidence = bb.value;
            current_result.x = bb.x;
            current_result.y = bb.y;
            current_result.width = bb.width;
            current_result.height = bb.height;
            current_result.valid = true;
        }
    }
    
    if (!device_found) {
        ei_printf("No objects found\n");
    }
    
#else
    // Classification mode
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: %.5f\r\n", ei_classifier_inferencing_categories[i], result.classification[i].value);
        
        // Update result with highest confidence classification
        if (result.classification[i].value > current_result.confidence) {
            current_result.device_name = String(ei_classifier_inferencing_categories[i]);
            current_result.confidence = result.classification[i].value;
            current_result.valid = true;
        }
    }
#endif

    // Update global detection result
    latest_detection = current_result;
    
    // Update OLED display
    update_display(current_result);
    
    // Update system status
    if (current_result.valid && current_result.confidence > 0.5) {
        system_status = "Detection: " + current_result.device_name;
    } else {
        system_status = "Scanning...";
    }

    free(snapshot_buf);
}

void setup_wifi() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting WiFi...");
    display.display();
    
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println();
        Serial.print("Connected to WiFi. IP address: ");
        Serial.println(WiFi.localIP());
        display_wifi_info();
    } else {
        wifi_connected = false;
        Serial.println("Failed to connect to WiFi");
        display_error("WiFi Failed");
    }
}

void setup_web_server() {
    if (!wifi_connected) return;
    
    server.on("/", handle_root);
    server.on("/api/status", handle_api_status);
    server.on("/api/detection", handle_api_detection);
    server.on("/stream", HTTP_GET, handle_stream); // <-- ADDED
    
    server.begin();
    Serial.println("Web server started");
}

void handle_root() {
    // UPDATED HTML
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-CAM Device Detection</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .header { text-align: center; margin-bottom: 20px;}
        .main-container { display: flex; flex-wrap: wrap; gap: 20px; }
        .stream-container { flex: 1; min-width: 320px; }
        .info-container { flex: 1; min-width: 300px; }
        .stream-img { width: 100%; max-width: 640px; border: 1px solid #ccc; border-radius: 10px; background-color: #000; }
        .status { padding: 10px; margin: 10px 0; border-radius: 5px; }
        .status.running { background-color: #d4edda; color: #155724; }
        .status.error { background-color: #f8d7da; color: #721c24; }
        .detection-box { border: 2px solid #007bff; padding: 15px; margin-top: 10px; border-radius: 5px; }
        .confidence { font-weight: bold; color: #007bff; }
        .timestamp { color: #666; font-size: 0.9em; }
    </style>
</head>
<body>
    <div class="header">
        <h1>ESP32-CAM Device Detection System</h1>
    </div>

    <div class="main-container">
        <div class="stream-container">
            <h3>Live Camera Feed</h3>
            <img class="stream-img" src="/stream">
        </div>
        <div class="info-container">
            <h3>System Information</h3>
            <div id="status" class="status">Loading...</div>
            <div id="detection" class="detection-box">
                <h4>Latest Detection</h4>
                <div id="detection-content">No data available</div>
            </div>
        </div>
    </div>

    <script>
        function refreshData() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    const statusDiv = document.getElementById('status');
                    statusDiv.textContent = 'System Status: ' + data.status;
                    statusDiv.className = 'status ' + (data.status.includes('Error') ? 'error' : 'running');
                });
                
            fetch('/api/detection')
                .then(response => response.json())
                .then(data => {
                    const detectionDiv = document.getElementById('detection-content');
                    if (data.valid) {
                        detectionDiv.innerHTML = 
                            '<p><strong>Device:</strong> ' + data.device_name + '</p>' +
                            '<p><strong>Confidence:</strong> <span class="confidence">' + (data.confidence * 100).toFixed(1) + '%</span></p>' +
                            '<p><strong>Position:</strong> x:' + data.x + ', y:' + data.y + ', w:' + data.width + ', h:' + data.height + '</p>' +
                            '<p class="timestamp">Last updated: ' + new Date(data.timestamp).toLocaleString() + '</p>';
                    } else {
                        detectionDiv.innerHTML = '<p>No device detected</p>';
                    }
                });
        }
        
        // Auto-refresh every 2 seconds
        setInterval(refreshData, 2000);
        
        // Initial load
        refreshData();
    </script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

void handle_api_status() {
    DynamicJsonDocument doc(200);
    doc["status"] = system_status;
    doc["wifi_connected"] = wifi_connected;
    doc["ip_address"] = WiFi.localIP().toString();
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handle_api_detection() {
    DynamicJsonDocument doc(500);
    doc["valid"] = latest_detection.valid;
    doc["device_name"] = latest_detection.device_name;
    doc["confidence"] = latest_detection.confidence;
    doc["x"] = latest_detection.x;
    doc["y"] = latest_detection.y;
    doc["width"] = latest_detection.width;
    doc["height"] = latest_detection.height;
    doc["timestamp"] = latest_detection.timestamp;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

// ADDED THIS ENTIRE NEW FUNCTION
void handle_stream() {
    WiFiClient client = server.client();
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server.sendContent(response);

    while (client.connected()) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            // If we fail to capture, it's better to break the loop 
            // and let the client reconnect than to get stuck here.
            break; 
        }

        client.print("--frame\r\n");
        client.print("Content-Type: image/jpeg\r\n");
        client.print("Content-Length: ");
        client.println(fb->len);
        client.print("\r\n");
        client.write(fb->buf, fb->len);
        client.print("\r\n");
        
        esp_camera_fb_return(fb);

        // A small delay is crucial to allow the main loop (and other tasks) to run.
        delay(100); 
    }
}

void update_display(const DetectionResult& result) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    
    // WiFi status
    if (wifi_connected) {
        display.print("IP: ");
        display.println(WiFi.localIP().toString());
    } else {
        display.println("WiFi: Disconnected");
    }
    
    display.println("---");
    
    // Detection results
    if (result.valid && result.confidence > 0.3) {
        display.setTextSize(1);
        display.print("Device: ");
        display.println(result.device_name);
        
        display.setTextSize(2);
        display.print((int)(result.confidence * 100));
        display.println("%");
        
        display.setTextSize(1);
        display.print("Pos: ");
        display.print(result.x);
        display.print(",");
        display.println(result.y);
    } else {
        display.setTextSize(2);
        display.println("Scanning...");
    }
    
    display.display();
}

void display_wifi_info() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Connected!");
    display.print("IP: ");
    display.println(WiFi.localIP());
    display.println("Web interface:");
    display.println("http://");
    display.print(WiFi.localIP());
    display.display();
    delay(3000);
}

void display_error(const char* error_msg) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("ERROR:");
    display.println(error_msg);
    display.display();
}

// Original camera functions (unchanged)
bool ei_camera_init(void) {
    if (is_initialised) return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ei_printf("Camera deinit failed\n");
        return;
    }
    is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if(!converted){
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif