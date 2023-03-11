/*
 * Project: esp32camOverlay
 * Description: Example sketch to show, how to draw text or lines on a esp32-cam captured photo and save the picture on a local SD card
 * 
 * This could be useful for example, if want to show a timestamp, logo oder label on a photo.
 * Due memory limitations no resolution higher then XGA (1024x768) is possible with 4 MB psram 
 * If you get incomplete jpg images reduce jpg quality
 * 
 * License: CC0
 * 
 * created by codingABI https://github.com/codingABI/esp32camOverlay
 * 
 * Needed hardware and environment:
 * - ESP32 Cam-Modul with ESP32-S Microcontroller, 2MP OV2640, 4 MB psram (In Boardmanager: "Ai Thinker ESP32-CAM")
 * - Sdk/IDF-Version v4.4.2
 * - Arduino ESP32 Release 2_0_5
 *
 * History:
 * 11.03.2023, Initial version
 */
 
#include <esp_camera.h>
#include <rom/rtc.h>
#include <FS.h>
#include <SD_MMC.h>
#include <fb_gfx.h>
#include <img_converters.h>
#include <core_version.h>

#define PICTURE_FILENAME "/overlay.jpg" // Filename for photo on SD card

#define LED_PIN 33 // Builtin led (low active). Only used to show, when device is resetting in case a problem

// Pin definition for my CAMERA_MODEL_AI_THINKER
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

// My esp32cam seem so need some delay after camera init and first photo to get acceptable pictures
#define ESPCAMSTARTUPTIME_MS 1500

// Draw ellipse on framebuffer (Code based on https://de.wikipedia.org/wiki/Bresenham-Algorithmus)
void fb_gfx_ellipse(fb_data_t *fb, int xm, int ym, int a, int b, uint32_t color) {
  switch (fb->format) {
    case FB_RGB565: if (fb->bytes_per_pixel != 2) return; break; // Format known, but bytes per pixel does not match
    case FB_RGB888: if (fb->bytes_per_pixel != 3) return; break; // Format known, but bytes per pixel does not match
    default: return; // Unknown format
  }
  if (a == 0) return;
  if (b == 0) return;
  
  uint8_t c0 = color >> 16;
  uint8_t c1 = color >> 8;
  uint8_t c2 = color;
  
  int dx = 0, dy = b; // In quadrant I. from upper left to lower right
  long a2 = a*a, b2 = b*b;
  long err = b2-(2*b-1)*a2, e2; // Error in first step

  do {
    setPixel(fb, xm + dx, ym + dy, c0, c1, c2); // quadrant I
    setPixel(fb, xm - dx, ym + dy, c0, c1, c2); // quadrant II
    setPixel(fb, xm - dx, ym - dy, c0, c1, c2); // quadrant III
    setPixel(fb, xm + dx, ym - dy, c0, c1, c2); // quadrant IV
  
    e2 = 2*err;
    if (e2 <  (2 * dx + 1) * b2) { ++dx; err += (2 * dx + 1) * b2; }
    if (e2 > -(2 * dy - 1) * a2) { --dy; err -= (2 * dy - 1) * a2; }
  } while (dy >= 0);

  while (dx++ < a) { // fix gap in flat ellipses 
    setPixel(fb, xm+dx, ym, c0, c1, c2);
    setPixel(fb, xm-dx, ym, c0, c1, c2);
  }
}

// Draw single pixel in frame buffer
void setPixel(fb_data_t *fb, int x, int y, uint8_t c0, uint8_t c1, uint8_t c2) {
  if (x < 0) return;
  if (y < 0) return;
  if (x >= fb->width) return;
  if (y >= fb->height) return;
  
  uint8_t *data = fb->data + ((x + (y * fb->width)) * fb->bytes_per_pixel);

  switch (fb->bytes_per_pixel) {
  case 2: // assume rgb565
    data[0] = c1;
    data[1] = c2;
    break;
  case 3: // assume rgb888
    data[0] = c0;
    data[1] = c1;
    data[2] = c2;
  default:
    break;
  }
}

// Disable camera as good as possible
void deinitCamera() {
  esp_camera_deinit();

  // Switch off camera power
  digitalWrite(PWDN_GPIO_NUM, HIGH);
}

// Restart esp32 in case of a problem
void reset() {
  // Turn on builtin led
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("Reset");
  Serial.flush();

  deinitCamera();

  // Disable the I2C bus
  periph_module_disable(PERIPH_I2C0_MODULE);
  periph_module_disable(PERIPH_I2C1_MODULE);

  delay(2000);

  // Reset the I2C bus
  periph_module_reset(PERIPH_I2C0_MODULE);
  periph_module_reset(PERIPH_I2C1_MODULE);

  // Turn off builtin led
  digitalWrite(LED_PIN, HIGH);
  pinMode(LED_PIN,INPUT);

  ESP.restart();
}

// Set camera settings
void setCameraSettings() {
  Serial.println("Camera settings");
  sensor_t *sensor = esp_camera_sensor_get();

  sensor->set_brightness(sensor, 0); // Brightness
  sensor->set_contrast(sensor, 0); // Contrast
  sensor->set_saturation(sensor, 0); // Saturation
  sensor->set_special_effect(sensor, 0); // Special Effect
  sensor->set_whitebal(sensor, 1); // AWB
  sensor->set_awb_gain(sensor, 1); // AWB Gain  
  sensor->set_wb_mode(sensor, 0); // WB Mode (0=Auto,...,2=Cloudy)
  sensor->set_exposure_ctrl(sensor,1); // AEC SENSOR 
  sensor->set_aec2(sensor, 0); // AEC DSP
  sensor->set_ae_level(sensor, 0); // AE Level [-2;2] (no effect, if exposure_ctrl == 0) 
  sensor->set_aec_value(sensor, 300); // Exposure (no effect, if exposure_ctrl == 1)
  sensor->set_gain_ctrl(sensor, 1); // AGC
  sensor->set_agc_gain(sensor, 0); // Gain (no effect, if gain_ctrl==0)
  sensor->set_gainceiling(sensor, (gainceiling_t)0); // Gain Ceiling (no effect, if gain_ctrl==1) )
  sensor->set_bpc(sensor, 0); // BPC
  sensor->set_wpc(sensor, 1); // WPC
  sensor->set_raw_gma(sensor, 1); // Raw GMA
  sensor->set_lenc(sensor, 1); // Lens correction
  sensor->set_hmirror(sensor, 0); // H-Mirror
  sensor->set_vflip(sensor, 0); // V-Flip
  sensor->set_colorbar(sensor, 0); // Color Bar
}

// Init camera
void initCamera() {
  camera_config_t config;

  Serial.println("Camera init");
  
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_count = 1;
  config.frame_size = FRAMESIZE_SVGA;  // Due memory limits more then XGA 1024x768 is no possible, when drawing on a framebuffer
  config.jpeg_quality = 10;
  config.xclk_freq_hz = 20000000;
  config.grab_mode = CAMERA_GRAB_LATEST;

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    reset();
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);

  // Switch off camera power
  pinMode(PWDN_GPIO_NUM,OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH);
  
  // SD init
  Serial.println("Starting SD Card");
  if(!SD_MMC.begin("/sdcard", true)) { // 1-Bit-Mode, slow, but prevents flash on GPIO04 and frees up GPIO12 and GPIO13
    Serial.println("SD Card Mount Failed");
    reset();
  }

  // Init camera
  initCamera();
  // Camera settings
  setCameraSettings();
}

// Take a photo and add a text overlay and a cross
void takePhoto() {  
  uint8_t *rgb888Buffer = NULL; // Data buffer for three bytes per pixel framebuffer
  uint8_t *outputBuffer = NULL; // Data buffer for target file format
  size_t outputLength;
  fb_data_t rfb888;
  camera_fb_t * fb = NULL;
  struct tm * ptrTimeinfo;
  time_t now;
  #define MAXSTRDATALENGTH 80
  char strData[MAXSTRDATALENGTH+1];

  Serial.println("Take photo");
 
  // Get camera settings
  sensor_t * sensor = esp_camera_sensor_get();
  
  // My esp32cam seem so need some delay after camera init and first photo to get acceptable pictures
  Serial.printf("Delay photo for %ims\n",ESPCAMSTARTUPTIME_MS);
  delay(ESPCAMSTARTUPTIME_MS);

  // Clean/dispose the camera buffer
  fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb); // dispose the buffered image
  fb = NULL; // reset to capture errors

  // Get new camera picture in buffer
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  /* 20230310 from https://github.com/espressif/esp32-camera/blob/master/driver/cam_hal.c:
    ...
    cam_obj->recv_size = cam_obj->width * cam_obj->height / 5;
    cam_obj->fb_size = cam_obj->recv_size;
    ...
  */
  if (fb->len >= fb->width*fb->height/5) { // Buffer from cam_hal.c is too small for choosen quality config.jpeg_quality 
    Serial.println("JPG quality too high. Expect incomplete images");
  }

  Serial.println("Get buffer for rgb888");

  rfb888.width = fb->width;
  rfb888.height = fb->height;
  rfb888.bytes_per_pixel = 3;

  rfb888.format = FB_RGB888;

  rgb888Buffer = (uint8_t*)calloc(1,rfb888.width*rfb888.height* rfb888.bytes_per_pixel);
  if (!rgb888Buffer) {
      Serial.println("rbg888 malloc failed");
      return;
  }

  Serial.println("Convert camera jpg buffer to rgb888 buffer");
  bool rgb888_converted = fmt2rgb888(fb->buf, fb->len, fb->format, rgb888Buffer);
  esp_camera_fb_return(fb);

  if (!rgb888_converted) {
    Serial.println("fmt2rgb888 malloc failed");      
    free(rgb888Buffer);
    return;
  }

  rfb888.data = rgb888Buffer;

  Serial.println("Write text and lines to rgb888 buffer");

  #define TEXTCOLOR 0x0000ff // Red
  #define LINECOLOR 0xff0000 // Blue
  // Font size (based on https://github.com/espressif/esp-who/blob/master/components/fb_gfx/fb_gfx.c: "FreeMonoBold12pt7b.h" => 14x24)
  uint8_t fontWidth = 14;
  uint8_t fontHeight = 24;
  // Origin with little offset from border
  int posX = 8;
  int posY = 8;

  snprintf(strData,MAXSTRDATALENGTH+1,"Compile time %s %s",__DATE__ , __TIME__);
  fb_gfx_print(&rfb888, posX, posY, TEXTCOLOR, strData);
  Serial.println(strData);
  
  time(&now);
  ptrTimeinfo = gmtime ( &now );
  strftime(strData, MAXSTRDATALENGTH+1, "UTC time %Y-%m-%d %H:%M:%S", ptrTimeinfo);
  fb_gfx_print(&rfb888, posX, posY+=fontHeight, TEXTCOLOR, strData);
  Serial.println(strData);

  // Cross at center of screen
  posX = fb->width/2-1;
  posY = fb->height/2-1;  
  for (int i=0;i<5;i++) {
    fb_gfx_ellipse(&rfb888,posX,posY,i*(fb->width/30),i*(fb->height/30), LINECOLOR);
  }
  fb_gfx_drawFastVLine(&rfb888, posX, posY - fb->height/6-1, fb->height/3, LINECOLOR);
  fb_gfx_drawFastHLine(&rfb888, posX - fb->width/6-1, posY, fb->width/3, LINECOLOR);

  Serial.println("Convert rgb888 buffer to JPG");
  
  byte outputQuality = 90;
  bool converted = false;
  do { // Convert framebuffer to jpg and repeat this with lowered quality, if size is too big for fmt2jpg
    converted = fmt2jpg(rgb888Buffer, rfb888.width*rfb888.height* rfb888.bytes_per_pixel, rfb888.width, rfb888.height, PIXFORMAT_RGB888, outputQuality, &outputBuffer, &outputLength);
    // fmt2jpg mallocs outputBuffer itself. Dont forget to free it

    /* 20230310 from https://github.com/espressif/esp32-camera/blob/master/conversions/to_jpg.cpp:
      ...
      int jpg_buf_len = 128*1024;
      ...
    */
    if (!converted) break;
    if (outputLength >= 128*1024) { // fmt2jpg supports only files <= 128kB 
      if (outputQuality > 30) {
        outputQuality -= 5; // Reduce quality
        Serial.printf("Reduce JPG quality to %i\n",outputQuality);
        free(outputBuffer);
      } else {
        Serial.println("JPG to big. Expect incomplete images");
        break;
      }
    } else break;
  } while (true);
  
  // free rgb888 buffer, because not needed any longer
  free(rgb888Buffer);

  if (converted) { // Conversion was successful => Write to SD card
    snprintf(strData,MAXSTRDATALENGTH+1,PICTURE_FILENAME);
    Serial.printf("Write to file: %s\n", strData);

    fs::FS &fs = SD_MMC; 
    File file = fs.open(strData, FILE_WRITE); // Open file for writing
    
    if(!file){
      Serial.println("Failed to open file in writing mode");
      free(outputBuffer);
      return;
    }

    if (!file.write(outputBuffer, outputLength)) { // Write to file
      Serial.println("Write to file failed");
    }

    file.close();
  } else {
    Serial.println("Conversion to JPG failed");
  }

  // free jpg buffer, because not needed any longer
  free(outputBuffer);
}

void loop() {
  // Just take a photo
  takePhoto();

  // and go to deep sleep until next press of the reset button
  Serial.println("Deep sleep");
  Serial.flush(); 
  esp_deep_sleep_start();
}
