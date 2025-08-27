#include <Adafruit_GFX.h>    // Adafruit 图形库
#include <Adafruit_ST7789.h> // ST7789 TFT 驱动
#include <SPI.h>
#include <SI4735.h>
#include <Wire.h>
#include <Fonts/FreeSansBold24pt7b.h> // 大号字体：用于频率显示
#include <Fonts/FreeSans9pt7b.h>      // 小号字体：用于文本标签

// ESP32 DevKit TFT引脚定义
#define TFT_CS 5     // 片选引脚
#define TFT_RST 15   // 复位引脚
#define TFT_DC 2     // 数据/命令切换引脚
#define TFT_MOSI 23  // SPI 数据输出
#define TFT_SCLK 18  // SPI 时钟
#define TFT_BL 13    // 背光 PWM 控制引脚

//收音机引脚定义
#define RESET_PIN    4  // GPIO4连接到SI4735的RST引脚
#define ESP32_I2C_SDA 21 // GPIO21连接到SDIO
#define ESP32_I2C_SCL 22 // GPIO22连接到SCLK

#define AM_FUNCTION 1   // AM模式标志
#define FM_FUNCTION 0   // FM模式标志

uint16_t currentFrequency;   // 当前频率
uint16_t previousFrequency;  // 上一次的频率
uint8_t bandwidthIdx = 0;   // AM带宽索引
const char *bandwidth[] = {"6", "4", "3", "2", "1", "1.8", "2.5"}; // 支持的AM带宽列表

// 状态缓存变量（避免闪屏）
uint16_t lastFrequency = 0;
uint8_t lastVolume = 255;
uint8_t lastRSSI = 255;
uint8_t lastSNR = 255;
bool lastIsFM = true;
bool lastStereo = false;

SI4735 rx; // 创建SI4735收音机对象

void showStatus(); 
void showHelp();

// 背光 PWM 配置 (使用 LEDC)
constexpr int BL_CH = 0;           // LEDC 通道号
constexpr int BL_RES_BITS = 8;     // PWM 分辨率：8位，范围0-255
constexpr int BL_FREQ_HZ = 20000;  // PWM 频率：20kHz，高于人耳可听范围

// 显示屏对象初始化
Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// 界面配色方案
#define C_BG ST77XX_BLACK    // 背景色：黑色
#define C_FG ST77XX_WHITE    // 前景色：白色
#define C_ACC ST77XX_CYAN    // 强调色：青色
#define C_BAR ST77XX_GREEN   // 进度条1：绿色
#define C_BAR2 ST77XX_YELLOW // 进度条2：黄色

#ifndef ST77XX_DARKGREY
#define ST77XX_DARKGREY  0x7BEF   // ~50% 灰
#endif
#ifndef ST77XX_NAVY
#define ST77XX_NAVY      0x0010   // 深蓝 (0,0,128)
#endif
#ifndef ST77XX_DARKGREEN
#define ST77XX_DARKGREEN 0x03E0   // 暗绿，约半亮度绿
#endif

// 然后你的配色宏保持不变：
#define C_DIM   ST77XX_DARKGREY
#define C_BADGE ST77XX_NAVY

// 布局
const int TOP_BAR_H = 32;
const int FREQ_AREA_Y = 68;
const int FREQ_AREA_H = 72;
const int INFO_BAR_Y = 180;

// -------- 背光控制 --------
/**
 * @brief 初始化背光 PWM
 * 配置 LEDC 通道参数并将引脚附加到通道
 */
void blInit()
{
  ledcSetup(BL_CH, BL_FREQ_HZ, BL_RES_BITS);
  ledcAttachPin(TFT_BL, BL_CH);
}

/**
 * @brief 设置背光亮度
 * @param level 亮度级别 (0-255)
 */
void blSet(uint8_t level)
{
  ledcWrite(BL_CH, level); // 0..255
}

// -------- 工具函数 --------

/**
 * @brief 从左上角绘制文本
 * @param txt 要显示的文本
 * @param x,y 文本左上角坐标
 * @param font 字体指针（可选，nullptr 表示使用当前字体）
 * @param color 文字颜色
 * @param bgColor 背景色（可选，-1 表示透明）
 * @return 实际绘制的文本区域高度
 */
int drawTextFromTop(const char *txt, int x, int y, const GFXfont *font = nullptr, 
                   uint16_t color = C_FG, int16_t bgColor = -1)
{
    if (font) {
        tft.setFont(font);
    }

    int16_t bx, by;
    uint16_t bw, bh;
    tft.getTextBounds(txt, 0, 0, &bx, &by, &bw, &bh);

    if (bgColor >= 0) {
        tft.setTextColor(color, bgColor);
    } else {
        tft.setTextColor(color);
    }

    int baseline = y - by;
    tft.setCursor(x - bx, baseline);
    tft.print(txt);

    return bh;
}

/**
 * @brief 在指定区域居中绘制文本（优化版，减少闪烁）
 * @param txt 要显示的文本
 * @param x,y 区域左上角坐标
 * @param w,h 区域宽度和高度
 * @param font 字体指针
 * @param color 文字颜色
 * 会自动清除背景并计算文本位置使其居中显示
 */
void drawTextCentered(const char *txt, int x, int y, int w, int h, const GFXfont *font, uint16_t color)
{
    static char lastText[32] = "";
    
    tft.setFont(font);
    int16_t bx, by;
    uint16_t bw, bh;
    tft.getTextBounds(txt, 0, 0, &bx, &by, &bw, &bh);
    
    // 只在文本真正改变时才清除背景
    if (strcmp(txt, lastText) != 0) {
        tft.fillRect(x, y, w, h, C_BG);
        strncpy(lastText, txt, sizeof(lastText)-1);
    }
    
    // 计算居中位置
    int cx = x + (w - bw) / 2;
    int cy = y + (h - bh) / 2;
    
    // 绘制文本
    drawTextFromTop(txt, cx, cy, font, color, C_BG);
}

/**
 * @brief 绘制水平进度条
 * @param x,y 进度条左上角坐标
 * @param w,h 进度条宽度和高度
 * @param val 当前值
 * @param vmin,vmax 值的范围
 * @param fg 前景色（填充色）
 * @param bg 背景色
 * 自动计算填充比例，带边框
 */
void drawHBar(int x, int y, int w, int h, int val, int vmin, int vmax, uint16_t fg, uint16_t bg)
{
  if (val < vmin)
    val = vmin;
  if (val > vmax)
    val = vmax;
  int filled = (w * (val - vmin)) / (vmax - vmin);
  tft.fillRect(x, y, w, h, bg);
  if (filled > 0)
    tft.fillRect(x, y, filled, h, fg);
  tft.drawRect(x - 1, y - 1, w + 2, h + 2, C_DIM);
}

/**
 * @brief 绘制圆角标签
 * @param x,y 标签左上角坐标
 * @param txt 标签文本
 * @param bg 背景色
 * @param fg 前景色和边框色
 * 固定大小(60x24)的圆角矩形标签，文本自动居中
 */
void drawBadge(int x, int y, const char *txt, uint16_t bg, uint16_t fg)
{
    int w = 60, h = 24, r = 6;  // 稍微加大避免文字溢出
    tft.fillRoundRect(x, y, w, h, r, bg);
    tft.drawRoundRect(x, y, w, h, r, fg);
    
    // 计算文本位置使其居中
    int16_t bx, by;
    uint16_t bw, bh;
    tft.setFont(&FreeSans9pt7b);
    tft.setTextSize(1);  // 确保字体大小
    tft.getTextBounds(txt, 0, 0, &bx, &by, &bw, &bh);
    
    int tx = x + (w - bw) / 2;
    int ty = y + (h - bh) / 2;
    
    drawTextFromTop(txt, tx, ty, &FreeSans9pt7b, fg, bg);
}

// -------- UI 绘制 --------
/**
 * @brief 绘制静态UI元素
 * 包括分隔线和固定标签文本
 */
void uiDrawStatic()
{
    tft.fillScreen(C_BG);
    tft.drawFastHLine(0, TOP_BAR_H, 240, C_DIM);
    tft.drawFastHLine(0, INFO_BAR_Y - 2, 240, C_DIM);
    
    // 使用新的文本绘制函数
    drawTextFromTop("RSSI", 8, INFO_BAR_Y + 4, &FreeSans9pt7b, C_FG, C_BG);
    drawTextFromTop("SNR", 8, INFO_BAR_Y + 26, &FreeSans9pt7b, C_FG, C_BG);
}

/**
 * @brief 设置波段指示
 * @param isFM true=FM模式，false=AM模式
 */
void uiSetBand(bool isFM)
{
  drawBadge(8, 6, isFM ? "FM" : "AM", C_BADGE, C_ACC);
}

/**
 * @brief 设置立体声指示
 * @param stereo true=显示ST标志，false=隐藏
 */
void uiSetStereo(bool stereo)
{
  if (stereo)
    drawBadge(240 - 8 - 60, 6, "ST", ST77XX_DARKGREEN, C_FG);
  else
    tft.fillRect(240 - 8 - 60, 6, 60, 24, C_BG);
}

/**
 * @brief 显示FM频率
 * @param mhz 频率值(MHz)，如106.50
 * 大字体显示，自动格式化为两位小数
 */
void uiSetFreqFM(float mhz)
{
  char buf[32];
  snprintf(buf, sizeof(buf), "%.2f", mhz); // 如 106.50
  drawTextCentered(buf, 0, FREQ_AREA_Y, 240, FREQ_AREA_H, &FreeSansBold24pt7b, C_FG);

  const int freq_right = 240/2 + 80;  // 根据实际布局调整位置
  const int unit_y = FREQ_AREA_Y + 40;  // 根据实际布局调整位置
  drawTextFromTop("MHz", freq_right, unit_y, &FreeSans9pt7b, C_FG, C_BG);
}

/**
 * @brief 显示AM频率
 * @param khz 频率值(kHz)，如1530
 */
void uiSetFreqAM(int khz)
{
  char buf[16];
  snprintf(buf, sizeof(buf), "%d", khz);
  drawTextCentered(buf, 0, FREQ_AREA_Y, 240, FREQ_AREA_H, &FreeSansBold24pt7b, C_FG);

  const int freq_right = 240/2 + 80;  // 根据实际布局调整位置
  const int unit_y = FREQ_AREA_Y + 40;  // 根据实际布局调整位置
  drawTextFromTop("KHz", freq_right, unit_y, &FreeSans9pt7b, C_FG, C_BG);
}

/**
 * @brief 更新信号强度指示
 * @param rssi_dBuV 信号强度(dBuV)，建议范围0-80
 */
void uiSetRSSI(int rssi_dBuV)
{ // 建议映射 0..80
  drawHBar(60, INFO_BAR_Y + 6, 240 - 68, 12, rssi_dBuV, 0, 80, C_BAR, C_DIM);
}

/**
 * @brief 更新信噪比指示
 * @param snr_dB 信噪比(dB)，建议范围0-30
 */
void uiSetSNR(int snr_dB)
{ // 建议映射 0..30
  drawHBar(60, INFO_BAR_Y + 28, 240 - 68, 12, snr_dB, 0, 30, C_BAR2, C_DIM);
}

/**
 * @brief 更新音量显示
 * @param vol_0_63 音量值，范围0-63
 * 在顶栏下方显示音量条
 */
void uiSetVolume(int vol_0_63)
{ // 0..63
  // 顶栏下横条
  int y = TOP_BAR_H + 6;
  static int lastVol = -1;
  
  // 只在音量改变时重绘标签
  if (vol_0_63 != lastVol) {
    tft.fillRect(8, y - 2, 56, 16, C_BG);
    drawTextFromTop("VOL", 8, y, &FreeSans9pt7b, C_FG, C_BG);
    lastVol = vol_0_63;
  }
  
  drawHBar(56, y, 170, 10, vol_0_63, 0, 63, ST77XX_CYAN, C_DIM);
}

// 显示命令帮助
void showHelp()
{
  Serial.println("Commands: F=FM, A=AM, U/D=Freq +/-, S/s=Seek, +=Vol+, -=Vol-, B=BW, 0=Status, ?=Help");
  Serial.println("==================================================");
}

// 显示当前收听状态，包括频率、信号质量等
void showStatus()
{
  previousFrequency = currentFrequency = rx.getCurrentFrequency(); // 获取当前频率
  rx.getCurrentReceivedSignalQuality(); // 获取当前信号质量
  Serial.print("Tuned to ");
  if (rx.isCurrentTuneFM())     // 如果当前为FM模式
  {
    Serial.print(String(currentFrequency / 100.0, 2)); // 显示MHz频率
    Serial.print(" MHz ");
    Serial.print((rx.getCurrentPilot()) ? "STEREO" : "MONO"); // 显示立体声/单声道
  }
  else
  {
    Serial.print(currentFrequency); // 显示AM频率（单位kHz）
    Serial.print(" kHz");
  }
  Serial.print(" [SNR:");
  Serial.print(rx.getCurrentSNR());    // 信噪比
  Serial.print("dB Signal:");
  Serial.print(rx.getCurrentRSSI());   // 信号强度
  Serial.println("dBuV]");
}

// 显示指定频率（AM或FM）- 修改版：同时更新屏幕
void showFrequency(uint16_t freq)
{
  // 参数说明：freq为需要显示的频率值，FM时单位为100Hz，AM时单位为kHz

  // 原有串口输出
  if (rx.isCurrentTuneFM())
  {
    Serial.print(String(freq / 100.0, 2));
    Serial.println(" MHz");
    // 添加屏幕更新
    uiSetFreqFM(freq / 100.0);
  }
  else
  {
    Serial.print(freq);
    Serial.println(" kHz");
    // 添加屏幕更新
    uiSetFreqAM(freq);
  }
}

void setup()
{
  // 串口保持9600
  Serial.begin(9600);

  // 背光 PWM 先初始化，避免花屏时亮度刺眼
  blInit();
  blSet(255); // 全亮（0..255 可调）

  // SPI 和屏幕
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS); // MISO不用传 -1
  tft.init(240, 240);                        // 240x240
  tft.setSPISpeed(40000000);                 // 若不稳降到 27/20 MHz
  tft.setRotation(0);                        // 左上角为(0,0)

  pinMode(RESET_PIN, OUTPUT);         // 设置RESET引脚为输出
  digitalWrite(RESET_PIN, LOW);      // 拉低复位
  delay(100);
  digitalWrite(RESET_PIN, HIGH);      // 拉高，SI4735正常工作

  // 为ESP32显式设置I2C引脚
  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL);

  Serial.println("SI4735 ESP32 DevKit Demo");
  showHelp();        // 显示帮助信息

  // 查找Si47XX的I2C地址
  int16_t si4735Addr = rx.getDeviceI2CAddress(RESET_PIN);
  if ( si4735Addr == 0 ) {   // 未找到设备
    Serial.println("Si473X not found!");
    Serial.flush();
    while (1);    // 死循环，停止程序
  } else {
    Serial.print("The SI473X I2C address is 0x");
    Serial.println(si4735Addr, HEX); // 显示找到的I2C地址
  }

  delay(500);
  rx.setup(RESET_PIN, FM_FUNCTION);      // 初始化为FM模式
  delay(500);
  rx.setFM(8400, 10800, 10650, 10);      // 设置FM频段，起始频率为106.5MHz
  delay(500);

  rx.setFrequency(9650); // 设置初始频率
  delay(500);
  Serial.println(rx.getCurrentFrequency()); // 获取当前频率

  currentFrequency = previousFrequency = rx.getCurrentFrequency(); // 获取初始频率
  rx.setVolume(45);      // 设置音量
  showStatus();          // 显示当前状态

  // 界面初始化
  uiDrawStatic();
  
  // 获取初始状态
  rx.getCurrentReceivedSignalQuality();
  lastFrequency = rx.getCurrentFrequency();
  lastVolume = rx.getCurrentVolume(); 
  lastRSSI = rx.getCurrentRSSI();
  lastSNR = rx.getCurrentSNR();
  lastIsFM = rx.isCurrentTuneFM();
  lastStereo = lastIsFM ? rx.getCurrentPilot() : false;
  
  // 初始UI显示
  uiSetBand(lastIsFM);
  uiSetStereo(lastStereo);
  if(lastIsFM) {
    uiSetFreqFM(lastFrequency / 100.0);
  } else {
    uiSetFreqAM(lastFrequency);
  }
  uiSetRSSI(lastRSSI);
  uiSetSNR(lastSNR);
  uiSetVolume(lastVolume);
}

void loop()
{
  // 获取当前状态
  rx.getCurrentReceivedSignalQuality();
  uint16_t currentFreq = rx.getCurrentFrequency();
  uint8_t currentVol = rx.getCurrentVolume();
  uint8_t currentRSSI = rx.getCurrentRSSI();
  uint8_t currentSNR = rx.getCurrentSNR();
  bool currentIsFM = rx.isCurrentTuneFM();
  bool currentStereo = currentIsFM ? rx.getCurrentPilot() : false;

  // 只在状态真正改变时才更新UI（避免闪屏）
  if(currentIsFM != lastIsFM) {
    uiSetBand(currentIsFM);
    lastIsFM = currentIsFM;
  }
  
  if(currentFreq != lastFrequency) {
    if(currentIsFM) {
      uiSetFreqFM(currentFreq / 100.0);
    } else {
      uiSetFreqAM(currentFreq);
    }
    lastFrequency = currentFreq;
  }
  
  if(currentStereo != lastStereo) {
    uiSetStereo(currentStereo);
    lastStereo = currentStereo;
  }
  
  if(currentVol != lastVolume) {
    uiSetVolume(currentVol);
    lastVolume = currentVol;
  }
  
  if(currentRSSI != lastRSSI) {
    uiSetRSSI(currentRSSI);
    lastRSSI = currentRSSI;
  }
  
  if(currentSNR != lastSNR) {
    uiSetSNR(currentSNR);
    lastSNR = currentSNR;
  }

  // 串口命令处理
  if (Serial.available() > 0)   // 如果串口有输入
  {
    char key = Serial.read();   // 读取一个字符命令
    switch (key)
    {
      case '+':  // 音量增加
        rx.volumeUp();
        break;
      case '-':  // 音量减少
        rx.volumeDown();
        break;
      case 'a':  // 切换到AM模式
      case 'A':
        rx.setAM(520, 1710, 1000, 10);     // 设置MW频段，起始1000kHz
        break;
      case 'f':  // 切换到FM模式
      case 'F':
        rx.setFM(8400, 10800, 10650, 10);  // 设置FM频段，起始106.5MHz
        break;
      case 'U':  // 频率上升
      case 'u':
        rx.frequencyUp();
        break;
      case 'D':  // 频率下降
      case 'd':
        rx.frequencyDown();
        break;
      case 'B':  // 切换AM带宽
      case 'b':
        if (!rx.isCurrentTuneFM()) // 仅在AM模式下有效
        {
          if (bandwidthIdx > 6) bandwidthIdx = 0; // 循环带宽参数
          rx.setBandwidth(bandwidthIdx, 1);       // 设置带宽（带宽索引，自动增益=1）
          Serial.print("AM Filter: ");
          Serial.print(bandwidth[bandwidthIdx]);
          Serial.println(" kHz");
          bandwidthIdx++;
        }
        break;
      case 'S':  // 自动搜台（向上）- 现在会同时更新屏幕
        rx.seekStationProgress(showFrequency, 1);
        break;
      case 's':  // 自动搜台（向下）- 现在会同时更新屏幕
        rx.seekStationProgress(showFrequency, 0);
        break;
      case '0':  // 显示当前状态
        showStatus();
        break;
      case '?':  // 显示帮助
        showHelp();
        break;
      default:
        break;
    }
  }

  delay(100);
}
