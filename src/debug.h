#ifndef __DH_DEBUG_H__
#define __DH_DEBUG_H__

#include <inttypes.h>
#include <ostream>
#include <iostream>
#include <sstream>

// nur temporär für debugging -> kann dann wieder wech ...
#define DEBUG_STACK_HIGH_WATERMARK RADIATOR_LOG_WARN(millis() << " ms: " << uxTaskGetStackHighWaterMark(NULL) << " -> uxTaskGetStackHighWaterMark (" << pcTaskGetTaskName(NULL) << ")" << std::endl;);

static std::ostream &null()
{
  static std::ostringstream m_null;
  m_null.str("");
  m_null.clear();
  return m_null;
}

#define LOG_trace (debug_level >= 5 ? std::cerr : null())
#define LOG_debug (debug_level >= 4 ? std::cerr : null())
#define LOG_info (debug_level >= 3 ? std::cerr : null())
#define LOG_warn (debug_level >= 2 ? std::cerr : null())
#define LOG_error (debug_level >= 1 ? std::cerr : null())
#define LOG_fatal (std::cerr)

extern uint8_t debug_level;

// ugly hack to eliminate logging from compilation & flash without changing original "Hoepfl source"
#if D_DEBUG_LEVEL >= 5
#define RADIATOR_LOG_TRACE(...) LOG_trace << __VA_ARGS__
#else
#define RADIATOR_LOG_TRACE(...) ;
#endif

#if D_DEBUG_LEVEL >= 4
#define RADIATOR_LOG_DEBUG(...) LOG_debug << __VA_ARGS__
#else
#define RADIATOR_LOG_DEBUG(...) ;
#endif

#if D_DEBUG_LEVEL >= 3
#define RADIATOR_LOG_INFO(...) LOG_info << __VA_ARGS__
#else
#define RADIATOR_LOG_INFO(...) ;
#endif

#if D_DEBUG_LEVEL >= 2
#define RADIATOR_LOG_WARN(...) LOG_warn << __VA_ARGS__
#else
#define RADIATOR_LOG_WARN(...) ;
#endif

#if D_DEBUG_LEVEL >= 1
#define RADIATOR_LOG_ERROR(...) LOG_error << __VA_ARGS__
#else
#define RADIATOR_LOG_ERROR(...) ;
#endif

void dump_string(const char *prefix,
                 const uint8_t *data,
                 int len,
                 std::ostream &ostream = LOG_debug);

#endif

// only for debugging ...
// #include "esp32/rom/rtc.h" //to get and print the reset reason
// static const char *resetReason[]{
//     "NO_MEAN",
//     "POWERON_RESET: Vbat power on reset",
//     "NO_MEAN",
//     "SW_RESET: Software reset digital core",
//     "OWDT_RESET: Legacy watch dog reset digital core",
//     "DEEPSLEEP_RESET: Deep Sleep reset digital core",
//     "SDIO_RESET: Reset by SLC module, reset digital core",
//     "TG0WDT_SYS_RESET: Timer Group0 Watch dog reset digital core",
//     "TG1WDT_SYS_RESET: Timer Group1 Watch dog reset digital core",
//     "RTCWDT_SYS_RESET: RTC Watch dog Reset digital core",
//     "INTRUSION_RESET: Instrusion tested to reset CPU",
//     "TGWDT_CPU_RESET: Time Group reset CPU",
//     "SW_CPU_RESET: Software reset CPU",
//     "RTCWDT_CPU_RESET: RTC Watch dog Reset CPU",
//     "EXT_CPU_RESET: for APP CPU, reseted by PRO CPU",
//     "RTCWDT_BROWN_OUT_RESET: Reset when the vdd voltage is not stable",
//     "RTCWDT_RTC_RESET: RTC Watch dog reset digital core and rtc module",
//     "NO_MEAN"};

// int resetReasonCPU0 = rtc_get_reset_reason(0);
// int resetReasonCPU1 = rtc_get_reset_reason(1);
// Serial.printf("resetReason CPU0: %s / CPU1: %s \n", resetReason[resetReasonCPU0], resetReason[resetReasonCPU1]);

// char message[128];
// snprintf(message, sizeof(message), "Last reset reason CPU0= %s / CPU1= %s", resetReason[resetReasonCPU0], resetReason[resetReasonCPU1]);
// // std::string message = "last reset reason CPU0= " + (std::string)resetReason[resetReasonCPU0] + " / CPU1= " + (std::string)resetReason[resetReasonCPU1];
// netHandler.publishToMQTT(message, "/error");
// RADIATOR_LOG_ERROR(message << std::endl;)
