#pragma once
// BOX-3-SENSOR dock features for Ziyuan: AHT30 temperature/humidity + infrared learn/send, exposed as MCP tools.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_encoder.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>
#include <freertos/task.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "mcp_server.h"

namespace sensor_dock {

static const char* TAG_SD = "SensorDock";

constexpr gpio_num_t kIrCtrl = GPIO_NUM_44;  // low = IR enabled
constexpr gpio_num_t kIrTx = GPIO_NUM_39;
constexpr gpio_num_t kIrRx = GPIO_NUM_38;
constexpr gpio_num_t kSensorScl = GPIO_NUM_40;
constexpr gpio_num_t kSensorSda = GPIO_NUM_41;
constexpr float kTempOffsetC = 0.0f;  // board self-heating compensation if needed

constexpr int kMaxSlots = 16;
constexpr size_t kMaxPulses = 800;     // max stored mark/space entries
constexpr uint32_t kResolutionHz = 1000000;  // 1 us per tick

// ---------------- AHT30 ----------------
class Aht30 {
public:
    bool Init() {
        i2c_master_bus_config_t cfg = {};
        cfg.i2c_port = (i2c_port_t)0;
        cfg.sda_io_num = kSensorSda;
        cfg.scl_io_num = kSensorScl;
        cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        cfg.glitch_ignore_cnt = 7;
        cfg.flags.enable_internal_pullup = 1;
        if (i2c_new_master_bus(&cfg, &bus_) != ESP_OK) {
            ESP_LOGE(TAG_SD, "sensor i2c bus init failed");
            return false;
        }
        i2c_device_config_t dev = {};
        dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev.device_address = 0x38;
        dev.scl_speed_hz = 100000;
        if (i2c_master_bus_add_device(bus_, &dev, &dev_) != ESP_OK) {
            ESP_LOGE(TAG_SD, "AHT30 add device failed");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};
        i2c_master_transmit(dev_, init_cmd, 3, 100);
        vTaskDelay(pdMS_TO_TICKS(20));
        ok_ = true;
        return true;
    }

    bool Read(float& temp, float& hum) {
        if (!ok_) return false;
        uint8_t trig[3] = {0xAC, 0x33, 0x00};
        if (i2c_master_transmit(dev_, trig, 3, 100) != ESP_OK) return false;
        vTaskDelay(pdMS_TO_TICKS(90));
        uint8_t d[7] = {0};
        for (int i = 0; i < 5; i++) {
            if (i2c_master_receive(dev_, d, 7, 100) != ESP_OK) return false;
            if (!(d[0] & 0x80)) break;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (d[0] & 0x80) return false;
        uint32_t raw_h = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
        uint32_t raw_t = ((uint32_t)(d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];
        hum = raw_h * 100.0f / 1048576.0f;
        temp = raw_t * 200.0f / 1048576.0f - 50.0f + kTempOffsetC;
        return true;
    }

private:
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;
    bool ok_ = false;
};

// ---------------- IR ----------------
static bool IRAM_ATTR RxDone(rmt_channel_handle_t, const rmt_rx_done_event_data_t* edata, void* ctx) {
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)ctx, edata, &woken);
    return woken == pdTRUE;
}

class Ir {
public:
    void Init() {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << kIrCtrl;
        io.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io);
        gpio_set_level(kIrCtrl, 1);  // disabled until used
        mutex_ = xSemaphoreCreateMutex();
    }

    // Start learning in the background (tool calls run on the main loop, so never block it).
    std::string StartLearn(const std::string& name, int timeout_s) {
        if (name.empty() || name.size() > 30) return "name must be 1-30 bytes";
        if (learning_) return "already listening, wait for it to finish";
        learning_ = true;
        last_result_ = "listening";
        auto* arg = new LearnArg{this, name, timeout_s};
        if (xTaskCreate(LearnTask, "ir_learn", 6144, arg, 4, nullptr) != pdPASS) {
            delete arg;
            learning_ = false;
            last_result_ = "task error";
            return "task error";
        }
        return "listening for " + std::to_string(timeout_s) +
               " seconds: tell the user to point the remote at the robot base and press the button once now, "
               "then call self.ir.status to confirm";
    }

    std::string Status() { return last_result_; }

    std::string Send(const std::string& name) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        std::string result = SendLocked(name);
        xSemaphoreGive(mutex_);
        return result;
    }

    std::string List() {
        nvs_handle_t h;
        if (nvs_open("ir", NVS_READONLY, &h) != ESP_OK) return "[]";
        std::string out = "[";
        bool first = true;
        for (int i = 0; i < kMaxSlots; i++) {
            char key[8];
            snprintf(key, sizeof(key), "n%d", i);
            char buf[40];
            size_t len = sizeof(buf);
            if (nvs_get_str(h, key, buf, &len) == ESP_OK) {
                if (!first) out += ",";
                out += "\"" + std::string(buf) + "\"";
                first = false;
            }
        }
        nvs_close(h);
        return out + "]";
    }

    std::string Delete(const std::string& name) {
        nvs_handle_t h;
        if (nvs_open("ir", NVS_READWRITE, &h) != ESP_OK) return "storage error";
        int slot = FindSlot(h, name);
        if (slot < 0) { nvs_close(h); return "not found"; }
        char kn[8], kd[8];
        snprintf(kn, sizeof(kn), "n%d", slot);
        snprintf(kd, sizeof(kd), "d%d", slot);
        nvs_erase_key(h, kn);
        nvs_erase_key(h, kd);
        nvs_commit(h);
        nvs_close(h);
        return "deleted";
    }

private:
    SemaphoreHandle_t mutex_ = nullptr;
    volatile bool learning_ = false;
    std::string last_result_ = "idle";

    struct LearnArg { Ir* self; std::string name; int timeout_s; };

    static void LearnTask(void* p) {
        std::unique_ptr<LearnArg> a((LearnArg*)p);
        xSemaphoreTake(a->self->mutex_, portMAX_DELAY);
        std::string r = a->self->LearnLocked(a->name, a->timeout_s);
        xSemaphoreGive(a->self->mutex_);
        a->self->last_result_ = r == "learned" ? "learned: " + a->name : r;
        a->self->learning_ = false;
        vTaskDelete(nullptr);
    }

    static int FindSlot(nvs_handle_t h, const std::string& name) {
        for (int i = 0; i < kMaxSlots; i++) {
            char key[8];
            snprintf(key, sizeof(key), "n%d", i);
            char buf[40];
            size_t len = sizeof(buf);
            if (nvs_get_str(h, key, buf, &len) == ESP_OK && name == buf) return i;
        }
        return -1;
    }

    static int FreeSlot(nvs_handle_t h) {
        for (int i = 0; i < kMaxSlots; i++) {
            char key[8];
            snprintf(key, sizeof(key), "n%d", i);
            size_t len = 0;
            if (nvs_get_str(h, key, nullptr, &len) == ESP_ERR_NVS_NOT_FOUND) return i;
        }
        return -1;
    }

    std::string LearnLocked(const std::string& name, int timeout_s) {
        gpio_set_level(kIrCtrl, 0);
        QueueHandle_t q = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
        rmt_rx_channel_config_t rx_cfg = {};
        rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        rx_cfg.resolution_hz = kResolutionHz;
        rx_cfg.mem_block_symbols = 96;
        rx_cfg.gpio_num = kIrRx;
        rmt_channel_handle_t rx = nullptr;
        if (rmt_new_rx_channel(&rx_cfg, &rx) != ESP_OK) {
            vQueueDelete(q);
            gpio_set_level(kIrCtrl, 1);
            return "rx channel error";
        }
        rmt_rx_event_callbacks_t cbs = {};
        cbs.on_recv_done = RxDone;
        rmt_rx_register_event_callbacks(rx, &cbs, q);
        rmt_enable(rx);

        const size_t kBufSymbols = 1024;
        auto* buf = (rmt_symbol_word_t*)heap_caps_malloc(kBufSymbols * sizeof(rmt_symbol_word_t), MALLOC_CAP_8BIT);
        if (buf == nullptr) {
            rmt_disable(rx);
            rmt_del_channel(rx);
            vQueueDelete(q);
            gpio_set_level(kIrCtrl, 1);
            return "out of memory";
        }
        rmt_receive_config_t rc = {};
        rc.signal_range_min_ns = 1250;
        rc.signal_range_max_ns = 30 * 1000 * 1000;  // 30 ms of silence ends a frame

        std::string result;
        rmt_rx_done_event_data_t ev;
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_s * 1000);
        bool saved = false;
        while (!saved && xTaskGetTickCount() < deadline) {
            if (rmt_receive(rx, buf, kBufSymbols * sizeof(rmt_symbol_word_t), &rc) != ESP_OK) {
                result = "receive error";
                break;
            }
            TickType_t remain = deadline - xTaskGetTickCount();
            if (xQueueReceive(q, &ev, remain) != pdTRUE) break;
            // flatten to mark(level 0 from demodulator)/space durations, mark first
            std::vector<uint16_t> pulses;
            bool started = false;
            for (size_t i = 0; i < ev.num_symbols; i++) {
                const rmt_symbol_word_t& s = ev.received_symbols[i];
                uint32_t lv[2] = {s.level0, s.level1};
                uint32_t du[2] = {s.duration0, s.duration1};
                for (int k = 0; k < 2; k++) {
                    if (du[k] == 0) continue;
                    bool mark = (lv[k] == 0);
                    if (!started) {
                        if (!mark) continue;
                        started = true;
                    }
                    size_t idx = pulses.size();
                    bool expect_mark = (idx % 2 == 0);
                    if (mark == expect_mark) {
                        pulses.push_back(du[k] > 65535 ? 65535 : du[k]);
                    } else if (!pulses.empty()) {
                        uint32_t sum = pulses.back() + du[k];
                        pulses.back() = sum > 65535 ? 65535 : sum;
                    }
                }
            }
            if (pulses.size() % 2 == 0 && !pulses.empty()) pulses.pop_back();  // drop trailing space
            ESP_LOGI(TAG_SD, "IR frame: %u pulses", (unsigned)pulses.size());
            if (pulses.size() < 6) continue;  // noise, wait for a real one
            if (pulses.size() > kMaxPulses) {
                result = "signal too long";
                break;
            }
            result = Store(name, pulses);
            saved = (result == "learned");
        }
        if (!saved && result.empty()) result = "timeout, no remote signal received";

        rmt_disable(rx);
        rmt_del_channel(rx);
        free(buf);
        vQueueDelete(q);
        gpio_set_level(kIrCtrl, 1);
        return result;
    }

    std::string Store(const std::string& name, const std::vector<uint16_t>& pulses) {
        nvs_handle_t h;
        if (nvs_open("ir", NVS_READWRITE, &h) != ESP_OK) return "storage error";
        int slot = FindSlot(h, name);
        if (slot < 0) slot = FreeSlot(h);
        if (slot < 0) { nvs_close(h); return "storage full, delete one first"; }
        char kn[8], kd[8];
        snprintf(kn, sizeof(kn), "n%d", slot);
        snprintf(kd, sizeof(kd), "d%d", slot);
        esp_err_t e1 = nvs_set_str(h, kn, name.c_str());
        esp_err_t e2 = nvs_set_blob(h, kd, pulses.data(), pulses.size() * sizeof(uint16_t));
        esp_err_t e3 = nvs_commit(h);
        nvs_close(h);
        if (e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK) return "storage write failed";
        return "learned";
    }

    std::string SendLocked(const std::string& name) {
        nvs_handle_t h;
        if (nvs_open("ir", NVS_READONLY, &h) != ESP_OK) return "no codes stored";
        int slot = FindSlot(h, name);
        if (slot < 0) { nvs_close(h); return "code not found, learn it first"; }
        char kd[8];
        snprintf(kd, sizeof(kd), "d%d", slot);
        size_t len = 0;
        nvs_get_blob(h, kd, nullptr, &len);
        std::vector<uint16_t> pulses(len / sizeof(uint16_t));
        nvs_get_blob(h, kd, pulses.data(), &len);
        nvs_close(h);

        std::vector<rmt_symbol_word_t> syms((pulses.size() + 1) / 2 + 1);
        size_t n = 0;
        for (size_t i = 0; i < pulses.size(); i += 2) {
            rmt_symbol_word_t s = {};
            s.level0 = 1;
            s.duration0 = pulses[i];
            s.level1 = 0;
            s.duration1 = (i + 1 < pulses.size()) ? pulses[i + 1] : 1000;
            syms[n++] = s;
        }
        rmt_symbol_word_t endsym = {};  // zero duration terminates
        syms[n++] = endsym;

        gpio_set_level(kIrCtrl, 0);
        rmt_tx_channel_config_t tx_cfg = {};
        tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        tx_cfg.resolution_hz = kResolutionHz;
        tx_cfg.mem_block_symbols = 48;
        tx_cfg.trans_queue_depth = 2;
        tx_cfg.gpio_num = kIrTx;
        rmt_channel_handle_t tx = nullptr;
        if (rmt_new_tx_channel(&tx_cfg, &tx) != ESP_OK) {
            gpio_set_level(kIrCtrl, 1);
            return "tx channel error";
        }
        rmt_carrier_config_t carrier = {};
        carrier.frequency_hz = 38000;
        carrier.duty_cycle = 0.33f;
        rmt_apply_carrier(tx, &carrier);
        rmt_copy_encoder_config_t enc_cfg = {};
        rmt_encoder_handle_t enc = nullptr;
        rmt_new_copy_encoder(&enc_cfg, &enc);
        rmt_enable(tx);
        rmt_transmit_config_t tc = {};
        esp_err_t err = rmt_transmit(tx, enc, syms.data(), n * sizeof(rmt_symbol_word_t), &tc);
        if (err == ESP_OK) rmt_tx_wait_all_done(tx, 2000);
        rmt_disable(tx);
        rmt_del_encoder(enc);
        rmt_del_channel(tx);
        gpio_set_level(kIrCtrl, 1);
        return err == ESP_OK ? "sent" : "send failed";
    }
};

inline void RegisterTools() {
    static Aht30 aht;
    static Ir ir;
    aht.Init();
    ir.Init();
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.sensor.get_temperature_humidity",
        "Read the real room temperature (Celsius) and relative humidity (%) from the sensor on the robot's base. "
        "Use it whenever the user asks about temperature, humidity, hot/cold, dry/damp, or the room climate.",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue {
            float t = 0, hum = 0;
            if (!aht.Read(t, hum)) return std::string("{\"error\":\"sensor read failed\"}");
            char out[96];
            snprintf(out, sizeof(out), "{\"temperature_c\":%.1f,\"humidity_percent\":%.0f}", t, hum);
            return std::string(out);
        });

    mcp.AddTool("self.ir.learn",
        "Learn an infrared remote button. Tell the user to point their real remote at the robot's base and press the button "
        "once within 20 seconds, then call this tool with a short name such as 'tv_power' or 'ac_on'. "
        "Returns immediately; afterwards call self.ir.status to see whether the signal was captured.",
        PropertyList({Property("name", kPropertyTypeString)}),
        [](const PropertyList& p) -> ReturnValue {
            return ir.StartLearn(p["name"].value<std::string>(), 20);
        });

    mcp.AddTool("self.ir.status",
        "Result of the latest self.ir.learn call: 'listening', 'learned: <name>', or an error such as a timeout.",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue { return ir.Status(); });

    mcp.AddTool("self.ir.send",
        "Send a previously learned infrared remote button by name (e.g. turn the TV or air conditioner on/off). "
        "Call self.ir.list first if unsure which names exist.",
        PropertyList({Property("name", kPropertyTypeString)}),
        [](const PropertyList& p) -> ReturnValue {
            return ir.Send(p["name"].value<std::string>());
        });

    mcp.AddTool("self.ir.list",
        "List the names of all learned infrared remote buttons.",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue { return ir.List(); });

    mcp.AddTool("self.ir.delete",
        "Delete a learned infrared remote button by name.",
        PropertyList({Property("name", kPropertyTypeString)}),
        [](const PropertyList& p) -> ReturnValue {
            return ir.Delete(p["name"].value<std::string>());
        });
}

}  // namespace sensor_dock
