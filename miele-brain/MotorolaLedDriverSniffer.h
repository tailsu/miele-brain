#include "esphome.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <sstream>

namespace esphome {

class MC14489 {
public:
    MC14489(uint8_t csPin, GPIOPin* data) : _data(*data), _cs(csPin, INPUT, true) {
    }

    MC14489(const MC14489&) = delete;
    MC14489(MC14489&&) = delete;

    void setup() {
        _cs.setup();
        _cs.attach_interrupt(&handleChipSelect, this, CHANGE);
    }

    static void ICACHE_RAM_ATTR HOT handleChipSelect(MC14489* self) {
        self->select();
    }

    void ICACHE_RAM_ATTR HOT select() {
        _selected = _cs.digital_read();
        if (_selected) {
            _buffer = 0;
            _bits = 0;
        } else {
            switch (_bits) {
                case 8:
                    _ctrlReg = uint8_t(_buffer);
                    ctrlUpdates++;
                    break;
                case 24:
                    _displayReg = _buffer;
                    displayUpdates++;
                    break;
                default:;
                    // glitch
            }
        }
    }

    void ICACHE_RAM_ATTR HOT tick() {
        if (!_selected) {
            return;
        }
        _buffer = (_buffer << 1) + _data.digital_read();
        _bits++;
    }

    uint32_t getDisplayReg() const {
        return _displayReg;
    }

    uint8_t getCtrlReg() const {
        return _ctrlReg;
    }

    volatile uint32_t displayUpdates = 0, ctrlUpdates = 0;

private:
    GPIOPin _data;
    GPIOPin _cs;

    volatile bool _selected = false;

    volatile uint32_t _buffer = 0;
    volatile uint8_t _bits = 0;

    volatile uint32_t _displayReg = 0;
    volatile uint8_t _ctrlReg = 0;

};

class MotorolaLedDriverSniffer : public Component, public esphome::text_sensor::TextSensor {

public:
    MotorolaLedDriverSniffer(text_sensor::TextSensor* timeOutput, text_sensor::TextSensor* stateOutput)
        : _data(14, INPUT) // D5
        , _clk(12, INPUT) // D6
        , _left(4, &_data) // D2
        , _right(5, &_data) // D1
        , _timeOutput(timeOutput)
        , _stateOutput(stateOutput)
    {}

    MotorolaLedDriverSniffer(const MotorolaLedDriverSniffer&) = delete;
    MotorolaLedDriverSniffer(MotorolaLedDriverSniffer&&) = delete;

    float get_setup_priority() const override {
        return esphome::setup_priority::AFTER_CONNECTION;
    }

    static void ICACHE_RAM_ATTR handleClk(MotorolaLedDriverSniffer* self) {
        self->handleClkImpl();
    }

    void ICACHE_RAM_ATTR handleClkImpl() {
        _left.tick();
        _right.tick();
    }

    void setup() override {
        _data.setup();
        _clk.setup();
        _left.setup();
        _right.setup();

        _clk.attach_interrupt(&handleClk, this, RISING);
    }

    std::string formatTime() const {
        std::ostringstream str;

        auto dispreg = _left.getDisplayReg();
        int hours = (dispreg & 0xf);
        int minutes = ((dispreg >> 4) & 0xf) * 10 + ((dispreg >> 8) & 0xf);

        if (hours > 0) {
            str << hours << "h " << minutes << "m";
        } else {
            str << minutes << " min";
        }
        return str.str();
    }

    std::string formatState() const {
        auto ldispreg = _left.getDisplayReg();
        auto lctrlreg = _left.getCtrlReg();

        auto rdispreg = _right.getDisplayReg();
        auto rctrlreg = _right.getCtrlReg();

        if (lctrlreg >= 0x71 && lctrlreg <= 0x77) {
            lctrlreg = 0; // number display
        }
        if (lctrlreg == 0x7e || lctrlreg == 0x7f) {
            lctrlreg = 1; // no display
        }

        std::vector<std::string> states;

        auto progress = ((rdispreg & 0xf000) >> 8) | (rdispreg & 0x7);

        // TODO: more reliable source
        // if (lctrlreg == 0x70) {
        //     states.emplace_back("Door open");
        // }

        switch (progress) {
            case 0x10: states.emplace_back("Pre-washing"); break;
            case 0x20: states.emplace_back("Washing"); break;
            case 0x40: states.emplace_back("Rinsing"); break;
            case 0x80: states.emplace_back("Paused Rinse"); break;
            case 0x01: states.emplace_back("Pumping"); break;
            case 0x02: states.emplace_back("Centrifuge"); break;
            case 0x04: states.emplace_back("Finished"); break;
            case 0: states.emplace_back("Idle"); break;
        }

        auto centrifugeSetting = (rdispreg & 0x7e0000) >> 16;
        switch (centrifugeSetting) {
            case 0x02: states.emplace_back("Ø 1600"); break;
            case 0x04: states.emplace_back("Ø 1400"); break;
            case 0x08: states.emplace_back("Ø 1200"); break;
            case 0x50: states.emplace_back("Ø 900"); break;
            case 0x40: states.emplace_back("Ø 600"); break;
            case 0x30: states.emplace_back("Ø 400"); break;
            case 0x20: states.emplace_back("Ø Rinse-pause"); break;
            case 0x10: states.emplace_back("Ø No"); break;
        }

        // TODO: debounce blinking
        // if (rdispreg & 0x800) {
        //     states.emplace_back("Delayed start");
        //     // TODO: if delayed start and progress is 0, then read the delayed start setting off of ldispreg
        // }
        // if (rdispreg & 0x200) {
        //     states.emplace_back("Soak");
        // }

        if (rdispreg & 0x100) {
            states.emplace_back("Pre-wash");
        }
        if (rdispreg & 0x40) {
            states.emplace_back("Short");
        }
        if (rdispreg & 0x80) {
            states.emplace_back("Wasser Plus");
        }
        if (rdispreg & 0x010000) {
            states.emplace_back("Summer");
        }

        std::ostringstream str;
        for (auto& state : states) {
            if (str.tellp() > 0) {
                str << ", ";
            }
            str << state;
        }

        // str << std::hex << "R:" << rdispreg << "," << int(rctrlreg) << " L:" << ldispreg << "," << int(lctrlreg);
        return str.str();
    }

    static void publishNew(text_sensor::TextSensor* sensor, std::string newState) {
        if (sensor->state != newState) {
            sensor->publish_state(std::move(newState));
        }
    }

    void loop() override {
        publishNew(_timeOutput, formatTime());
        publishNew(_stateOutput, formatState());

        // ESP_LOGI("sniffer", "Left: display=%x ctrl=%x DU=%d CU=%d", _left.getDisplayReg(), _left.getCtrlReg(), _left.displayUpdates, _left.ctrlUpdates);
        // ESP_LOGI("sniffer", "Right: display=%x ctrl=%x  DU=%d CU=%d", _right.getDisplayReg(), _right.getCtrlReg(), _right.displayUpdates, _right.ctrlUpdates);
    }

    GPIOPin _data, _clk;
    MC14489 _left, _right;

    text_sensor::TextSensor* _timeOutput;
    text_sensor::TextSensor* _stateOutput;
};

}
